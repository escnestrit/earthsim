// Quality checks over real data. Run: es_tests <data dir> [radius_chunks]
// Every check prints a line; the exit code is non-zero if any hard limit is broken.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include "world.hpp"

using namespace es;
using clk = std::chrono::steady_clock;

static int failures = 0;
static void check(bool ok, const char *name, const std::string &detail) {
    printf("[%s] %-34s %s\n", ok ? "PASS" : "FAIL", name, detail.c_str());
    if (!ok) failures++;
}
static std::string fmt(const char *f, double a = 0, double b = 0, double c = 0, double d = 0) {
    char buf[256];
    snprintf(buf, sizeof buf, f, a, b, c, d);
    return buf;
}

int main(int argc, char **argv) {
    const char *root = argc > 1 ? argv[1] : "data";
    int R = argc > 2 ? atoi(argv[2]) : 3;
    World W;
    auto t0 = clk::now();
    if (!W.load(root)) {
        for (auto &l : W.log) printf("%s\n", l.c_str());
        return 2;
    }
    double load_ms = std::chrono::duration<double, std::milli>(clk::now() - t0).count();
    check(load_ms < 8000, "world load time", fmt("%.0f ms", load_ms));
    check(W.net.edges.size() > 1000, "road graph size", fmt("%.0f edges, %.0f nodes", W.net.edges.size(), W.net.nodes.size()));

    // ---- road model
    {
        size_t samples = 0, steep = 0;
        for (auto &e : W.net.edges)
            for (size_t k = 0; k + 1 < e.ps.size(); k++) {
                samples++;
                if (std::fabs(e.pz[k + 1] - e.pz[k]) / std::max(0.1, e.ps[k + 1] - e.ps[k]) > 0.14) steep++;
            }
        check(steep < samples * 0.01, "road grade > 14%", fmt("%.0f of %.0f samples", steep, samples));
        // node heights equal the profile ends
        int bad = 0;
        for (auto &e : W.net.edges)
            if (std::fabs(e.pz.front() - W.net.nodes[e.a].z) > 1e-6 || std::fabs(e.pz.back() - W.net.nodes[e.b].z) > 1e-6) bad++;
        check(bad == 0, "profiles meet at junctions", fmt("%.0f mismatches", bad));
        // junction outlines: simple polygons
        int self_x = 0, njs = 0;
        for (auto &J : W.geom.junctions) {
            if (J.arms.size() < 3 || J.flat) continue;
            njs++;
            auto &o = J.outline;
            bool x = false;
            for (size_t i = 0; i < o.size() && !x; i++)
                for (size_t j = i + 2; j < o.size() && !x; j++) {
                    if (i == 0 && j == o.size() - 1) continue;
                    double t, u;
                    if (seg_intersect(o[i].xy(), o[(i + 1) % o.size()].xy(), o[j].xy(), o[(j + 1) % o.size()].xy(), t, u) && t > 1e-6 && t < 1 - 1e-6 && u > 1e-6 && u < 1 - 1e-6) x = true;
                }
            self_x += x;
        }
        check(self_x < njs * 0.03, "junction outlines self-intersect", fmt("%.0f of %.0f", self_x, njs));
        // dual carriageways: the two directions at about the same height
        {
            size_t n = 0, big = 0;
            double worst = 0;
            V2 wat;
            for (auto &e : W.net.edges) {
                if (e.twins.empty() || e.bridge || e.tunnel || e.level) continue;
                for (double s = 5; s < e.len - 5; s += 10) {
                    V2 p = e.at(s);
                    double z = e.z_at(s);
                    for (int t : e.twins) {
                        auto &f = W.net.edges[t];
                        if (f.bridge || f.tunnel || f.level) continue;
                        double bd = 1e9, acc = 0, arc = 0;
                        for (size_t k = 0; k + 1 < f.pts.size(); k++) {
                            double u, sl = dist(f.pts[k], f.pts[k + 1]);
                            double d = seg_dist(p, f.pts[k], f.pts[k + 1], &u);
                            if (d < bd) bd = d, arc = acc + u * sl;
                            acc += sl;
                        }
                        if (bd > 40 || arc < 1 || arc > f.len - 1) continue;
                        double dz = std::fabs(f.z_at(arc) - z);
                        n++;
                        if (dz > 0.8) big++;
                        if (dz > worst) worst = dz, wat = p;
                    }
                }
            }
            check(big <= n / 100, "twin carriageway height gap > 0.8 m", fmt("%.0f of %.0f samples, worst %.2f m at (%.0f,", big, n, worst, wat.x) + fmt("%.0f)", wat.y));
        }
        // traffic signals only on arms with arriving traffic is enforced by construction; count heads
    }

    // ---- chunks around the centre
    int cx0 = 0, cy0 = 0;
    if (argc > 4) cx0 = atoi(argv[3]), cy0 = atoi(argv[4]);
    double worst_ms = 0, sum_ms = 0;
    size_t n_chunks = 0, road_pts = 0, road_poke = 0, pave_pts = 0, pave_poke = 0, kerb_pts = 0, kerb_bad = 0;
    size_t bld = 0, bld_low = 0, bld_float = 0, signals = 0, props[I_COUNT] = {0}, props_bad = 0, fence_tris = 0;
    double worst_poke = 0, worst_low = 0;
    V2 worst_poke_at, worst_low_at;
    std::vector<std::unique_ptr<ChunkData>> chunks;
    for (int cy = cy0 - R; cy <= cy0 + R; cy++)
        for (int cx = cx0 - R; cx <= cx0 + R; cx++) {
            auto t = clk::now();
            auto c = W.build_chunk(cx, cy);
            double ms = std::chrono::duration<double, std::milli>(clk::now() - t).count();
            worst_ms = std::max(worst_ms, ms), sum_ms += ms, n_chunks++;
            signals += c->inst[I_SIGNAL].size();
            Rect r = W.chunk_rect(cx, cy);
            RoadGeom::Cache rc;
            W.geom.prefetch(r.grown(40), rc);
            fence_tris += c->mats[M_FENCE].idx.size() / 3;
            for (int k : {I_CAR, I_BIN, I_STOP, I_PLAY})
                for (auto &I : c->inst[k]) {
                    props[k]++;
                    SurfaceHit h = W.geom.query({I.p.x + c->origin.x, I.p.y + c->origin.y}, rc, 0);
                    if (h.kind == 1 && !h.margin) props_bad++;
                }
            // dense sampling: terrain triangles never above carriageway / pavement surfaces
            for (double y = r.mn.y + 0.37; y < r.mx.y; y += 1.13)
                for (double x = r.mn.x + 0.29; x < r.mx.x; x += 1.13) {
                    V2 p{x, y};
                    SurfaceHit h = W.geom.query(p, rc, 0);
                    if (!h.kind || h.bridge) continue;
                    double g = World::grid_height(*c, p);
                    if (std::isnan(g)) continue;
                    // only the true surface (not the 1 m margin): check the road surface really is there
                    if (h.kind == 1) {
                        road_pts++;
                        if (g > h.z - 0.02) {
                            road_poke++;
                            if (g - h.z > worst_poke) worst_poke = g - h.z, worst_poke_at = p;
                        }
                    } else {
                        pave_pts++;
                        if (g > h.z - 0.01) pave_poke++;
                    }
                }
            // kerbs: pavement is KERB_H above the carriageway next to it
            for (int ei : rc.edges) {
                auto &e = W.net.edges[ei];
                if (!e.sidewalk || e.bridge || e.tunnel || e.level) continue;
                V2 mid = e.at(e.len * 0.5);
                if (!r.contains(mid)) continue;
                for (double s = e.trim_a + 2; s < e.len - e.trim_b - 2; s += 7) {
                    V2 d;
                    V2 p = e.at(s, &d);
                    for (double side : {-1.0, 1.0}) {
                        if (e.sw(side) <= 0) continue;
                        V2 q = p + perp(d) * (side * (e.half() + e.sw(side) * 0.5));
                        SurfaceHit h = W.geom.query(q, rc, 0);
                        if (h.kind != 2) continue;
                        kerb_pts++;
                        if (std::fabs(h.z - (e.z_at(s) + KERB_H)) > 0.05) {
                            if (kerb_bad < 4) printf("   kerb sample (%.1f,%.1f) edge %d: pavement %.2f road %.2f\n", q.x, q.y, ei, h.z, e.z_at(s));
                            kerb_bad++;
                        }
                    }
                }
            }
            chunks.push_back(std::move(c));
        }
    check(props_bad == 0 && props[I_CAR] > 50, "props off the carriageway", fmt("%.0f parked cars, ", props[I_CAR]) + fmt("%.0f container sites, ", props[I_BIN]) + fmt("%.0f shelters, ", props[I_STOP]) + fmt("%.0f playground items, ", props[I_PLAY]) + fmt("%.0f fence tris, ", fence_tris) + fmt("%.0f in a carriageway", props_bad));
    check(sum_ms / n_chunks < 150 && worst_ms < 450, "chunk build time", fmt("worst %.0f ms, mean %.0f ms over %.0f chunks", worst_ms, sum_ms / n_chunks, n_chunks));
    check(road_poke < road_pts * 0.002, "terrain under carriageway", fmt("%.0f of %.0f samples poke through, worst %.2f m at (%.0f,", road_poke, road_pts, worst_poke, worst_poke_at.x) + fmt("%.0f)", worst_poke_at.y));
    check(pave_poke < pave_pts * 0.005, "terrain under pavement", fmt("%.0f of %.0f samples poke through", pave_poke, pave_pts));
    check(kerb_bad <= kerb_pts / 500, "kerb height", fmt("%.0f of %.0f samples off by > 5 cm", kerb_bad, kerb_pts));
    check(signals > 0, "traffic signal heads", fmt("%.0f placed", signals));

    // ---- buildings: never below the pavement next to them, never floating over the ground
    {
        for (int cy = cy0 - R; cy <= cy0 + R; cy++)
            for (int cx = cx0 - R; cx <= cx0 + R; cx++) {
                auto it = W.chunk_buildings.find(World::ckey(cx, cy));
                if (it == W.chunk_buildings.end()) continue;
                for (int bi : it->second) {
                    auto &b = W.buildings[bi];
                    if (!b.render || b.parent >= 0 || b.min_h > 0.3) continue;
                    RoadGeom::Cache rc;
                    W.geom.prefetch(b.box.grown(40), rc);
                    double lowest;
                    double base = W.building_base(b, rc, &lowest);
                    bld++;
                    auto &rr = W.region.buildings[b.src];
                    double low = 0;
                    for (auto &ring : rr.rings) {
                        auto &P = ring.outer;
                        for (size_t i = 0; i < P.size(); i++) {
                            V2 a = P[i], c2 = P[(i + 1) % P.size()];
                            V2 m = (a + c2) * 0.5, out = perp(c2 - a) * -1.0;
                            out = norm(out);
                            SurfaceHit h = W.geom.query(m + out * 0.8, rc, 0);
                            if (h.kind == 2 && !h.bridge) low = std::max(low, h.z - base);
                        }
                    }
                    if (low > 0.05) {
                        bld_low++;
                        if (low > worst_low) worst_low = low, worst_low_at = b.centroid;
                    }
                    // floating: the lowest wall bottom (base - 1.5) must be under the lowest ground
                    double gmin = 1e9;
                    for (auto &ring : rr.rings)
                        for (auto p : ring.outer) gmin = std::min(gmin, W.terrain_at(p, rc));
                    double foot = std::min(base - 1.0, lowest - 0.6);
                    if (foot > gmin + 0.01) bld_float++;
                }
            }
        check(bld_low == 0, "buildings below adjacent pavement", fmt("%.0f of %.0f, worst %.2f m", bld_low, bld, worst_low));
        check(bld_float < bld * 0.01, "buildings with visible gap below", fmt("%.0f of %.0f", bld_float, bld));
    }
    {
        // architecture: the historic centre is old / Stalinist, not khrushchevkas; residential districts have both
        double era_c[16] = {0}, era_o[16] = {0}, nc = 0, no = 0, att = 0;
        for (auto &b : W.buildings) {
            if (b.parent >= 0 || b.area < 60) continue;
            bool c = len(b.centroid) < 1200;
            (c ? era_c : era_o)[b.era]++;
            (c ? nc : no)++;
            att += b.attached;
        }
        auto pc = [](double a, double n) { return n > 0 ? 100.0 * a / n : 0.0; };
        printf("   centre (%.0f): historic %.0f%% stalin %.0f%% khrush %.0f%% panel %.0f%% modern %.0f%% office %.0f%% public %.0f%%\n", nc, pc(era_c[E_HISTORIC], nc), pc(era_c[E_STALIN], nc),
               pc(era_c[E_KHRUSH], nc), pc(era_c[E_PANEL], nc), pc(era_c[E_MODERN], nc), pc(era_c[E_OFFICE], nc), pc(era_c[E_PUBLIC], nc));
        printf("   outside (%.0f): historic %.0f%% stalin %.0f%% khrush %.0f%% panel %.0f%% modern %.0f%% house %.0f%% shed %.0f%% indus %.0f%%\n", no, pc(era_o[E_HISTORIC], no), pc(era_o[E_STALIN], no),
               pc(era_o[E_KHRUSH], no), pc(era_o[E_PANEL], no), pc(era_o[E_MODERN], no), pc(era_o[E_HOUSE], no), pc(era_o[E_SHED], no), pc(era_o[E_INDUSTRIAL], no));
        if (nc > 50) check(era_c[E_KHRUSH] + era_c[E_PANEL] < 0.25 * nc, "centre is not prefab housing", fmt("%.0f%% khrushchevka/panel", pc(era_c[E_KHRUSH] + era_c[E_PANEL], nc)));
    }
    printf("%s: %d failed\n", failures ? "FAILED" : "OK", failures);
    return failures ? 1 : 0;
}
