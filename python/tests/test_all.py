import math, sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from earth_core import *
from earth_core.geo import A
from building_pipeline.stage_a import build, signed_area, triangulate
from building_pipeline.passport import Status, Origin
from world_streamer.cell_state import Cell, S

MINSK = (53.9045, 27.5615)   # approx. centre; test location only

def test_ecef_equator():
    x, y, z = geodetic_to_ecef(0, 0, 0)
    assert abs(x - A) < 1e-6 and abs(y) < 1e-6 and abs(z) < 1e-6

def test_ecef_roundtrip():
    for lat, lon, h in [(*MINSK, 200), (-33.9, 151.2, 5), (0.0, 0.0, 0), (89.0, 10, 100)]:
        la, lo, hh = ecef_to_geodetic(*geodetic_to_ecef(lat, lon, h))
        assert abs(la - lat) < 1e-9 and abs(lo - lon) < 1e-9 and abs(hh - h) < 1e-5

def test_enu_roundtrip_and_origin():
    f = LocalFrame(*MINSK)
    assert max(abs(v) for v in f.to_local(*MINSK)) < 1e-6
    e, n, u = f.to_local(MINSK[0] + 0.05, MINSK[1] - 0.07, 30)
    la, lo, h = f.from_local(e, n, u)
    assert abs(la - MINSK[0] - 0.05) < 1e-9 and abs(lo - MINSK[1] + 0.07) < 1e-9 and abs(h - 30) < 1e-5

def test_utm_known_values():
    assert utm_zone(MINSK[1]) == 35
    _, e, n = utm_forward(0.0, 27.0, 35)
    assert abs(e - 500000) < 1e-6 and abs(n) < 1e-6
    # meridian arc length to 45N = 4,984,944.4 m (textbook), times k0
    _, e, n = utm_forward(45.0, 27.0, 35)
    assert abs(n - 4984944.38 * 0.9996) < 1.0, n

def test_utm_roundtrip():
    z, e, n = utm_forward(*MINSK)
    la, lo = utm_inverse(z, e, n)
    assert abs(la - MINSK[0]) < 1e-8 and abs(lo - MINSK[1]) < 1e-8  # ~1 mm; measured 0.7 mm (3-term series)

def test_utm_vs_enu_distance_agree():
    a = MINSK; b = (MINSK[0] + 0.012, MINSK[1] + 0.02)
    _, e1, n1 = utm_forward(*a); _, e2, n2 = utm_forward(*b)
    d_utm = math.hypot(e2 - e1, n2 - n1)
    e, n, _ = LocalFrame(*a).to_local(*b)
    assert abs(d_utm / math.hypot(e, n) - 1) < 6e-4   # UTM scale ~1.0000x..1.0004 here

def test_cells():
    c = cell_of(*MINSK)
    assert c == cell_of(*MINSK) and c.zone == 35
    e0, n0, e1, n1 = cell_bounds(c)
    _, e, n = utm_forward(*MINSK)
    assert e0 <= e < e1 and n0 <= n < n1
    assert len(set(neighbors(c))) == 8 and c not in neighbors(c)
    la, lo = cell_center_latlon(c)
    assert cell_of(la, lo) == c

def _sq(f, side=20.0):
    return [f.from_local(x, y, 0)[:2] for x, y in [(0, 0), (side, 0), (side, side), (0, side)]]

def test_building_box_and_provenance():
    f = LocalFrame(*MINSK)
    p, v, i = build("osm:way/1", _sq(f), {"building": "apartments", "building:levels": "9"}, f)
    assert p.height_m.value == 27.0 and p.height_m.origin is Origin.ESTIMATED
    assert p.status is Status.PROCEDURAL_APPROXIMATION and p.geometry_confidence == "footprint_only"
    assert len(i) == 4 * 2 + 2 and max(z for *_, z in v) == 27.0

def test_default_height_marked():
    f = LocalFrame(*MINSK)
    p, *_ = build("osm:way/2", _sq(f), {"building": "yes"}, f)
    assert p.height_m.origin is Origin.DEFAULT

def test_roof_area_concave():
    L = [(0,0),(10,0),(10,4),(4,4),(4,10),(0,10)]
    t = triangulate(L)
    assert abs(sum(abs(signed_area([L[a], L[b], L[c]])) for a, b, c in t) - 64) < 1e-9


def test_triangulation_fuzz_collinear_and_duplicates():
    """Star-shaped (hence simple) polygons with injected collinear midpoints and duplicate vertices, as in real OSM."""
    import random
    rnd = random.Random(3); tested = fallback = bad = 0
    while tested < 2000:
        n = rnd.randint(3, 14); angs = sorted(rnd.uniform(0, 2 * math.pi) for _ in range(n))
        gaps = [(angs[(i + 1) % n] - angs[i]) % (2 * math.pi) for i in range(n)]
        if max(gaps) >= math.pi - 0.1: continue                    # keep it star-shaped about the origin => simple
        pts = [(math.cos(a) * r, math.sin(a) * r) for a in angs for r in [rnd.uniform(3, 10)]]
        out = []
        for i, p in enumerate(pts):
            out.append(p); q = pts[(i + 1) % n]
            if rnd.random() < .4: out.append(((p[0] + q[0]) / 2, (p[1] + q[1]) / 2))
            if rnd.random() < .1: out.append(p)
        tested += 1
        t = triangulate(out)
        if t is None: fallback += 1; continue
        tri = [signed_area([out[a], out[b], out[c]]) for a, b, c in t]
        if min(tri) < -1e-9 or abs(sum(tri) - signed_area(out)) > 1e-6: bad += 1
    assert bad == 0, bad
    assert fallback <= tested * 0.005, fallback      # fallbacks must be rare, never silently wrong

def test_bad_polygon_does_not_crash_and_is_not_overclaimed():
    f = LocalFrame(*MINSK)
    bow = [f.from_local(x, y, 0)[:2] for x, y in [(0,0),(10,10),(10,0),(0,10)]]
    p, v, i = build("osm:way/3", bow, {"building": "yes"}, f)
    assert p.geometry_confidence == "footprint_only"

def test_converter_on_overpass_shaped_data():
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))
    from osm_to_scene import convert
    la, lo = MINSK
    def g(dx, dy):                      # offsets in degrees ~ tens of metres
        return {"lat": la + dy * 1e-5, "lon": lo + dx * 1.7e-5}
    sq = [g(5, 5), g(25, 5), g(25, 20), g(5, 20), g(5, 5)]
    data = {"version": 0.6, "elements": [
        {"type": "way", "id": 1, "bounds": {}, "nodes": [10, 11, 12], "geometry": [g(-40, 0), g(0, 0), g(40, 0)],
         "tags": {"highway": "primary", "lanes": "4", "name": "x"}},
        {"type": "way", "id": 2, "nodes": [20, 11, 22], "geometry": [g(0, -40), g(0, 0), g(0, 40)], "tags": {"highway": "residential"}},
        {"type": "way", "id": 3, "nodes": [30, 31], "geometry": [g(0, 10), g(10, 10)], "tags": {"highway": "footway"}},   # not drivable
        {"type": "way", "id": 4, "geometry": sq, "tags": {"building": "apartments", "building:levels": "9", "height": "27.5 m"}},
        {"type": "way", "id": 5, "geometry": sq[:4], "tags": {"building": "yes"}},                                        # not closed -> skipped
        {"type": "way", "id": 6, "geometry": [g(30, 30), g(40, 30), g(40, 40), g(30, 40), g(30, 30)], "tags": {"building": "house", "height": "abc"}},
        {"type": "node", "id": 7, **g(8, -8), "tags": {"natural": "tree"}},
    ]}
    sc = convert(data, la, lo, 500)
    m = sc["meta"]
    assert (m["n_roads"], m["n_buildings"], m["n_trees"]) == (2, 2, 1)
    w = {r["cls"]: r for r in sc["roads"]}
    assert abs(w["primary"]["w"] - 13.2) < 1e-9 and w["primary"]["w_src"] == "tag" and w["residential"]["w_src"] == "default"
    assert any(w["primary"]["junction"]) and any(w["residential"]["junction"])      # shared node 11 is a junction
    b = {x["id"]: x for x in sc["buildings"]}
    assert b["osm:way/4"]["h"] == 27.5 and b["osm:way/4"]["h_src"] == "tag"
    assert b["osm:way/6"]["h_src"] in ("default", "estimated")                       # bad height tag must not crash or be trusted
    assert m["height_origin_counts"]["tag"] == 1

def test_cell_state_machine():
    c = Cell("x")
    for s in (S.FETCHING, S.BASIC_READY, S.REFINING, S.DETAILED_READY, S.CACHED): c.go(s)
    try: c.go(S.FETCHING); assert False
    except ValueError: pass
    d = Cell("y"); d.go(S.FETCHING); d.go(S.ERROR); d.go(S.FETCHING)
    assert d.retries == 1

if __name__ == "__main__":
    fails = 0
    for k, fn in list(globals().items()):
        if k.startswith("test_"):
            try: fn(); print("PASS", k)
            except Exception as ex: fails += 1; print("FAIL", k, repr(ex))
    sys.exit(1 if fails else 0)
