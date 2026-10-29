#include "terrain.hpp"
#include <algorithm>

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#define STBI_NO_STDIO
#include "../../third_party/stb_image.h"

namespace ec {

bool terrarium_decode(const std::string &png, std::vector<float> &out, std::string &err) {
    int w = 0, h = 0, ch = 0;
    unsigned char *px = stbi_load_from_memory((const unsigned char *)png.data(), (int)png.size(), &w, &h, &ch, 3);
    if (!px) {
        err = std::string("png: ") + (stbi_failure_reason() ? stbi_failure_reason() : "?");
        return false;
    }
    if (w != 256 || h != 256) {
        stbi_image_free(px);
        err = "terrarium tile is not 256x256";
        return false;
    }
    out.resize(256 * 256);
    for (int i = 0; i < 256 * 256; i++) out[i] = float(px[i * 3] * 256.0 + px[i * 3 + 1] + px[i * 3 + 2] / 256.0 - 32768.0);
    stbi_image_free(px);
    return true;
}

void Dem::add(int x, int y, std::vector<float> h) {
    std::lock_guard<std::mutex> l(mu_);
    tiles_[TileId{DEM_ZOOM, x, y}.key()] = std::make_shared<const std::vector<float>>(std::move(h));
}
bool Dem::has(int x, int y) const {
    std::lock_guard<std::mutex> l(mu_);
    return tiles_.count(TileId{DEM_ZOOM, x, y}.key()) > 0;
}

double Dem::texel(int gx, int gy, bool &ok) const {
    int tx = gx >> 8, ty = gy >> 8;
    auto it = tiles_.find(TileId{DEM_ZOOM, tx, ty}.key());
    if (it == tiles_.end()) {
        ok = false;
        return 0;
    }
    return (*it->second)[(gy & 255) * 256 + (gx & 255)];
}

double Dem::height(double lat, double lon, double fallback) const {
    double n = std::ldexp(1.0, DEM_ZOOM) * 256;
    double fx = (lon + 180) / 360 * n - 0.5;
    double latr = lat * PI / 180;
    double fy = (1 - std::asinh(std::tan(latr)) / PI) / 2 * n - 0.5;
    int ix = (int)std::floor(fx), iy = (int)std::floor(fy);
    double tx = fx - ix, ty = fy - iy;
    // Catmull-Rom bicubic over 4x4 texels
    auto w = [](double t, double *o) {
        double t2 = t * t, t3 = t2 * t;
        o[0] = -0.5 * t3 + t2 - 0.5 * t;
        o[1] = 1.5 * t3 - 2.5 * t2 + 1;
        o[2] = -1.5 * t3 + 2 * t2 + 0.5 * t;
        o[3] = 0.5 * t3 - 0.5 * t2;
    };
    double wx[4], wy[4];
    w(tx, wx);
    w(ty, wy);
    std::lock_guard<std::mutex> l(mu_);
    double s = 0;
    bool ok = true;
    for (int j = 0; j < 4; j++)
        for (int i = 0; i < 4; i++) s += wx[i] * wy[j] * texel(ix - 1 + i, iy - 1 + j, ok);
    return ok ? s : fallback;
}

std::vector<TileId> Dem::tiles_for(const Bounds &b) {
    TileId a = tile_of(b.n, b.w, DEM_ZOOM), c = tile_of(b.s, b.e, DEM_ZOOM);
    std::vector<TileId> out;
    // one texel of margin for the bicubic kernel: widen by a tile when close to an edge is cheap enough
    for (int y = a.y - 1; y <= c.y + 1; y++)
        for (int x = a.x - 1; x <= c.x + 1; x++) {
            Bounds tb = tile_bounds({DEM_ZOOM, x, y});
            double mlat = (tb.n - tb.s) * 4 / 256, mlon = (tb.e - tb.w) * 4 / 256;
            if (tb.s - mlat > b.n || tb.n + mlat < b.s || tb.w - mlon > b.e || tb.e + mlon < b.w) continue;
            out.push_back({DEM_ZOOM, x, y});
        }
    return out;
}

double HeightGrid::sample(double x, double y) const {
    double fx = std::clamp((x - x0) / cell, 0.0, (double)n), fy = std::clamp((y - y0) / cell, 0.0, (double)n);
    int i = std::min((int)fx, n - 1), j = std::min((int)fy, n - 1);
    double tx = fx - i, ty = fy - j;
    // same split as the render/collision triangles: diagonal from (i,j) to (i+1,j+1)
    double h00 = at(i, j), h10 = at(i + 1, j), h01 = at(i, j + 1), h11 = at(i + 1, j + 1);
    if (tx >= ty) return h00 + (h10 - h00) * tx + (h11 - h10) * ty;
    return h00 + (h11 - h01) * tx + (h01 - h00) * ty;
}

V3 HeightGrid::normal(double x, double y) const {
    double e = cell;
    double dx = sample(x + e, y) - sample(x - e, y), dy = sample(x, y + e) - sample(x, y - e);
    return V3(-dx / (2 * e), -dy / (2 * e), 1).norm();
}

}  // namespace ec
