#include "textures.hpp"
#include <algorithm>
#include <cmath>
#include <functional>

namespace ec {

namespace {

constexpr int TEX = FACADE_TEX, BAYS = 4, BAY = TEX / BAYS;
constexpr float WALL_D = 0.1f;

struct Px {
    float c[3];
    float rough = 0.9f, metal = 0, lit = 0, depth = WALL_D;
};

float fract(float x) { return x - std::floor(x); }

float hsh(uint32_t a, uint32_t b) {
    uint32_t h = a * 0x9E3779B1u ^ b * 0x85EBCA77u;
    h ^= h >> 15;
    h *= 0x2C1B3C6Du;
    h ^= h >> 12;
    return float(h & 0xFFFF) / 65535.0f;
}

Px mk(float r, float g, float b, float rough, float metal, float lit, float depth) {
    Px p;
    p.c[0] = r, p.c[1] = g, p.c[2] = b;
    p.rough = rough, p.metal = metal, p.lit = lit, p.depth = depth;
    return p;
}

using WallFn = std::function<Px(float, float, int, int)>;

Px plaster(float x, float y, int, int) {
    float n = hsh((uint32_t)(x * 40), (uint32_t)(y * 40)) * 0.04f;
    float band = y < 0.12f ? 0.88f : 0.97f;
    return mk(band - n, band - n, band - n, 0.9f, 0, 0, WALL_D);
}
Px brick(float x, float y, int, int) {
    uint32_t course = (uint32_t)(y / 0.075f);
    float off = course % 2 == 0 ? 0 : 0.125f;
    float bx = fract((x + off) / 0.25f), by = fract(y / 0.075f);
    bool mortar = bx < 0.05f || by < 0.13f;
    float v = hsh((uint32_t)((x + off) / 0.25f), course) * 0.18f;
    if (mortar) return mk(1.25f, 1.2f, 1.1f, 0.95f, 0, 0, 0.22f);
    return mk(0.92f - v, 0.9f - v, 0.9f - v, 0.85f, 0, 0, WALL_D);
}
Px panel(float x, float y, int bx, int by) {
    bool seam = x < 0.03f || y < 0.03f;
    float n = hsh(bx * 7 + (uint32_t)(x * 8), by * 13 + (uint32_t)(y * 8)) * 0.06f;
    float v = seam ? 0.55f : 0.95f - n;
    return mk(v, v, v, 0.92f, 0, 0, seam ? 0.35f : WALL_D);
}
Px glass(float x, float y, int, int) {
    bool mull = fract(x / 1.5f) < 0.03f || y < 0.04f;
    if (mull) return mk(0.55f, 0.57f, 0.6f, 0.4f, 0.8f, 0, WALL_D);
    if (y < 0.7f) return mk(0.35f, 0.38f, 0.42f, 0.3f, 0.2f, 0, WALL_D);
    return mk(0.27f, 0.35f, 0.43f, 0.08f, 0, 1, WALL_D);
}
Px wood(float, float y, int, int by) {
    float board = fract(y / 0.18f);
    float v = 0.9f - hsh((uint32_t)(y / 0.18f), by) * 0.15f - (board < 0.08f ? 0.3f : 0);
    return mk(v, v, v, 0.85f, 0, 0, board < 0.08f ? 0.2f : WALL_D);
}
Px metal(float x, float y, int, int) {
    float rib = std::abs(fract(x / 0.2f) - 0.5f) * 2;
    float v = 0.82f + rib * 0.15f - (y < 0.05f ? 0.2f : 0);
    return mk(v, v, v, 0.45f, 0.6f, 0, WALL_D + (1 - rib) * 0.1f);
}
Px tile_panel(float x, float y, int bx, int by) {
    bool seam = x < 0.03f || y < 0.03f;
    float tx = fract(x / 0.12f), ty = fract(y / 0.12f);
    bool grout = tx < 0.12f || ty < 0.12f;
    float n = hsh((uint32_t)(x / 0.12f) + bx * 97, (uint32_t)(y / 0.12f) + by * 89) * 0.08f;
    float v = seam ? 0.5f : grout ? 0.8f : 0.97f - n;
    return mk(v, v, v, grout ? 0.95f : 0.35f, 0, 0, seam ? 0.35f : grout ? 0.16f : WALL_D);
}
Px silicate(float x, float y, int bx, int by) {
    uint32_t course = (uint32_t)(y / 0.088f);
    float off = course % 2 == 0 ? 0 : 0.125f;
    float fx = fract((x + off) / 0.25f), fy = fract(y / 0.088f);
    bool mortar = fx < 0.04f || fy < 0.11f;
    float v = hsh((uint32_t)((x + off) / 0.25f) + bx * 7, course + by * 31) * 0.07f;
    if (mortar) return mk(0.8f, 0.79f, 0.76f, 0.95f, 0, 0, 0.2f);
    return mk(0.97f - v, 0.96f - v, 0.93f - v, 0.85f, 0, 0, WALL_D);
}
Px glass_ribbon(float x, float y, int bx, int by) {
    if (y < 1.0f) {
        float v = 0.9f - hsh(bx, by) * 0.03f;
        return mk(v, v, v, 0.6f, 0.1f, 0, WALL_D);
    }
    return glass(x, y, bx, by);
}
Px rustic(float x, float y, int bx, int by) {
    uint32_t course = (uint32_t)(y / 0.375f);
    float off = course % 2 == 0 ? 0 : 0.6f;
    float fx = fract((x + off) / 1.2f), fy = fract(y / 0.375f);
    bool joint = fy < 0.07f || fx < 0.025f;
    float v = 0.95f - hsh((uint32_t)((x + off) / 1.2f) + bx * 13, course + by * 7) * 0.05f;
    if (joint) return mk(0.7f, 0.7f, 0.7f, 0.95f, 0, 0, 0.3f);
    return mk(v, v, v, 0.85f, 0, 0, WALL_D);
}
Px sandwich(float x, float y, int, int) {
    float fy = fract(y);
    bool rib = fract(fy * 4) < 0.08f;
    bool seam = fy < 0.02f || fract(x / 6) < 0.004f;
    float v = seam ? 0.7f : rib ? 0.88f : 0.97f;
    return mk(v, v, v, 0.5f, 0.3f, 0, seam ? 0.3f : rib ? 0.15f : WALL_D);
}
Px alu_panel(float x, float y, int bx, int by) {
    bool joint = fract(x / 1.5f) < 0.012f || fract(y) < 0.018f;
    float v = 0.93f - hsh(bx * 3 + (uint32_t)(x / 1.5f), by * 5 + (uint32_t)y) * 0.025f;
    if (joint) return mk(0.35f, 0.36f, 0.38f, 0.6f, 0.2f, 0, 0.25f);
    return mk(v, v, v * 1.01f, 0.35f, 0.55f, 0, WALL_D);
}
Px stone_slab(float x, float y, int bx, int by) {
    bool joint = fract(x / 1.5f) < 0.01f || fract(y / 0.75f) < 0.02f;
    float v = 0.92f - hsh(bx * 7 + (uint32_t)(x / 1.5f), by * 11 + (uint32_t)(y / 0.75f)) * 0.08f + hsh((uint32_t)(x * 30), (uint32_t)(y * 30)) * 0.02f;
    if (joint) return mk(0.6f, 0.6f, 0.6f, 0.5f, 0, 0, 0.2f);
    return mk(v, v * 0.98f, v * 0.95f, 0.5f, 0, 0, WALL_D);
}

struct Win {
    float x0, x1, y0, y1;
};
struct Style {
    WallFn wall;
    std::vector<Win> windows;
    float frame[3];
    bool loggia = false;
    float l0 = 0, l1 = 0;
    float pilaster = 0;
    bool arch = false;
    int panes = 2;
    bool doors = false, no_sill = false;
};

Style st(WallFn w, std::vector<Win> win, float r, float g, float b) {
    Style s;
    s.wall = w;
    s.windows = std::move(win);
    s.frame[0] = r, s.frame[1] = g, s.frame[2] = b;
    return s;
}

Style style(int si, int v) {
    const float D = 0.25f, W = 0.95f, G = 0.33f;
    Style s;
    switch (si) {
        case 12:
            return v == 0 ? st(stone_slab, {}, D, D, D) : v == 1 ? st(panel, {}, D, D, D) : st(brick, {}, D, D, D);
        case 8:
            if (v == 0) s = st(plaster, {{1.0f, 2.0f, 0.6f, 2.65f}}, W, W, W), s.pilaster = 0.35f;
            else if (v == 1) s = st(plaster, {{0.55f, 1.25f, 0.8f, 2.55f}, {1.75f, 2.45f, 0.8f, 2.55f}}, W, W, W);
            else s = st(rustic, {{0.95f, 2.05f, 0.7f, 2.6f}}, 0.9f, 0.9f, 0.88f);
            s.arch = true;
            return s;
        case 9:
            s = v == 0 ? st(plaster, {{0.2f, 2.8f, 0.85f, 2.6f}}, W, W, W) : v == 1 ? st(brick, {{0.25f, 2.75f, 0.85f, 2.6f}}, W, W, W) : st(silicate, {{0.2f, 2.8f, 0.85f, 2.6f}}, W, W, W);
            s.panes = 3;
            return s;
        case 10:
            if (v == 0) {
                s = st(stone_slab, {{0.12f, 2.88f, 0.12f, 2.45f}}, G, G, G);
                s.panes = 1, s.no_sill = true;
            } else if (v == 1) s = st(sandwich, {}, D, D, D);
            else {
                s = st(brick, {{0.3f, 2.7f, 0.0f, 2.25f}}, 0.6f, 0.6f, 0.6f);
                s.doors = true, s.no_sill = true;
            }
            return s;
        case 11:
            if (v == 0) s = st(alu_panel, {{0.15f, 2.85f, 0.95f, 2.7f}}, G, G, G), s.panes = 3;
            else if (v == 1) s = st(alu_panel, {{0.3f, 2.7f, 0.25f, 2.8f}}, G, G, G), s.pilaster = 0.4f, s.no_sill = true;
            else s = st(stone_slab, {{0.45f, 2.55f, 0.85f, 2.6f}}, G, G, G);
            return s;
        case 1:
            if (v == 0) return st(brick, {{0.85f, 2.15f, 0.85f, 2.45f}}, 0.92f, 0.92f, 0.9f);
            if (v == 1) {
                s = st(brick, {{0.95f, 2.05f, 0.7f, 2.55f}}, W, W, W);
                s.pilaster = 0.25f;
                return s;
            }
            return st(brick, {{0.55f, 1.3f, 0.9f, 2.3f}, {1.7f, 2.45f, 0.9f, 2.3f}}, 0.85f, 0.85f, 0.82f);
        case 2:
            if (v == 0) return st(panel, {{0.75f, 2.25f, 0.9f, 2.3f}}, 0.9f, 0.9f, 0.88f);
            if (v == 1) {
                s = st(panel, {{0.6f, 2.4f, 1.0f, 2.4f}}, 0.92f, 0.92f, 0.9f);
                s.loggia = true, s.l0 = 0.05f, s.l1 = 0.95f;
                return s;
            }
            return st(panel, {{0.45f, 1.35f, 0.9f, 2.35f}, {1.75f, 2.55f, 0.9f, 2.35f}}, 0.88f, 0.88f, 0.86f);
        case 3:
            return v == 0 ? st(glass, {}, D, D, D) : v == 1 ? st(glass_ribbon, {}, D, D, D) : st(glass, {}, 0.7f, 0.7f, 0.72f);
        case 4:
            return v == 0 ? st(wood, {{1.0f, 2.0f, 1.0f, 2.2f}}, W, W, W) : st(wood, {{0.6f, 1.4f, 0.9f, 2.2f}, {1.8f, 2.4f, 0.9f, 2.2f}}, W, W, W);
        case 6:
            if (v == 0) return st(tile_panel, {{0.6f, 2.4f, 0.9f, 2.35f}}, W, W, W);
            if (v == 1) {
                s = st(tile_panel, {{0.45f, 1.3f, 0.9f, 2.35f}, {1.7f, 2.55f, 0.9f, 2.35f}}, W, W, W);
                s.loggia = true, s.l0 = 0.05f, s.l1 = 0.9f;
                return s;
            }
            return st(tile_panel, {{0.75f, 2.25f, 0.95f, 2.3f}}, 0.88f, 0.88f, 0.86f);
        case 7:
            return v == 0 ? st(silicate, {{0.8f, 2.2f, 0.9f, 2.35f}}, W, W, W) : st(silicate, {{0.55f, 1.3f, 0.9f, 2.3f}, {1.7f, 2.45f, 0.9f, 2.3f}}, W, W, W);
        case 5:
            return v == 0 ? st(metal, {}, D, D, D) : v == 1 ? st(metal, {{0.3f, 2.7f, 2.0f, 2.6f}}, D, D, D) : st(metal, {{0.6f, 2.4f, 1.6f, 2.5f}}, D, D, D);
        default:
            if (v == 0) return st(plaster, {{0.8f, 2.2f, 0.9f, 2.4f}}, W, W, W);
            if (v == 1) {
                s = st(plaster, {{1.0f, 2.0f, 0.75f, 2.55f}}, W, W, W);
                s.pilaster = 0.3f;
                return s;
            }
            return st(plaster, {{0.5f, 1.35f, 0.95f, 2.35f}, {1.65f, 2.5f, 0.95f, 2.35f}}, 0.9f, 0.88f, 0.84f);
    }
}

Px scaled(Px p, float k) {
    for (float &c : p.c) c *= k;
    return p;
}

Px pixel(const Style &s, int px, int py) {
    int bx = px / BAY, by = py / BAY;
    float m = 3.0f / BAY;
    float x = (px % BAY) * m + m * 0.5f, y = (py % BAY) * m + m * 0.5f;
    if (s.pilaster > 0 && (x < s.pilaster / 2 || x > 3 - s.pilaster / 2)) {
        Px p = scaled(s.wall(x, y, bx, by), 0.9f);
        p.depth = 0;
        return p;
    }
    if (s.loggia && y > s.l0 && y < s.l1 && x > 0.3f && x < 2.7f) {
        Px p = scaled(s.wall(x, y, bx, by), 0.62f);
        p.depth = 0.75f;
        return p;
    }
    for (auto &w : s.windows) {
        float f = 0.07f, r = (w.x1 - w.x0) / 2, cx = (w.x0 + w.x1) / 2;
        auto in_arch = [&](float pad) {
            float yc = w.y1 - r;
            return y <= yc || std::pow((x - cx) / (r + pad), 2.0f) + std::pow((y - yc) / (r + pad), 2.0f) <= 1;
        };
        if (x >= w.x0 - f && x <= w.x1 + f && y >= w.y0 - f && y <= w.y1 + f && (!s.arch || in_arch(f))) {
            bool inside = x >= w.x0 && x <= w.x1 && y >= w.y0 && y <= w.y1 && (!s.arch || in_arch(0));
            if (s.doors && inside) {
                bool slat = fract(y / 0.1f) < 0.15f;
                float v = (slat ? 0.62f : 0.78f) - hsh(bx * 5, by * 3 + 1) * 0.1f;
                return mk(v, v, v * 1.02f, 0.5f, 0.4f, 0, slat ? 0.45f : 0.38f);
            }
            float pw = (w.x1 - w.x0) / s.panes;
            bool mull = false;
            for (int k = 1; k < s.panes; k++) mull |= std::abs(x - (w.x0 + pw * k)) < 0.03f;
            bool transom = s.arch && std::abs(y - (w.y1 - r)) < 0.03f;
            // a horizontal transom in tall windows (top light)
            bool top_light = !s.arch && (w.y1 - w.y0) > 1.3f && std::abs(y - (w.y1 - 0.45f)) < 0.025f;
            if (!inside || mull || transom || top_light) return mk(s.frame[0], s.frame[1], s.frame[2], 0.5f, 0, 0, inside ? 0.55f : 0.4f);
            bool curtain = hsh(bx * 31 + 1, by * 17 + 3) > 0.7f && y > w.y1 - (w.y1 - w.y0) * 0.35f;
            float g[3] = {0.13f, 0.17f, 0.22f};
            if (curtain) g[0] = 0.55f, g[1] = 0.52f, g[2] = 0.48f;
            float edge = std::min({x - w.x0, w.x1 - x, y - w.y0, w.y1 - y});
            float ao = 0.55f + 0.45f * std::clamp(edge / 0.25f, 0.0f, 1.0f);
            float sky = curtain ? 1 : 1 + 0.6f * ((y - w.y0) / (w.y1 - w.y0));
            float k = ao * sky;
            return mk(g[0] * k, g[1] * k, g[2] * k, 0.04f, 0, 1, 1);
        }
        if (s.no_sill) continue;
        if (x >= w.x0 - 0.1f && x <= w.x1 + 0.1f && y < w.y0 - f && y > w.y0 - f - 0.06f) {
            Px p = scaled(s.wall(x, y, bx, by), 1.05f);
            p.depth = 0;
            return p;
        }
        if (x >= w.x0 && x <= w.x1 && y < w.y0 - f - 0.06f && y > w.y0 - f - 0.16f) return scaled(s.wall(x, y, bx, by), 0.75f);
        if ((std::abs(x - w.x0) < 0.08f || std::abs(x - w.x1) < 0.08f) && y < w.y0 - 0.16f && y > w.y0 - 1.4f) {
            float k = 0.93f + 0.07f * ((w.y0 - y) / 1.4f) + hsh((uint32_t)(x * 50), (uint32_t)(y * 30)) * 0.02f;
            return scaled(s.wall(x, y, bx, by), k);
        }
    }
    return s.wall(x, y, bx, by);
}

uint8_t u8(float v) { return (uint8_t)std::lround(std::clamp(v, 0.0f, 1.0f) * 255); }

}  // namespace

FacadeTex facade_texture(int si, int v) {
    Style s = style(si, v);
    FacadeTex t;
    t.albedo.resize(TEX * TEX * 4);
    t.orm.resize(TEX * TEX * 4);
    t.emission.resize(TEX * TEX * 4);
    t.normal.resize(TEX * TEX * 4);
    t.depth.resize(TEX * TEX);
    std::vector<float> dep(TEX * TEX);
    for (int row = 0; row < TEX; row++) {
        int py = TEX - 1 - row;
        for (int px = 0; px < TEX; px++) {
            Px p = pixel(s, px, py);
            size_t i = (size_t)row * TEX + px;
            dep[i] = std::clamp(p.depth, 0.0f, 1.0f);
            t.depth[i] = u8(p.depth);
            for (int k = 0; k < 3; k++) t.albedo[i * 4 + k] = u8(p.c[k]);
            t.albedo[i * 4 + 3] = 255;
            // occlusion: darker in deep reveals
            t.orm[i * 4] = u8(1 - p.depth * 0.35f);
            t.orm[i * 4 + 1] = u8(p.rough);
            t.orm[i * 4 + 2] = u8(p.metal);
            t.orm[i * 4 + 3] = 255;
            int bx = px / BAY, by = py / BAY;
            bool on = p.lit > 0 && hsh(bx + 101, by + 211) < 0.35f;
            bool warm = hsh(bx + 7, by + 5) < 0.7f;
            float e[3] = {0, 0, 0};
            if (on) {
                if (warm) e[0] = 1, e[1] = 0.78f, e[2] = 0.45f;
                else e[0] = 0.85f, e[1] = 0.9f, e[2] = 1;
            }
            for (int k = 0; k < 3; k++) t.emission[i * 4 + k] = u8(e[k]);
            t.emission[i * 4 + 3] = 255;
        }
    }
    // normals from the relief: depth in metres ~ 0.25 m for 1.0, one texel = 3/128 m
    const float scale = 0.25f / (3.0f / BAY);
    for (int row = 0; row < TEX; row++)
        for (int px = 0; px < TEX; px++) {
            auto D = [&](int x, int y) { return dep[(size_t)((y + TEX) % TEX) * TEX + (x + TEX) % TEX]; };
            // height = -depth; image rows go down while v (tangent-space +y) goes up
            float dx = -(D(px + 1, row) - D(px - 1, row)) * 0.5f * scale;
            float dy = (D(px, row + 1) - D(px, row - 1)) * 0.5f * scale;
            float nx = -dx, ny = -dy, nz = 1;
            float l = std::sqrt(nx * nx + ny * ny + nz * nz);
            size_t i = ((size_t)row * TEX + px) * 4;
            t.normal[i] = u8(nx / l * 0.5f + 0.5f);
            t.normal[i + 1] = u8(ny / l * 0.5f + 0.5f);
            t.normal[i + 2] = u8(nz / l * 0.5f + 0.5f);
            t.normal[i + 3] = 255;
        }
    return t;
}

}  // namespace ec
