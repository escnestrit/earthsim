// Procedural tileable material textures (ground covers, road, roofs, bark, foliage). Each is built from a
// height field and colour layers made of periodic noise, so they tile without seams.
#include <algorithm>
#include <cmath>
#include "textures.hpp"

namespace ec {

namespace {

constexpr int S = GROUND_TEX;

uint32_t ih(uint32_t a, uint32_t b, uint32_t c = 0) {
    uint32_t h = a * 0x9E3779B1u ^ b * 0x85EBCA77u ^ c * 0xC2B2AE3Du;
    h ^= h >> 15;
    h *= 0x2C1B3C6Du;
    h ^= h >> 12;
    h *= 0x297A2D39u;
    h ^= h >> 15;
    return h;
}
float h01(uint32_t a, uint32_t b, uint32_t c = 0) { return (ih(a, b, c) & 0xFFFFFF) / float(0xFFFFFF); }

// periodic value noise: `p` cells across the texture
float vnoise(float x, float y, int p, uint32_t seed) {
    float fx = x * p, fy = y * p;
    int ix = (int)std::floor(fx), iy = (int)std::floor(fy);
    float tx = fx - ix, ty = fy - iy;
    tx = tx * tx * (3 - 2 * tx), ty = ty * ty * (3 - 2 * ty);
    auto g = [&](int i, int j) { return h01((uint32_t)(((i % p) + p) % p), (uint32_t)(((j % p) + p) % p), seed); };
    float a = g(ix, iy), b = g(ix + 1, iy), c = g(ix, iy + 1), d = g(ix + 1, iy + 1);
    return (a + (b - a) * tx) + ((c + (d - c) * tx) - (a + (b - a) * tx)) * ty;
}
float fbm(float x, float y, int p, int oct, uint32_t seed, float gain = 0.5f) {
    float s = 0, a = 1, n = 0;
    for (int o = 0; o < oct; o++) {
        s += vnoise(x, y, p << o, seed + o * 17) * a;
        n += a;
        a *= gain;
    }
    return s / n;
}
// periodic cellular noise: distance to nearest feature point (cells across), and id of that cell
float cells(float x, float y, int p, uint32_t seed, uint32_t *id = nullptr, float *second = nullptr) {
    float fx = x * p, fy = y * p;
    int ix = (int)std::floor(fx), iy = (int)std::floor(fy);
    float best = 1e9, best2 = 1e9;
    for (int j = -1; j <= 1; j++)
        for (int i = -1; i <= 1; i++) {
            int cx = ix + i, cy = iy + j;
            uint32_t wx = (uint32_t)(((cx % p) + p) % p), wy = (uint32_t)(((cy % p) + p) % p);
            float px = cx + h01(wx, wy, seed), py = cy + h01(wx, wy, seed + 1);
            float d = std::hypot(px - fx, py - fy);
            if (d < best) {
                best2 = best;
                best = d;
                if (id) *id = ih(wx, wy, seed + 2);
            } else if (d < best2) best2 = d;
        }
    if (second) *second = best2;
    return best;
}

float clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }
uint8_t u8(float v) { return (uint8_t)std::lround(clamp01(v) * 255); }
float mixf(float a, float b, float t) { return a + (b - a) * t; }
float smooth(float e0, float e1, float x) {
    float t = clamp01((x - e0) / (e1 - e0));
    return t * t * (3 - 2 * t);
}

struct Field {
    std::vector<float> h, r, g, b, a, rough, metal, ao;
    Field() : h(S * S, 0), r(S * S), g(S * S), b(S * S), a(S * S, 1), rough(S * S, 0.9f), metal(S * S, 0), ao(S * S, 1) {}
};

MatTex finish(Field &f, float strength) {
    MatTex t;
    t.albedo.resize(S * S * 4);
    t.normal.resize(S * S * 4);
    t.orm.resize(S * S * 4);
    for (int y = 0; y < S; y++)
        for (int x = 0; x < S; x++) {
            size_t i = (size_t)y * S + x;
            auto H = [&](int xx, int yy) { return f.h[(size_t)((yy + S) % S) * S + (xx + S) % S]; };
            float dx = (H(x + 1, y) - H(x - 1, y)) * 0.5f * strength;
            float dy = (H(x, y + 1) - H(x, y - 1)) * 0.5f * strength;
            float nx = -dx, ny = dy, nz = 1;
            float l = std::sqrt(nx * nx + ny * ny + nz * nz);
            t.normal[i * 4] = u8(nx / l * 0.5f + 0.5f);
            t.normal[i * 4 + 1] = u8(ny / l * 0.5f + 0.5f);
            t.normal[i * 4 + 2] = u8(nz / l * 0.5f + 0.5f);
            t.normal[i * 4 + 3] = 255;
            t.albedo[i * 4] = u8(f.r[i]);
            t.albedo[i * 4 + 1] = u8(f.g[i]);
            t.albedo[i * 4 + 2] = u8(f.b[i]);
            t.albedo[i * 4 + 3] = u8(f.a[i]);
            t.orm[i * 4] = u8(f.ao[i]);
            t.orm[i * 4 + 1] = u8(f.rough[i]);
            t.orm[i * 4 + 2] = u8(f.metal[i]);
            t.orm[i * 4 + 3] = 255;
        }
    return t;
}

void set(Field &f, size_t i, float r, float g, float b) { f.r[i] = r, f.g[i] = g, f.b[i] = b; }

// leaf / needle cards: clusters drawn on a transparent background
MatTex foliage(bool conifer) {
    Field f;
    for (auto &v : f.a) v = 0;
    for (size_t i = 0; i < f.r.size(); i++) set(f, i, 0.12f, 0.2f, 0.07f);
    int n = conifer ? 2600 : 900;
    for (int k = 0; k < n; k++) {
        float cx = h01(k, 1, 91) * S, cy = h01(k, 2, 91) * S;
        // keep away from the card borders so mips don't bleed
        float ddx = cx / S - 0.5f, ddy = cy / S - 0.5f;
        if (std::hypot(ddx, ddy) > 0.47f) continue;
        float ang = h01(k, 3, 91) * 6.2832f;
        float len = conifer ? 10 + h01(k, 4, 91) * 8 : 12 + h01(k, 4, 91) * 10, wid = conifer ? 1.3f : len * 0.42f;
        float shade = 0.65f + h01(k, 5, 91) * 0.45f;
        float hue = h01(k, 6, 91);
        float cr = conifer ? 0.09f + hue * 0.05f : 0.16f + hue * 0.14f, cg = conifer ? 0.22f + hue * 0.06f : 0.3f + hue * 0.12f, cb = conifer ? 0.08f : 0.06f + hue * 0.04f;
        float ca = std::cos(ang), sa = std::sin(ang);
        int r = (int)len + 2;
        for (int y = (int)cy - r; y <= (int)cy + r; y++)
            for (int x = (int)cx - r; x <= (int)cx + r; x++) {
                if (x < 0 || y < 0 || x >= S || y >= S) continue;
                float u = (x - cx) * ca + (y - cy) * sa, v = -(x - cx) * sa + (y - cy) * ca;
                float e = (u * u) / (len * len) + (v * v) / (wid * wid);
                if (e > 1) continue;
                size_t i = (size_t)y * S + x;
                float mid = conifer ? 1 : (std::abs(v) < 0.6f ? 0.8f : 1.0f);
                float side = 0.85f + 0.15f * (v > 0);
                set(f, i, cr * shade * mid * side, cg * shade * mid * side, cb * shade * mid);
                f.a[i] = 1;
                f.h[i] = 1 - e;
                f.ao[i] = 0.7f + 0.3f * (1 - std::hypot(ddx, ddy) * 2);
            }
    }
    // twig lines under the leaves
    for (size_t i = 0; i < f.r.size(); i++) f.rough[i] = 0.6f;
    return finish(f, 1.5f);
}

// grass blades on a transparent background, bases along the bottom edge
MatTex blades() {
    Field f;
    for (auto &v : f.a) v = 0;
    for (size_t i = 0; i < f.r.size(); i++) set(f, i, 0.2f, 0.35f, 0.1f);
    for (int k = 0; k < 140; k++) {
        float x0 = h01(k, 1, 77) * S, lean = (h01(k, 2, 77) - 0.5f) * 0.5f, hgt = (0.45f + 0.55f * h01(k, 3, 77)) * S * 0.97f;
        float w0 = 3 + h01(k, 4, 77) * 4, hue = h01(k, 5, 77);
        for (int y = 0; y < (int)hgt; y++) {
            float t = y / hgt;
            float cx = x0 + lean * y + lean * y * t * 0.6f, w = w0 * (1 - t);
            for (int x = (int)(cx - w - 1); x <= (int)(cx + w + 1); x++) {
                int xx = ((x % S) + S) % S;
                if (std::abs(x - cx) > w) continue;
                int row = S - 1 - y;
                size_t i = (size_t)row * S + xx;
                float shade = 0.55f + 0.45f * t;
                set(f, i, (0.16f + 0.12f * hue) * shade, (0.3f + 0.12f * hue) * shade, (0.08f + 0.04f * hue) * shade);
                f.a[i] = 1;
                f.h[i] = 1 - std::abs(x - cx) / std::max(0.5f, w);
                f.ao[i] = 0.5f + 0.5f * t;
            }
        }
    }
    for (auto &r : f.rough) r = 0.7f;
    return finish(f, 1.0f);
}

}  // namespace

MatTex material_texture(int kind) {
    if (kind == T_GRASS_BLADES) return blades();
    if (kind == T_LEAVES) return foliage(false);
    if (kind == T_CONIFER) return foliage(true);
    Field f;
    float strength = 2;
    for (int y = 0; y < S; y++)
        for (int x = 0; x < S; x++) {
            size_t i = (size_t)y * S + x;
            float u = (x + 0.5f) / S, v = (y + 0.5f) / S;
            switch (kind) {
                case T_GRASS: {
                    // uniform at the texture's scale (the shader adds large patches): blades and clumps
                    float mid = fbm(u, v, 16, 3, 12), blades = vnoise(u * 1.0f, v * 6.0f, 160, 13), clump = cells(u, v, 48, 14);
                    float shade = 0.78f + 0.25f * mid + 0.22f * (blades - 0.5f) - 0.12f * clump;
                    float yellow = h01(x / 3, y / 9, 15) > 0.93f ? 0.35f : 0.0f;
                    set(f, i, mixf(0.2f, 0.42f, yellow) * shade, mixf(0.36f, 0.4f, yellow) * shade, mixf(0.11f, 0.15f, yellow) * shade);
                    f.h[i] = blades * 0.6f + (1 - clump) * 0.4f;
                    f.rough[i] = 0.92f;
                    f.ao[i] = 0.75f + 0.25f * blades;
                    strength = 3;
                    break;
                }
                case T_FOREST_FLOOR: {
                    uint32_t id;
                    float c = cells(u, v, 40, 21, &id);
                    float leaf = c < 0.42f ? 1 : 0;
                    float n = fbm(u, v, 8, 4, 22);
                    float hue = (id & 255) / 255.0f;
                    if (leaf) set(f, i, 0.32f + hue * 0.22f, 0.2f + hue * 0.1f, 0.09f);
                    else set(f, i, 0.17f * (0.7f + n * 0.6f), 0.13f * (0.7f + n * 0.6f), 0.08f);
                    f.h[i] = leaf * (0.42f - c) * 2 + n * 0.3f;
                    f.rough[i] = 0.95f;
                    strength = 4;
                    break;
                }
                case T_SAND: {
                    float rip = std::sin((u * 24 + fbm(u, v, 4, 2, 31) * 3) * 6.2832f) * 0.5f + 0.5f;
                    float n = vnoise(u, v, 256, 32);
                    float k = 0.85f + 0.1f * rip + 0.1f * n;
                    set(f, i, 0.76f * k, 0.68f * k, 0.52f * k);
                    f.h[i] = rip * 0.5f + n * 0.3f;
                    f.rough[i] = 0.85f;
                    break;
                }
                case T_ASPHALT: {
                    float n = fbm(u, v, 8, 3, 41);
                    float sp = h01(x, y, 42);
                    float stone = sp > 0.93f ? 0.12f : sp < 0.06f ? -0.05f : 0;
                    uint32_t id;
                    float c2 = 0, c = cells(u, v, 6, 43, &id, &c2);
                    float crack = (c2 - c) < 0.012f && fbm(u, v, 3, 2, 44) > 0.7f ? 1.0f : 0.0f;
                    float g = 0.15f + 0.05f * n + stone - crack * 0.06f;
                    set(f, i, g, g, g * 1.02f);
                    f.h[i] = (sp > 0.9f ? 0.6f : 0.3f) + n * 0.3f - crack * 0.6f;
                    f.rough[i] = 0.82f - stone * 0.5f;
                    f.ao[i] = 1 - crack * 0.5f;
                    strength = 3;
                    break;
                }
                case T_PAVERS: {
                    // 3 m across: rectangular concrete pavers 0.2 x 0.1 m in a running bond
                    float mx = u * 3, my = v * 3;
                    int row = (int)std::floor(my / 0.1f);
                    float off = (row % 2) * 0.1f;
                    int col = (int)std::floor((mx + off) / 0.2f);
                    float fx = std::fmod(mx + off, 0.2f) / 0.2f, fy = std::fmod(my, 0.1f) / 0.1f;
                    float edge = std::min({fx * 0.2f, (1 - fx) * 0.2f, fy * 0.1f, (1 - fy) * 0.1f});
                    float bevel = smooth(0.0f, 0.012f, edge);
                    float hue = h01(col & 15, row & 31, 51);
                    float n = vnoise(u, v, 128, 52);
                    float k = 0.46f + 0.1f * hue + 0.05f * n;
                    float rr = hue > 0.85f ? 0.08f : 0;
                    set(f, i, (k + rr) * bevel + 0.25f * (1 - bevel), k * bevel + 0.24f * (1 - bevel), (k - rr * 0.5f) * bevel + 0.22f * (1 - bevel));
                    f.h[i] = bevel * 0.8f + n * 0.1f;
                    f.ao[i] = 0.6f + 0.4f * bevel;
                    f.rough[i] = 0.85f;
                    strength = 6;
                    break;
                }
                case T_FARM: {
                    float row = std::sin(v * 40 * 6.2832f) * 0.5f + 0.5f;
                    float n = fbm(u, v, 8, 4, 61);
                    float green = smooth(0.6f, 0.8f, fbm(u, v, 4, 2, 62)) * row;
                    set(f, i, mixf(0.3f, 0.25f, green) * (0.8f + n * 0.4f), mixf(0.22f, 0.32f, green) * (0.8f + n * 0.4f), mixf(0.14f, 0.1f, green));
                    f.h[i] = row * 0.6f + n * 0.4f;
                    f.rough[i] = 0.95f;
                    strength = 3;
                    break;
                }
                case T_DIRT:
                case T_GRAVEL: {
                    bool gravel = kind == T_GRAVEL;
                    uint32_t id;
                    float c = cells(u, v, gravel ? 90 : 50, 71, &id);
                    float stone = smooth(gravel ? 0.5f : 0.35f, gravel ? 0.25f : 0.15f, c);
                    float n = fbm(u, v, 8, 4, 72);
                    float hue = (id & 255) / 255.0f;
                    float sr = 0.45f + hue * 0.2f;
                    float dr = gravel ? 0.36f : 0.3f, dg = gravel ? 0.35f : 0.24f, db = gravel ? 0.33f : 0.17f;
                    set(f, i, mixf(dr * (0.8f + n * 0.4f), sr, stone), mixf(dg * (0.8f + n * 0.4f), sr * 0.97f, stone), mixf(db * (0.8f + n * 0.4f), sr * 0.92f, stone));
                    f.h[i] = stone * 0.8f + n * 0.3f;
                    f.ao[i] = 0.75f + 0.25f * stone;
                    f.rough[i] = 0.9f;
                    strength = 4;
                    break;
                }
                case T_PITCH: {
                    float stripe = std::fmod(u * 6, 1.0f) < 0.5f ? 1.0f : 0.88f;
                    float fine = vnoise(u, v * 3, 128, 81);
                    float k = stripe * (0.85f + 0.2f * fine);
                    set(f, i, 0.17f * k, 0.38f * k, 0.12f * k);
                    f.h[i] = fine;
                    f.rough[i] = 0.9f;
                    break;
                }
                case T_CONCRETE:
                case T_CURB: {
                    float n = fbm(u, v, 6, 5, 91), pore = h01(x, y, 92) > 0.985f ? 1.0f : 0.0f;
                    float stain = smooth(0.55f, 0.8f, fbm(u, v, 3, 3, 93));
                    float g = (kind == T_CURB ? 0.72f : 0.66f) + 0.08f * n - 0.05f * stain - pore * 0.2f;
                    set(f, i, g, g * 0.99f, g * 0.97f);
                    f.h[i] = n * 0.4f - pore * 0.5f;
                    f.rough[i] = 0.88f;
                    f.ao[i] = 1 - pore * 0.4f;
                    break;
                }
                case T_ROOF_TILE: {
                    // 3 m: clay tiles 0.3 m wide, rows 0.33 m, curved profile
                    float mx = u * 3, my = v * 3;
                    int row = (int)std::floor(my / 0.333f);
                    float off = (row % 2) * 0.15f;
                    float fx = std::fmod(mx + off, 0.3f) / 0.3f, fy = std::fmod(my, 0.333f) / 0.333f;
                    float curve = std::sin(fx * 3.14159f);
                    float overlap = smooth(0.0f, 0.15f, fy);
                    float hue = h01((int)std::floor((mx + off) / 0.3f) & 15, row & 15, 101);
                    float k = (0.7f + 0.3f * curve) * (0.55f + 0.45f * overlap) * (0.85f + hue * 0.25f);
                    set(f, i, 0.62f * k, 0.3f * k, 0.2f * k);
                    f.h[i] = curve * 0.5f + fy * 0.5f;
                    f.ao[i] = 0.5f + 0.5f * overlap;
                    f.rough[i] = 0.7f;
                    strength = 8;
                    break;
                }
                case T_ROOF_METAL: {
                    float mx = std::fmod(u * 3, 0.5f) / 0.5f;
                    float seam = smooth(0.03f, 0.0f, std::min(mx, 1 - mx));
                    float n = fbm(u, v, 4, 3, 111);
                    float k = 0.8f + 0.15f * n;
                    set(f, i, k, k, k);
                    f.h[i] = seam;
                    f.rough[i] = 0.45f + 0.2f * n;
                    f.metal[i] = 0.7f;
                    strength = 6;
                    break;
                }
                case T_ROOF_FLAT: {
                    float seam = std::fmod(v * 4, 1.0f) < 0.012f ? 1.0f : 0.0f;
                    float n = fbm(u, v, 8, 4, 121), sp = h01(x, y, 122);
                    float g = 0.24f + 0.08f * n + (sp > 0.9f ? 0.08f : 0) + seam * 0.05f;
                    set(f, i, g, g * 0.98f, g * 0.96f);
                    f.h[i] = n * 0.3f + seam * 0.5f + (sp > 0.9f ? 0.3f : 0);
                    f.rough[i] = 0.9f;
                    break;
                }
                case T_BARK: {
                    float fiss = fbm(u * 6, v * 0.8f, 4, 4, 131);
                    float ridge = smooth(0.35f, 0.6f, fiss);
                    float k = 0.45f + 0.55f * ridge;
                    set(f, i, 0.33f * k, 0.27f * k, 0.22f * k);
                    f.h[i] = ridge;
                    f.ao[i] = 0.5f + 0.5f * ridge;
                    f.rough[i] = 0.95f;
                    strength = 5;
                    break;
                }
                default: {  // rock
                    float n = fbm(u, v, 4, 6, 141);
                    float k = 0.4f + 0.3f * n;
                    set(f, i, k, k * 0.97f, k * 0.93f);
                    f.h[i] = n;
                    f.rough[i] = 0.8f;
                    strength = 6;
                }
            }
        }
    return finish(f, strength);
}

}  // namespace ec
