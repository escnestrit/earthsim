#include "mesh.hpp"

namespace es {

void Mesh::smooth_normals() {
    size_t n = verts();
    std::vector<double> acc(n * 3, 0.0);
    for (size_t i = 0; i + 2 < idx.size(); i += 3) {
        int a = idx[i], b = idx[i + 1], c = idx[i + 2];
        V3 pa(pos[a * 3], pos[a * 3 + 1], pos[a * 3 + 2]), pb(pos[b * 3], pos[b * 3 + 1], pos[b * 3 + 2]), pc(pos[c * 3], pos[c * 3 + 1], pos[c * 3 + 2]);
        V3 f = cross(pb - pa, pc - pa);
        for (int v : {a, b, c}) acc[v * 3] += f.x, acc[v * 3 + 1] += f.y, acc[v * 3 + 2] += f.z;
    }
    for (size_t v = 0; v < n; v++) {
        V3 q = norm(V3(acc[v * 3], acc[v * 3 + 1], acc[v * 3 + 2]));
        nrm[v * 3] = float(q.x), nrm[v * 3 + 1] = float(q.y), nrm[v * 3 + 2] = float(q.z);
    }
}

void add_quad(Mesh &m, V3 a, V3 b, V3 c, V3 d, V2 ua, V2 ub, V2 uc, V2 ud, V2 uv2, float r, float g, float b_, float al) {
    V3 n = norm(cross(b - a, c - a));
    if (dot(n, n) < 0.5) n = norm(cross(c - a, d - a));
    int i0 = m.vert(a, n, ua, uv2, r, g, b_, al), i1 = m.vert(b, n, ub, uv2, r, g, b_, al);
    int i2 = m.vert(c, n, uc, uv2, r, g, b_, al), i3 = m.vert(d, n, ud, uv2, r, g, b_, al);
    m.quad(i0, i1, i2, i3);
}

}  // namespace es
