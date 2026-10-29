#include "glue.h"
#include <cstring>
#include <godot_cpp/variant/packed_color_array.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/packed_vector2_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>

namespace godot {

Array mesh_arrays(const ec::MeshBuf &m) {
    Array a;
    a.resize(Mesh::ARRAY_MAX);
    size_t n = m.verts();
    PackedVector3Array pos, nrm;
    PackedVector2Array uv;
    PackedColorArray col;
    PackedFloat32Array tan;
    PackedInt32Array idx;
    pos.resize((int64_t)n);
    nrm.resize((int64_t)n);
    uv.resize((int64_t)n);
    col.resize((int64_t)n);
    tan.resize((int64_t)n * 4);
    Vector3 *P = pos.ptrw(), *N = nrm.ptrw();
    Vector2 *U = uv.ptrw();
    Color *C = col.ptrw();
    float *T = tan.ptrw();
    for (size_t i = 0; i < n; i++) {
        P[i] = Vector3(m.pos[i * 3], m.pos[i * 3 + 2], -m.pos[i * 3 + 1]);
        N[i] = Vector3(m.nrm[i * 3], m.nrm[i * 3 + 2], -m.nrm[i * 3 + 1]);
        U[i] = Vector2(m.uv[i * 2], m.uv[i * 2 + 1]);
        C[i] = Color(m.col[i * 4], m.col[i * 4 + 1], m.col[i * 4 + 2], m.col[i * 4 + 3]);
    }
    // tangents from uv gradients per triangle, accumulated
    std::vector<Vector3> acc(n, Vector3());
    idx.resize((int64_t)m.idx.size());
    int32_t *I = idx.ptrw();
    for (size_t k = 0; k + 2 < m.idx.size(); k += 3) {
        int a0 = m.idx[k], a1 = m.idx[k + 1], a2 = m.idx[k + 2];
        // Godot's front faces are clockwise: flip
        I[k] = a0, I[k + 1] = a2, I[k + 2] = a1;
        Vector3 e1 = P[a1] - P[a0], e2 = P[a2] - P[a0];
        Vector2 d1 = U[a1] - U[a0], d2 = U[a2] - U[a0];
        float r = d1.x * d2.y - d2.x * d1.y;
        if (std::abs(r) < 1e-12f) continue;
        Vector3 t = (e1 * d2.y - e2 * d1.y) / r;
        acc[a0] += t, acc[a1] += t, acc[a2] += t;
    }
    for (size_t i = 0; i < n; i++) {
        Vector3 t = acc[i] - N[i] * N[i].dot(acc[i]);
        if (t.length_squared() < 1e-12f) {
            t = N[i].cross(Vector3(0, 0, 1));
            if (t.length_squared() < 1e-6f) t = N[i].cross(Vector3(1, 0, 0));
        }
        t = t.normalized();
        T[i * 4] = t.x, T[i * 4 + 1] = t.y, T[i * 4 + 2] = t.z, T[i * 4 + 3] = 1;
    }
    a[Mesh::ARRAY_VERTEX] = pos;
    a[Mesh::ARRAY_NORMAL] = nrm;
    a[Mesh::ARRAY_TANGENT] = tan;
    a[Mesh::ARRAY_TEX_UV] = uv;
    a[Mesh::ARRAY_COLOR] = col;
    a[Mesh::ARRAY_INDEX] = idx;
    return a;
}

PackedByteArray bytes(const std::vector<uint8_t> &v) {
    PackedByteArray b;
    b.resize((int64_t)v.size());
    if (!v.empty()) memcpy(b.ptrw(), v.data(), v.size());
    return b;
}


}  // namespace godot
