// 2D polygon helpers in local metres.
#pragma once
#include <array>
#include <vector>
#include "util.hpp"

namespace ec {

using Ring = std::vector<V2>;

double signed_area(const Ring &r);
// counter-clockwise outer ring
void make_ccw(Ring &r);
void make_cw(Ring &r);
// Drop vertices closer than `min_d` and nearly-straight ones (turn < ~`deg` degrees).
Ring simplify(const Ring &r, double min_d = 0.3, double deg = 6);
// Triangles of a polygon with holes (outer first); indices into the concatenation of all rings.
std::vector<int> triangulate(const std::vector<Ring> &rings);
bool point_in_ring(V2 p, const Ring &r);
bool point_in_poly(V2 p, const std::vector<Ring> &rings);
double seg_dist(V2 p, V2 a, V2 b);
double ring_dist(V2 p, const Ring &r);
// Outward offset of a CCW ring by d (mitred, limited).
Ring offset_ring(const Ring &r, double d);
// Depth of a building corpus: short side of the rectangle with the same area and perimeter.
double corpus_depth(const Ring &r);
double perimeter(const Ring &r);
V2 centroid(const Ring &r);
// Polyline helpers
std::vector<double> chainage(const std::vector<V2> &p);
// mitred left normals and scale per point
void normals(const std::vector<V2> &p, std::vector<V2> &n, std::vector<double> &scale);
// Points at most `step` metres apart
std::vector<V2> densify(const std::vector<V2> &p, double step);
// Split a long triangle mesh: recursively halve triangles until edges <= max
void split_tri(V3 a, V3 b, V3 c, double max, int depth, std::vector<std::array<V3, 3>> &out);
// Oriented bounding frame of a footprint: centre, unit axis along the longest edge, length, width
void footprint_frame(const Ring &r, V2 &c, V2 &axis, double &len, double &wid);

}  // namespace ec
