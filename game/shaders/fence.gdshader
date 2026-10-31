// Fences, walls, hedges, guard rails, retaining walls, underpass parapets and stairwells.
// UV = (metres along, metres above the ground line); UV2.x = type, UV2.y = height.
// 0 welded mesh, 1 masonry wall, 2 hedge, 3 guard rail, 4 retaining wall, 5 wooden picket,
// 6 cast-iron railings, 7 concrete panel fence, 8 stairwell (seen from above), 9 granite parapet
shader_type spatial;
render_mode cull_disabled;
#include "res://shaders/common.gdshaderinc"

varying vec2 info;
varying vec3 wpos;

void vertex() {
	info = UV2;
	wpos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
	if (far_hidden(wpos)) discard;
	int type = int(info.x + 0.5);
	float h = info.y;
	float x = UV.x, y = UV.y;
	vec3 c = vec3(0.5);
	float r = 0.8, m = 0.0;
	float n = vnoise(vec2(x, y) * 2.0 + wpos.xz * 0.1);
	if (type == 0) {
		// welded mesh panels between posts
		vec2 g = abs(fract(vec2(x / 0.2, y / 0.05)) - 0.5);
		float post = step(abs(fract(x / 2.5) - 0.5) * 2.5, 0.04);
		float wire = max(step(0.44, g.x), step(0.4, g.y));
		float rail = step(h - 0.06, y) + step(y, 0.04);
		float solid = max(max(post, rail), wire);
		if (solid < 0.5 || y < -0.1) discard;
		c = vec3(0.22, 0.32, 0.26) * (0.85 + 0.3 * n);
		r = 0.5; m = 0.5;
	} else if (type == 5) {
		// wooden picket
		float pk = step(abs(fract(x / 0.14) - 0.5), 0.3);
		float rails = step(abs(y - h * 0.25), 0.05) + step(abs(y - h * 0.75), 0.05);
		if (pk + rails < 0.5) discard;
		c = vec3(0.5, 0.42, 0.3) * (0.75 + 0.4 * n);
	} else if (type == 6) {
		// cast-iron railings with a top band and finials
		float bar = step(abs(fract(x / 0.16) - 0.5), 0.12);
		float band = step(abs(y - (h - 0.25)), 0.06) + step(abs(y - 0.15), 0.05);
		float post = step(abs(fract(x / 3.0) - 0.5) * 3.0, 0.1);
		if (bar + band + post < 0.5 || y > h - 0.05 + 0.08 * step(0.5, bar)) discard;
		c = vec3(0.06, 0.06, 0.07);
		r = 0.4; m = 0.7;
	} else if (type == 7) {
		// concrete panel fence (ПО-2 slabs with a rhombus relief)
		vec2 p = vec2(fract(x / 0.5) - 0.5, fract(y / 0.5) - 0.5);
		float rh = step(abs(p.x) + abs(p.y), 0.35);
		float joint = 1.0 - smoothstep(0.02, 0.05, abs(fract(x / 4.0) - 0.5) * 4.0 - 1.95);
		c = vec3(0.62, 0.61, 0.58) * (0.85 + 0.25 * n) * (1.0 - 0.12 * rh) * (1.0 - 0.3 * joint);
	} else if (type == 1) {
		// masonry: brick or plastered block wall
		vec2 b = vec2(x / 0.25, y / 0.075);
		b.x += floor(b.y) * 0.5;
		vec2 f = abs(fract(b) - 0.5);
		float mortar = smoothstep(0.38, 0.47, max(f.x * 0.9, f.y));
		c = mix(vec3(0.55, 0.3, 0.22) * (0.85 + 0.3 * h21(floor(b))), vec3(0.6, 0.58, 0.54), mortar * 0.7);
		if (y > h - 0.1) c = vec3(0.5, 0.5, 0.48);
	} else if (type == 2) {
		// hedge
		float leaves = fbm(vec2(x, y) * 6.0 + wpos.xz);
		c = mix(vec3(0.08, 0.2, 0.06), vec3(0.22, 0.38, 0.12), leaves);
		r = 0.95;
	} else if (type == 3) {
		// steel guard rail: W-beam on posts
		float beam = step(abs(y - 0.6), 0.17);
		float post = step(abs(fract(x / 2.0) - 0.5) * 2.0, 0.07) * step(y, 0.6);
		if (beam + post < 0.5) discard;
		c = vec3(0.6, 0.62, 0.63) * (0.9 + 0.15 * sin(y * 40.0));
		r = 0.35; m = 0.8;
	} else if (type == 4) {
		c = vec3(0.56, 0.55, 0.52) * (0.8 + 0.3 * n);
		float joint = 1.0 - smoothstep(0.02, 0.05, abs(fract(x / 6.0) - 0.5) * 6.0 - 2.95);
		c *= 1.0 - 0.3 * joint;
	} else if (type == 8) {
		// stairwell from above: steps fading into the dark
		float t = clamp(1.0 - y / 7.0, 0.0, 1.0);
		float step_line = step(0.85, fract(y / 0.3));
		c = mix(vec3(0.45, 0.44, 0.42), vec3(0.02), t) * (1.0 - 0.3 * step_line);
		if (x < 0.25 || x > info.y * 0.0 + 3.15) c *= 0.6;
		r = 0.9;
	} else if (type == 9) {
		c = vec3(0.42, 0.38, 0.36) * (0.85 + 0.3 * h21(floor(vec2(x * 8.0, y * 8.0))));
		r = 0.45;
	}
	apply_wet(c, r, 0.4);
	ALBEDO = c;
	ROUGHNESS = r;
	METALLIC = m;
}
