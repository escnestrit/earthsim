// Painted markings lying on the carriageway: zebra stripes (UV2.x = 0) or solid (UV2.x = 1).
shader_type spatial;
render_mode cull_back, depth_prepass_alpha;
#include "res://shaders/common.gdshaderinc"

varying vec3 wpos;
varying vec2 info;

void vertex() {
	wpos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	info = UV2;
}

void fragment() {
	if (info.x < 0.5) {
		// stripes 0.5 m wide every 1 m along the crossing, with a margin at both sides
		if (fract(UV.y / 1.0) > 0.5 || UV.x < 0.15 || UV.x > 3.05) discard;
	}
	float wear = smoothstep(0.2, 0.55, vnoise(wpos.xz * 2.5));
	vec3 col = vec3(0.86, 0.86, 0.82) * (0.85 + 0.15 * wear);
	float rough = 0.55;
	apply_wet(col, rough, 0.5);
	ALBEDO = col;
	ROUGHNESS = rough;
}
