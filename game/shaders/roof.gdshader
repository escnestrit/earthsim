// Roofs: vertex colour with some weathering. Pitched roofs (steep normals) get tile rows.
shader_type spatial;
render_mode cull_back;
#include "res://shaders/common.gdshaderinc"

varying vec3 wpos;
varying vec3 wnrm;

void vertex() {
	wpos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	wnrm = (MODEL_MATRIX * vec4(NORMAL, 0.0)).xyz;
}

void fragment() {
	if (far_hidden(wpos)) discard;
	vec3 col = COLOR.rgb * (0.85 + 0.25 * fbm(wpos.xz * 0.7));
	float rough = 0.85;
	if (wnrm.y < 0.95) {
		float rows = abs(fract(wpos.y / 0.35) - 0.5);
		col *= 0.85 + 0.15 * smoothstep(0.3, 0.5, rows);
		rough = 0.7;
	} else {
		col *= 0.9 + 0.2 * vnoise(wpos.xz * 4.0);
	}
	apply_wet(col, rough, 0.6);
	ALBEDO = col;
	ROUGHNESS = rough;
}
