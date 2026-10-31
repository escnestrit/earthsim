// Ground: COLOR weights r grass, g forest floor, b bare soil/sand, a urban dirt/gravel.
shader_type spatial;
render_mode cull_back;
#include "res://shaders/common.gdshaderinc"

varying vec3 wpos;

void vertex() {
	wpos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
	if (far_hidden(wpos)) discard;
	vec2 w = wpos.xz;
	float n1 = fbm(w * 0.08), n2 = fbm(w * 0.6), n3 = vnoise(w * 5.0);
	vec4 m = COLOR;
	float s = max(0.001, m.r + m.g + m.b + m.a);
	m /= s;
	vec3 grass = mix(vec3(0.20, 0.30, 0.10), vec3(0.32, 0.38, 0.14), n1) * (0.8 + 0.35 * n2) * (0.9 + 0.2 * n3);
	vec3 forest = mix(vec3(0.14, 0.17, 0.08), vec3(0.22, 0.18, 0.11), n2) * (0.8 + 0.3 * n3);
	vec3 soil = mix(vec3(0.36, 0.30, 0.22), vec3(0.50, 0.44, 0.33), n1) * (0.85 + 0.3 * n3);
	vec3 urban = mix(vec3(0.30, 0.29, 0.26), vec3(0.38, 0.36, 0.30), n2) * (0.85 + 0.3 * n3);
	// patches of worn grass in urban ground
	float patchy = smoothstep(0.45, 0.6, fbm(w * 0.25));
	urban = mix(urban, grass * 0.85, patchy * 0.6);
	vec3 col = grass * m.r + forest * m.g + soil * m.b + urban * m.a;
	float rough = 0.95;
	apply_wet(col, rough, 0.7);
	ALBEDO = col;
	ROUGHNESS = rough;
	SPECULAR = 0.25;
}
