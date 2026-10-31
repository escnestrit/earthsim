// Ground: COLOR weights r grass, g forest floor, b bare soil/sand, a urban dirt/gravel.
shader_type spatial;
render_mode cull_back;
#include "res://shaders/common.gdshaderinc"

varying vec3 wpos;
varying float paved;

void vertex() {
	wpos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	paved = UV2.x;
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
	urban = mix(urban, grass * 0.85, patchy * 0.35);
	// climate of the region
	int bi = int(biome + 0.5);
	if (bi == 1) { grass *= vec3(0.85, 0.95, 0.9); forest *= vec3(0.8, 0.9, 0.85); }
	else if (bi == 2) { grass = mix(grass, vec3(0.42, 0.40, 0.22) * (0.8 + 0.3 * n2), 0.55); }
	else if (bi == 3) {
		vec3 sand = mix(vec3(0.66, 0.55, 0.40), vec3(0.76, 0.66, 0.50), n1) * (0.85 + 0.25 * n3);
		grass = mix(sand, grass * vec3(1.1, 0.9, 0.7), 0.25 * smoothstep(0.55, 0.75, n2));
		forest = mix(forest, sand * 0.8, 0.5);
		soil = mix(soil, sand, 0.6);
	}
	else if (bi == 4) { grass *= vec3(0.85, 1.12, 0.8); forest *= vec3(0.8, 1.1, 0.8); }
	else if (bi == 5) {
		grass = mix(grass * vec3(0.95, 0.9, 0.85), vec3(0.82, 0.84, 0.86), smoothstep(0.5, 0.7, n1));
	}
	vec3 col = grass * m.r + forest * m.g + soil * m.b + urban * m.a;
	float rough = 0.95;
	if (paved > 0.25) {
		// squares: granite paving; car parks: worn asphalt
		vec3 pv;
		if (paved > 0.75) {
			vec2 g = w / 0.6;
			vec2 f = abs(fract(g + vec2(0.0, floor(g.x) * 0.5)) - 0.5);
			float groove = smoothstep(0.44, 0.49, max(f.x, f.y));
			pv = vec3(0.47, 0.45, 0.42) * (0.88 + 0.18 * h21(floor(g))) * (0.9 + 0.2 * n2);
			pv = mix(pv, pv * 0.7, groove);
		} else {
			pv = vec3(0.2, 0.2, 0.205) * (0.85 + 0.3 * n2);
		}
		float k = smoothstep(0.25, 0.6, paved);
		col = mix(col, pv, k);
		rough = mix(rough, 0.8, k);
	}
	apply_wet(col, rough, 0.7);
	ALBEDO = col;
	ROUGHNESS = rough;
	SPECULAR = 0.25;
}
