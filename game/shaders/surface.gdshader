// Pavement, kerbs, concrete, footpaths, rails: one shader, behaviour picked by `kind`.
// kind 0 pavement (tiles), 1 kerb (granite), 2 concrete, 3 footpath (UV2.x 1 paved / 0 dirt), 4 rail
shader_type spatial;
render_mode cull_back;
#include "res://shaders/common.gdshaderinc"

uniform int kind = 0;
varying vec3 wpos;
varying vec2 info;

void vertex() {
	wpos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	info = UV2;
}

void fragment() {
	vec2 w = wpos.xz;
	float n = fbm(w * 1.3);
	vec3 col;
	float rough = 0.85;
	if (kind == 0) {
		vec2 g = w / vec2(0.5, 0.5);
		vec2 f = abs(fract(g) - 0.5);
		float groove = smoothstep(0.44, 0.49, max(f.x, f.y));
		vec3 base = vec3(0.52, 0.5, 0.47) * (0.85 + 0.25 * h21(floor(g))) * (0.85 + 0.3 * n);
		col = mix(base, base * 0.55, groove);
	} else if (kind == 1) {
		col = vec3(0.58, 0.58, 0.56) * (0.85 + 0.3 * vnoise(w * 6.0 + wpos.y));
		rough = 0.7;
	} else if (kind == 2) {
		col = vec3(0.5, 0.5, 0.48) * (0.8 + 0.35 * fbm(vec2(w.x + w.y, wpos.y) * 0.7));
		rough = 0.9;
	} else if (kind == 3) {
		if (info.x > 0.5) {
			col = vec3(0.42, 0.41, 0.4) * (0.85 + 0.3 * n);
		} else {
			col = vec3(0.36, 0.29, 0.21) * (0.8 + 0.4 * n);
			rough = 0.95;
		}
	} else {
		// rails: UV.x across (0..width), UV.y along
		float c = info.y * 0.5;
		float u = UV.x - c;
		float rail = max(1.0 - smoothstep(0.03, 0.05, abs(u - 0.76)), 1.0 - smoothstep(0.03, 0.05, abs(u + 0.76)));
		if (info.x > 1.5) {
			// tram rails set into the street
			if (rail < 0.5 && abs(abs(u) - 0.76) > 0.1) discard;
			col = mix(vec3(0.12), vec3(0.55, 0.55, 0.58), rail);
		} else {
			float sleeper = step(fract(UV.y / 0.55), 0.35) * step(abs(u), 1.3);
			vec3 ballast = vec3(0.35, 0.33, 0.31) * (0.6 + 0.6 * vnoise(w * 18.0));
			col = mix(ballast, vec3(0.28, 0.24, 0.2), sleeper);
			col = mix(col, vec3(0.5, 0.5, 0.52), rail);
		}
		rough = mix(0.9, 0.35, rail);
		METALLIC = rail * 0.8;
	}
	apply_wet(col, rough, kind == 4 ? 0.5 : 1.0);
	ALBEDO = col;
	ROUGHNESS = rough;
}
