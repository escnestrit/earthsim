// Sky: gradient, sun disc and glow, moving fbm clouds whose cover goes up with rain.
shader_type sky;

uniform vec3 top_color = vec3(0.32, 0.5, 0.78);
uniform vec3 horizon_color = vec3(0.68, 0.76, 0.84);
uniform vec3 ground_color = vec3(0.25, 0.26, 0.28);
uniform float cloud_cover = 0.35;   // 0 clear .. 1 overcast
uniform float night = 0.0;

float h(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }
float n2(vec2 p) {
	vec2 i = floor(p), f = fract(p);
	vec2 u = f * f * (3.0 - 2.0 * f);
	return mix(mix(h(i), h(i + vec2(1, 0)), u.x), mix(h(i + vec2(0, 1)), h(i + vec2(1, 1)), u.x), u.y);
}
float fbm(vec2 p) {
	float s = 0.0, a = 0.5;
	for (int i = 0; i < 5; i++) { s += a * n2(p); p = p * 2.03 + vec2(1.7, 9.2); a *= 0.5; }
	return s;
}

void sky() {
	vec3 d = EYEDIR;
	float y = d.y;
	vec3 col;
	if (y >= 0.0) {
		col = mix(horizon_color, top_color, pow(clamp(y, 0.0, 1.0), 0.55));
		// sun
		vec3 sd = LIGHT0_DIRECTION;
		float sdot = max(dot(d, sd), 0.0);
		float day = LIGHT0_ENABLED ? 1.0 - night : 0.0;
		col += LIGHT0_COLOR * (pow(sdot, 900.0) * 18.0 + pow(sdot, 12.0) * 0.25) * day * (1.0 - cloud_cover * 0.85);
		// clouds on a flat layer
		vec2 uv = d.xz / max(y + 0.08, 0.02) * 0.6 + vec2(TIME * 0.004, TIME * 0.002);
		float c = fbm(uv * 1.3);
		float cov = mix(0.62, 0.25, cloud_cover);
		float cl = smoothstep(cov, cov + 0.25, c) * smoothstep(0.0, 0.12, y);
		vec3 lit = mix(vec3(1.0), horizon_color * 1.2, 0.25) * (1.0 - night * 0.85);
		vec3 shade = mix(lit * 0.62, vec3(0.45, 0.47, 0.5) * (1.0 - night * 0.9), cloud_cover);
		vec3 ccol = mix(lit, shade, smoothstep(0.4, 0.95, c));
		col = mix(col, ccol, cl * (0.85 + 0.15 * cloud_cover));
		// overcast: everything goes grey
		col = mix(col, vec3(0.55, 0.57, 0.6) * (1.0 - night * 0.92), cloud_cover * 0.65);
	} else {
		col = mix(horizon_color, ground_color, clamp(-y * 4.0, 0.0, 1.0));
	}
	COLOR = col;
}
