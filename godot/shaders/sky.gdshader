// Sky: atmospheric gradient lit by the sun (warm near the horizon at dusk), sun disk and halo, drifting
// layered clouds, stars at night.
shader_type sky;
render_mode use_debanding;

uniform vec3 zenith : source_color = vec3(0.2, 0.38, 0.72);
uniform vec3 horizon : source_color = vec3(0.68, 0.79, 0.92);
uniform vec3 ground_col : source_color = vec3(0.27, 0.28, 0.27);
uniform float cloud_cover : hint_range(0.0, 1.0) = 0.45;
uniform float cloud_speed = 0.004;
// the real sun direction (the key light becomes the moon at night)
uniform vec3 sun_dir = vec3(0.0, 1.0, 0.0);

float hash(vec2 p) {
	p = fract(p * vec2(123.34, 456.21));
	p += dot(p, p + 45.32);
	return fract(p.x * p.y);
}
float noise(vec2 p) {
	vec2 i = floor(p), f = fract(p);
	f = f * f * (3.0 - 2.0 * f);
	return mix(mix(hash(i), hash(i + vec2(1, 0)), f.x), mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), f.x), f.y);
}
float fbm(vec2 p) {
	float s = 0.0, a = 0.5;
	for (int i = 0; i < 6; i++) {
		s += noise(p) * a;
		p = p * 2.03 + vec2(1.7, 9.2);
		a *= 0.5;
	}
	return s;
}

void sky() {
	vec3 d = normalize(EYEDIR);
	vec3 sd = normalize(sun_dir);
	float sun_h = sd.y;
	float day = smoothstep(-0.12, 0.25, sun_h);
	float dusk = smoothstep(0.35, 0.0, abs(sun_h - 0.02));
	float up = max(d.y, 0.0);
	vec3 zen = mix(vec3(0.01, 0.015, 0.04), zenith, day);
	vec3 hor = mix(vec3(0.03, 0.04, 0.08), horizon, day);
	hor = mix(hor, vec3(1.0, 0.55, 0.3), dusk * 0.7 * pow(max(dot(normalize(vec2(d.x, d.z)), normalize(vec2(sd.x, sd.z))), 0.0), 2.0));
	vec3 col = mix(hor, zen, pow(up, 0.45));
	float cosang = dot(d, sd);
	// sun disk and glow
	float disk = smoothstep(0.9996, 0.99985, cosang);
	col += vec3(1.0, 0.92, 0.8) * disk * 40.0 * day;
	col += vec3(1.0, 0.75, 0.5) * pow(max(cosang, 0.0), 64.0) * 0.6 * (day + dusk);
	col += vec3(1.0, 0.85, 0.7) * pow(max(cosang, 0.0), 6.0) * 0.12 * day;
	// clouds on a dome
	if (d.y > 0.0) {
		vec2 uv = d.xz / (d.y + 0.08) * 1.4 + TIME * cloud_speed * vec2(1.0, 0.4);
		float c = fbm(uv * 1.3);
		float cov = smoothstep(1.0 - cloud_cover, 1.0 - cloud_cover + 0.35, c);
		float shade = fbm(uv * 1.3 + sd.xz * 0.06);
		vec3 lit = mix(vec3(0.55, 0.58, 0.64), vec3(1.0, 0.98, 0.95), clamp(1.0 - (shade - c) * 4.0, 0.0, 1.0));
		lit = mix(lit, vec3(1.0, 0.62, 0.4), dusk * 0.6);
		lit *= mix(0.06, 1.0, day);
		float fade = smoothstep(0.0, 0.18, d.y);
		col = mix(col, lit, cov * fade * 0.92);
		// stars
		float night = 1.0 - smoothstep(-0.2, 0.0, sun_h);
		vec2 sp = d.xz / (d.y + 0.3) * 160.0;
		float st = step(0.997, hash(floor(sp))) * night * (1.0 - cov);
		col += vec3(st);
	} else {
		col = mix(hor * 0.9, ground_col * mix(0.05, 1.0, day), smoothstep(0.0, -0.25, d.y));
	}
	COLOR = col;
}
