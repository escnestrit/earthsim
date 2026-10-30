// LED advertising screens on shopping centres: a looping programme of procedural "ads" (colour fields,
// moving shapes, lines of fake text, a logo disc), with a visible LED pixel grid up close.
shader_type spatial;
render_mode cull_back;

uniform float brightness = 1.6;
uniform float still = 0.0;  // posters: one ad, no animation, no LED grid, lit only by the scene

float hash(float n) { return fract(sin(n) * 43758.5453); }

vec3 palette(float t) {
	return 0.5 + 0.5 * cos(6.2831 * (vec3(0.0, 0.33, 0.67) + t));
}

void fragment() {
	float seed = COLOR.r * 97.0;
	float aspect = max(COLOR.g * 4.0, 0.5);
	vec2 uv = UV;
	vec2 p = vec2(uv.x * aspect, uv.y);
	float t = still > 0.5 ? seed * 37.0 + 3.0 : TIME + seed * 13.0;
	float slot = floor(t / 7.0);
	float ph = still > 0.5 ? 0.5 : fract(t / 7.0);
	float kind = floor(hash(slot + seed) * 4.0);
	vec3 c1 = palette(hash(slot * 1.7 + seed));
	vec3 c2 = palette(hash(slot * 2.3 + seed + 0.5));
	vec3 col;
	if (kind < 1.0) {
		// diagonal sweep with a big disc "product"
		col = mix(c1, c2, smoothstep(0.0, 1.0, uv.x + uv.y * 0.5 - ph * 0.6));
		float d = length(p - vec2(aspect * 0.7, 0.5));
		col = mix(col, vec3(1.0), smoothstep(0.32, 0.3, d) * 0.85);
		col = mix(col, c2 * 0.4, smoothstep(0.22, 0.2, d));
	} else if (kind < 2.0) {
		// headline bars scrolling in
		col = c1 * 0.25;
		for (int i = 0; i < 4; i++) {
			float y = 0.2 + float(i) * 0.17;
			float len = 0.4 + hash(slot + float(i)) * 0.5;
			float x0 = 0.08 + (1.0 - smoothstep(0.0, 0.25 + float(i) * 0.08, ph)) * 1.2;
			float bar = step(y, uv.y) * step(uv.y, y + (i == 0 ? 0.12 : 0.06)) * step(x0, uv.x) * step(uv.x, x0 + len);
			col = mix(col, i == 0 ? vec3(1.0) : c2, bar);
		}
	} else if (kind < 3.0) {
		// pulsing rings
		float d = length(p - vec2(aspect * 0.5, 0.5));
		col = mix(c1, c2, 0.5 + 0.5 * sin(d * 18.0 - t * 4.0));
		col *= 0.6 + 0.4 * smoothstep(0.9, 0.1, d);
	} else {
		// split screen with a price tag
		col = uv.x < 0.55 ? c1 : c2 * 0.8;
		vec2 q = p - vec2(aspect * 0.78, 0.68);
		float tag = step(abs(q.x), 0.22 * aspect * 0.4) * step(abs(q.y), 0.12);
		col = mix(col, vec3(1.0, 0.85, 0.1), tag);
		float digits = tag * step(0.5, fract(q.x * 20.0)) * step(abs(q.y), 0.06);
		col = mix(col, vec3(0.1), digits);
	}
	// fade between ads
	col *= smoothstep(0.0, 0.05, ph) * smoothstep(1.0, 0.95, ph);
	// LED grid
	vec2 g = fract(vec2(uv.x * aspect, uv.y) * 90.0);
	float led = smoothstep(0.0, 0.2, g.x) * smoothstep(1.0, 0.8, g.x) * smoothstep(0.0, 0.2, g.y) * smoothstep(1.0, 0.8, g.y);
	if (still < 0.5) {
		col *= mix(0.55, 1.0, led);
		ALBEDO = col * 0.15;
		EMISSION = col * brightness;
	} else {
		ALBEDO = col * brightness;
		ROUGHNESS = 0.6;
	}
}
