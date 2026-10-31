// Carriageway: asphalt (or setts) with lane markings drawn from the ribbon's UVs.
// UV.x = metres from the left edge, UV.y = arc length; UV2.x = lanes + 16*flags, UV2.y = width.
// flags: 1 oneway, 2 edge lines, 4 double centre line, 8 setts
shader_type spatial;
render_mode cull_back;
#include "res://shaders/common.gdshaderinc"

uniform bool junction = false;
varying vec3 wpos;
varying vec2 info;

void vertex() {
	wpos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	info = UV2;
}

float band(float x, float c, float hw) {
	float aa = fwidth(x) * 0.75 + 0.002;
	return 1.0 - smoothstep(hw - aa, hw + aa, abs(x - c));
}

void fragment() {
	vec2 w = wpos.xz;
	float n = fbm(w * 0.9) * 0.6 + vnoise(w * 7.0) * 0.4;
	float big = fbm(w * 0.05);
	vec3 col = vec3(0.16, 0.16, 0.165) * (0.82 + 0.3 * n) * (0.9 + 0.2 * big);
	float rough = 0.86 - 0.12 * n;
	float code = floor(info.x + 0.5);
	float flags = floor(code / 16.0);
	float lanes = code - flags * 16.0;
	float width = info.y;
	bool setts = mod(flags, 16.0) >= 8.0;
	if (setts && !junction) {
		vec2 g = vec2(UV.x * 5.0, UV.y * 2.5);
		vec2 f = abs(fract(g + vec2(0.0, floor(g.x) * 0.5)) - 0.5);
		float edge = smoothstep(0.42, 0.48, max(f.x, f.y));
		col = mix(vec3(0.33, 0.31, 0.3) * (0.8 + 0.4 * h21(floor(g))), vec3(0.12), edge);
		rough = 0.75;
	}
	float paint = 0.0;
	if (!junction && width > 0.0) {
		bool oneway = mod(flags, 2.0) >= 1.0;
		bool edges = mod(flags, 4.0) >= 2.0;
		bool dbl = mod(flags, 8.0) >= 4.0;
		float u = UV.x, v = UV.y;
		float dash = step(fract(v / 12.0), 0.25);  // 3 m dash, 9 m gap
		if (edges && width > 5.0) {
			paint = max(paint, band(u, 0.35, 0.06));
			paint = max(paint, band(u, width - 0.35, 0.06));
		}
		if (!oneway && lanes >= 2.0) {
			float c = width * 0.5;
			if (dbl) paint = max(paint, max(band(u, c - 0.12, 0.06), band(u, c + 0.12, 0.06)));
			else paint = max(paint, band(u, c, 0.06) * dash);
			float per = max(1.0, floor(lanes / 2.0));
			for (float k = 1.0; k < per; k += 1.0) {
				float o = c * k / per;
				paint = max(paint, band(u, c - o, 0.06) * dash);
				paint = max(paint, band(u, c + o, 0.06) * dash);
			}
		} else if (oneway && lanes >= 2.0) {
			for (float k = 1.0; k < lanes; k += 1.0) paint = max(paint, band(u, width * k / lanes, 0.06) * dash);
		}
		paint *= 0.75 + 0.25 * smoothstep(0.25, 0.6, vnoise(w * 3.0));  // worn paint
	}
	col = mix(col, vec3(0.82, 0.82, 0.78), paint);
	rough = mix(rough, 0.55, paint);
	apply_wet(col, rough, 1.0 - paint * 0.5);
	ALBEDO = col;
	ROUGHNESS = rough;
	SPECULAR = 0.4;
}
