// Building walls with procedural windows.
// UV.x metres along the wall, UV.y metres above the building base; UV2.x floor height, UV2.y style;
// COLOR.rgb facade colour, COLOR.a per-building seed.
// styles: 0 panel block, 1 brick, 2 plaster (old town), 3 glass office, 4 industrial, 5 house, 6 garage/shed
shader_type spatial;
render_mode cull_back;
#include "res://shaders/common.gdshaderinc"

varying vec2 info;
varying vec3 wpos;

void vertex() {
	info = UV2;
	wpos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
	float fh = max(info.x, 2.4);
	int style = int(info.y + 0.5);
	float seed = COLOR.a;
	vec3 wall = COLOR.rgb;
	float x = UV.x, y = UV.y;
	float n = fbm(vec2(x, y) * 0.8 + seed * 50.0);
	wall *= 0.9 + 0.15 * n;
	float rough = 0.85;
	float metal = 0.0;
	vec3 emit = vec3(0.0);
	// geometry of the window grid per style
	float bay = 3.0, ww = 1.4, wh = 1.5, sill = 0.9;
	if (style == 1) { bay = 2.7; ww = 1.3; wh = 1.6; sill = 0.85; }
	else if (style == 2) { bay = 3.3; ww = 1.2; wh = 1.9; sill = 0.8; }
	else if (style == 3) { bay = 1.6; ww = 1.5; wh = fh - 0.9; sill = 0.45; }
	else if (style == 5) { bay = 3.2; ww = 1.2; wh = 1.35; sill = 0.9; }
	float floor_i = floor(y / fh);
	float fy = y - floor_i * fh;
	float bay_i = floor(x / bay);
	float fx = x - bay_i * bay - (bay - ww) * 0.5;
	bool win = fx > 0.0 && fx < ww && fy > sill && fy < sill + wh && y > 0.6;
	if (style == 4) {
		// industrial: one band of windows high on each 5 m level, few of them
		win = fy > fh * 0.55 && fy < fh * 0.85 && fx > 0.0 && fx < ww * 1.8 && h21(vec2(bay_i, floor_i) + seed) > 0.35;
	}
	if (style == 6) win = false;
	// ground floor of plaster/glass buildings: tall shop windows
	if ((style == 2 || style == 3) && floor_i < 0.5 && seed > 0.45) {
		win = fx > -0.3 && fx < ww + 0.3 && fy > 0.5 && fy < fh - 0.5;
	}
	// panel joints
	if (style == 0) {
		float jx = abs(fract(x / bay + 0.5) - 0.5) * bay, jy = abs(fract(y / fh) - 0.5) * fh;
		float joint = (1.0 - smoothstep(0.02, 0.05, abs(jx - bay * 0.5))) + (1.0 - smoothstep(0.02, 0.05, abs(jy - fh * 0.5)));
		wall *= 1.0 - 0.25 * clamp(joint, 0.0, 1.0);
	}
	if (style == 1) {
		// bricks
		vec2 b = vec2(x / 0.25, y / 0.075);
		b.x += floor(b.y) * 0.5;
		vec2 f = abs(fract(b) - 0.5);
		float mortar = smoothstep(0.38, 0.47, max(f.x * 0.9, f.y));
		wall = mix(wall * (0.85 + 0.3 * h21(floor(b) + seed)), vec3(0.62, 0.6, 0.56), mortar * 0.7);
	}
	// plinth and storey bands
	if (y < 0.6) wall = mix(wall, vec3(0.32, 0.31, 0.3), 0.75);
	if (style == 2 && abs(fy - fh) < 0.12) wall *= 0.85;
	vec3 col = wall;
	if (win) {
		// frame
		float fr = min(min(fx, ww - fx), min(fy - sill, sill + wh - fy));
		float frame = 1.0 - smoothstep(0.04, 0.08, fr);
		float cell = h21(vec2(bay_i * 1.7 + seed * 31.0, floor_i * 3.1));
		vec3 glass = mix(vec3(0.08, 0.1, 0.12), vec3(0.22, 0.26, 0.3), cell * 0.6);
		// curtains/blinds inside some windows
		glass = mix(glass, vec3(0.35, 0.32, 0.28), step(0.72, cell) * 0.5);
		col = mix(glass, vec3(0.85, 0.85, 0.82), frame);
		rough = mix(0.08, 0.6, frame);
		metal = (1.0 - frame) * 0.15;
		float lit = step(0.7, h21(vec2(bay_i + seed * 13.0, floor_i + seed * 7.0)));
		emit = vec3(1.0, 0.78, 0.5) * lit * night * 1.8 * (1.0 - frame);
	}
	if (style == 3 && !win) {
		col = mix(col, vec3(0.25, 0.27, 0.3), 0.3);
	}
	apply_wet(col, rough, win ? 0.0 : 0.5);
	ALBEDO = col;
	ROUGHNESS = rough;
	METALLIC = metal;
	EMISSION = emit;
}
