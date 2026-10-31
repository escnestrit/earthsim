// Building walls with procedural windows and details.
// UV.x metres along the wall, UV.y metres above the building base;
// UV2.x = floor height (+100 when the ground floor of this wall is a shop front);
// UV2.y = style + 32 * number of storeys with windows (0 = no limit);
// COLOR.rgb facade colour, COLOR.a per-building seed.
// styles: 0 panel block, 1 brick, 2 old town plaster, 3 glass office, 4 industrial, 5 house, 6 shed,
//         7/8/9 balcony (open, glazed, slab), 10 Stalinist, 11 modern housing, 12 tiled prefab (khrushchevka),
//         13 public building, 14 church, 15 plain trim (cornices, gables, canopies), 16 entrance door
shader_type spatial;
render_mode cull_back;
#include "res://shaders/common.gdshaderinc"

varying vec2 info;
varying vec3 wpos;

void vertex() {
	info = UV2;
	wpos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

vec3 accent_colour(float s) {
	float k = fract(s * 7.31);
	if (k < 0.25) return vec3(0.72, 0.36, 0.2);
	if (k < 0.5) return vec3(0.28, 0.42, 0.58);
	if (k < 0.75) return vec3(0.45, 0.55, 0.32);
	return vec3(0.82, 0.62, 0.25);
}

void fragment() {
	if (far_hidden(wpos)) discard;
	float shop_flag = step(99.0, info.x);
	float fh = max(info.x - 100.0 * shop_flag, 2.4);
	float nfl = floor(info.y / 32.0 + 0.01);
	int style = int(info.y - nfl * 32.0 + 0.5);
	if (nfl < 0.5) nfl = 999.0;
	float seed = floor(COLOR.a * 4096.0 + 0.5) / 4096.0;  // exact: the hashes below amplify tiny differences
	vec3 wall = COLOR.rgb;
	float x = UV.x, y = UV.y;
	if (style >= 7 && style <= 9) {
		// balconies
		vec3 c = wall * (0.9 + 0.1 * vnoise(vec2(x, y) * 3.0));
		float r = 0.85;
		vec3 e = vec3(0.0);
		if (style == 8 && y > 1.15) {
			float fx = fract(x / 0.9);
			float fr = min(min(fx, 1.0 - fx) * 0.9, min(y - 1.15, fh * 0.92 + 0.15 - y));
			float frame = 1.0 - smoothstep(0.03, 0.06, fr);
			float cell = h21(vec2(floor(x / 0.9) + seed * 17.0, floor(wpos.y / fh)));
			c = mix(mix(vec3(0.1, 0.12, 0.14), vec3(0.3, 0.33, 0.36), cell * 0.5), vec3(0.88, 0.88, 0.86), frame);
			r = mix(0.08, 0.5, frame);
			e = vec3(1.0, 0.8, 0.55) * step(0.82, cell) * night * 1.2 * (1.0 - frame);
		} else if (style == 7) {
			float bars = step(0.5, fract(x / 0.12)) * step(0.1, y) * step(y, 0.95);
			c = mix(c, c * 0.75, bars * 0.5);
		} else if (style == 9) {
			c *= 0.8;
		}
		apply_wet(c, r, 0.4);
		ALBEDO = c;
		ROUGHNESS = r;
		EMISSION = e;
	} else if (style == 22) {
		// LED screen: pixel grid showing slowly changing colourful content
		vec2 px = floor(vec2(x, y) / 0.08);
		vec2 g = fract(vec2(x, y) / 0.08);
		float t = TIME * 0.25 + seed * 40.0;
		float scene = floor(t / 6.0);
		vec2 q = vec2(x, y) / max(info.x, 1.0);
		vec3 c1 = 0.5 + 0.5 * cos(6.2831 * (vec3(0.0, 0.33, 0.67) + h21(vec2(scene, 1.0)) + q.x * 0.6 + t * 0.3));
		float blob = smoothstep(0.35, 0.0, length(q - vec2(0.5 + 0.3 * sin(t * 1.3), 0.5 + 0.2 * cos(t))) - 0.15);
		float band = step(0.8, fract(q.y * 3.0 + t * 0.5)) * step(0.5, h21(vec2(scene, 7.0)));
		vec3 c = mix(c1 * 0.6, vec3(1.0), blob * 0.7) + band * 0.4;
		float grid = step(0.12, g.x) * step(0.12, g.y);
		c *= grid;
		ALBEDO = c * 0.15;
		ROUGHNESS = 0.3;
		EMISSION = c * mix(1.4, 3.0, night);
	} else if (style == 24) {
		// lit sign: block lettering on the sign colour
		float hh = max(info.x, 0.5);
		vec3 bg = wall;
		float lum = dot(bg, vec3(0.3, 0.59, 0.11));
		vec3 fg = lum > 0.7 ? vec3(0.08, 0.1, 0.25) : vec3(1.0, 0.98, 0.92);
		float ty = (y - hh * 0.2) / (hh * 0.6);
		float cw = hh * 0.6 * 0.62;
		float ci = floor(x / cw);
		float cx = fract(x / cw);
		float word = step(0.15, h21(vec2(floor(ci / 7.0), seed * 9.0)));
		vec2 cell = floor(vec2(cx * 4.0, ty * 5.0));
		float on = step(0.42, h21(cell + vec2(ci * 5.3, seed * 33.0)));
		on *= step(0.0, ty) * step(ty, 1.0) * step(0.12, cx) * step(cx, 0.88) * word;
		on *= step(0.6, x) * step(x, info.x * 0.0 + 1e9);
		vec3 c = mix(bg, fg, on);
		ALBEDO = c * 0.8;
		ROUGHNESS = 0.4;
		EMISSION = c * mix(0.25, 1.8, night);
	} else if (style == 26) {
		// stadium stands: rows of coloured seats with aisles
		float row = fract(y / 0.8);
		vec3 seat = vec3(0.12, 0.3, 0.7);
		float k = fract(seed * 5.7);
		if (k < 0.25) seat = vec3(0.7, 0.12, 0.12); else if (k < 0.5) seat = vec3(0.9, 0.75, 0.15); else if (k < 0.7) seat = vec3(0.12, 0.5, 0.25);
		float sector = h21(vec2(floor(x / 18.0), seed));
		seat = mix(seat, vec3(0.85), step(0.85, sector) * 0.6);
		vec3 c = row < 0.55 ? seat * (0.85 + 0.15 * row) : vec3(0.55, 0.55, 0.53);
		if (abs(fract(x / 14.0) - 0.5) * 14.0 < 0.6) c = vec3(0.6, 0.6, 0.58);
		ALBEDO = c;
		ROUGHNESS = 0.6;
	} else if (style == 15) {
		vec3 c = wall * (0.92 + 0.12 * fbm(vec2(x, y) * 1.3 + seed * 20.0));
		float r = 0.85;
		apply_wet(c, r, 0.5);
		ALBEDO = c;
		ROUGHNESS = r;
	} else if (style == 16) {
		// door: two leaves, frame, small glazing on top
		vec3 c = wall;
		float fr = min(min(x, 1.5 - x), min(y, 2.45 - y));
		float frame = 1.0 - smoothstep(0.05, 0.09, fr);
		float split = 1.0 - smoothstep(0.015, 0.03, abs(x - 0.75));
		float glass = step(1.75, y) * step(y, 2.3) * step(0.15, x) * step(x, 1.35);
		c = mix(c * (0.85 + 0.1 * vnoise(vec2(x * 4.0, y * 0.5))), c * 0.55, max(frame, split));
		c = mix(c, vec3(0.1, 0.12, 0.14), glass * (1.0 - split));
		float r = mix(0.6, 0.1, glass);
		ALBEDO = c;
		ROUGHNESS = r;
		EMISSION = vec3(1.0, 0.85, 0.6) * glass * night * 0.8;
	} else {
	float n = fbm(vec2(x, y) * 0.8 + seed * 50.0);
	wall *= 0.9 + 0.15 * n;
	float rough = 0.85;
	float metal = 0.0;
	vec3 emit = vec3(0.0);
	// window grid per style (bays must match style_bay() in world.cpp)
	float bay = 3.0, ww = 1.4, wh = 1.5, sill = 0.9;
	float vary = fract(seed * 13.7);
	if (style == 1) { bay = 2.7; ww = 1.3; wh = 1.5; sill = 0.85; }
	else if (style == 2) { bay = 3.3; ww = 1.15 + 0.15 * vary; wh = min(2.1, fh - 1.5); sill = 0.85; }
	else if (style == 3) { bay = 1.6; ww = 1.5; wh = fh - 0.9; sill = 0.45; }
	else if (style == 5) { bay = 3.2; ww = 1.2; wh = 1.35; sill = 0.9; }
	else if (style == 10) { bay = 3.2; ww = 1.35; wh = min(1.95, fh - 1.3); sill = 0.85; }
	else if (style == 11) { bay = 3.0; ww = 1.5 + 0.4 * vary; wh = 1.7; sill = 0.7; }
	else if (style == 12) { bay = 3.2; ww = 1.5; wh = 1.4; sill = 0.85; }
	else if (style == 13) { bay = 3.4; ww = 2.3; wh = min(2.0, fh - 1.4); sill = 0.9; }
	else if (style == 14) { bay = 4.2; ww = 1.3; wh = fh * 2.0 - 2.5; sill = 1.8; }
	else if (style == 17) { bay = 3.2; ww = 1.0; wh = 1.4; sill = 0.9; }
	else if (style == 18) { bay = 3.5; ww = 1.0 + 0.3 * vary; wh = 1.2; sill = 1.0; }
	else if (style == 19) { bay = 3.0; ww = 1.4; wh = 1.5; sill = 0.9; }
	else if (style == 20) { bay = 4.2; ww = 1.5; wh = fh * 0.62; sill = fh * 0.18; }
	else if (style == 21) { bay = 3.0; ww = 2.6; wh = 1.0; sill = fh * 0.45; }
	else if (style == 23) { bay = 8.0; ww = 7.4; wh = fh - 1.1; sill = 0.95; }
	else if (style == 25) { bay = 2.0; ww = 1.2; wh = fh * 0.45; sill = fh * 0.3; }
	float floor_i = floor(y / fh);
	float fy = y - floor_i * fh;
	if (style == 14) { fy = y; floor_i = 0.0; }
	float bay_i = floor(x / bay);
	float fx = x - bay_i * bay - (bay - ww) * 0.5;
	bool shop = shop_flag > 0.5 && floor_i < 0.5 && y > 0.0;
	if (style == 21 && floor_i < 0.5 && y > 0.0) shop = true;
	bool win = fx > 0.0 && fx < ww && fy > sill && fy < sill + wh && y > 0.6 && floor_i < nfl && floor_i >= 0.0;
	if (style == 21 && floor_i > 0.5) win = win && h21(vec2(floor(seed * 64.0), floor_i)) > 0.45;  // ribbon windows on some floors only
	if (style == 18) win = win && h21(vec2(bay_i, floor_i) + seed * 3.0) > 0.18;  // irregular, a few blank bays
	if (style == 14) {
		// arched church windows
		float cx = fx - ww * 0.5, top = sill + wh - ww * 0.5;
		win = abs(cx) < ww * 0.5 && fy > sill && (fy < top || length(vec2(cx, fy - top)) < ww * 0.5);
	}
	if (style == 4) {
		// industrial: one band of windows high on each 5 m level, few of them
		win = fy > fh * 0.55 && fy < fh * 0.85 && fx > 0.0 && fx < ww * 1.8 && h21(vec2(bay_i, floor_i) + seed) > 0.35 && floor_i < nfl;
	}
	if (style == 6) win = false;
	// glass offices: a few have a shop/lobby base too
	if (style == 3 && floor_i < 0.5 && y > 0.0 && seed > 0.45) shop = true;
	float sx = x - bay_i * bay;
	if (shop) {
		// shop front: wide display windows, sign band above, a glazed door every few bays
		win = sx > 0.3 && sx < bay - 0.3 && fy > 0.45 && fy < fh - 0.85;
	}
	// ---- wall surface per style
	if (style == 0 || style == 12) {
		float jx = abs(fract(x / bay + 0.5) - 0.5) * bay, jy = abs(fract(y / fh) - 0.5) * fh;
		float joint = (1.0 - smoothstep(0.02, 0.05, abs(jx - bay * 0.5))) + (1.0 - smoothstep(0.02, 0.05, abs(jy - fh * 0.5)));
		wall *= 1.0 - 0.25 * clamp(joint, 0.0, 1.0);
		if (style == 12) {
			// small ceramic tiles
			vec2 t = abs(fract(vec2(x, y) / 0.12) - 0.5);
			wall *= 1.0 - 0.08 * smoothstep(0.38, 0.48, max(t.x, t.y));
			wall *= 0.95 + 0.1 * h21(floor(vec2(x, y) / 0.12) + seed);
		}
	}
	if (style == 1) {
		vec2 b = vec2(x / 0.25, y / 0.075);
		b.x += floor(b.y) * 0.5;
		vec2 f = abs(fract(b) - 0.5);
		float mortar = smoothstep(0.38, 0.47, max(f.x * 0.9, f.y));
		wall = mix(wall * (0.88 + 0.24 * h21(floor(b) + seed)), vec3(0.62, 0.6, 0.56), mortar * 0.6);
	}
	vec3 trim = mix(wall, vec3(0.95, 0.93, 0.88), 0.55);
	if (style == 2 || style == 10 || style == 13) {
		// rusticated ground floor
		if (floor_i < 0.5 && !shop) {
			float g = 1.0 - smoothstep(0.02, 0.05, abs(fract(y / 0.45) - 0.5) * 0.45 - 0.2);
			wall *= 1.0 - 0.18 * g;
			if (style == 10) wall = mix(wall, vec3(0.55, 0.5, 0.45), 0.35);
		}
		// storey band
		if (abs(fy - fh + 0.1) < 0.1 && floor_i < nfl - 0.5) wall = trim * 0.95;
		// window surround and sandrik (cornice over the window)
		if (!win && floor_i > 0.5 && floor_i < nfl) {
			bool surround = fx > -0.18 && fx < ww + 0.18 && fy > sill - 0.12 && fy < sill + wh + 0.15;
			bool sandrik = fx > -0.32 && fx < ww + 0.32 && fy > sill + wh + 0.18 && fy < sill + wh + 0.38 && (style != 13);
			if (surround || sandrik) wall = trim * (sandrik ? 0.9 : 1.0);
		}
		// pilasters between every second bay on Stalinist facades
		if (style == 10 && floor_i > 0.5 && floor_i < nfl) {
			float px = abs(fract(x / (bay * 2.0)) - 0.0) * bay * 2.0;
			if (px < 0.5) wall = mix(wall, trim, 0.6);
		}
	}
	if (style == 11) {
		// modern housing: accent stripes on some bays
		float grp = floor(x / (bay * 3.0));
		if (h21(vec2(grp, seed * 9.0)) > 0.6 && sx > 0.0) wall = mix(wall, accent_colour(seed + grp * 0.13), 0.75);
		if (abs(fy - fh) < 0.15) wall *= 0.9;
	}
	if (style == 14) wall *= 1.0;
	if (style == 17) {
		// clapboard siding
		float board = fract(y / 0.2);
		wall *= 0.86 + 0.14 * smoothstep(0.0, 0.85, board);
	}
	if (style == 20) {
		// colonnade: round columns between the tall windows, podium, entablature
		float cx = abs(fract(x / bay) - 0.0) * bay;
		cx = min(cx, bay - cx);
		float top_band = step(fh * nfl - 1.6, y);
		if (cx < 0.55 && y > 1.2 && top_band < 0.5) {
			float r = cx / 0.55;
			wall = mix(vec3(0.95, 0.93, 0.88), wall, 0.25) * (0.7 + 0.35 * sqrt(max(0.0, 1.0 - r * r)));
			win = false;
		}
		if (y < 1.2) wall *= 0.85 - 0.1 * step(0.5, fract(y / 0.3));
		if (top_band > 0.5) wall = mix(wall, vec3(0.92, 0.9, 0.86), 0.5) * (0.92 + 0.08 * step(0.5, fract(y / 0.25)));
	}
	if (style == 21 && !win && !shop) {
		vec2 pnl = abs(fract(vec2(x / 1.5, y / 1.0)) - 0.5);
		wall *= 0.92 + 0.12 * h21(floor(vec2(x / 1.5, y / 1.0)) + seed * 5.0);
		wall *= 1.0 - 0.25 * step(0.47, max(pnl.x, pnl.y));
	}
	if (style == 23) {
		// open parking decks: slab edges, dark open storeys, columns
		float colm = step(abs(fract(x / bay) - 0.5) * bay, 0.3);
		bool opn = fy > 0.95 && fy < fh - 0.05 && floor_i < nfl && floor_i >= 0.0 && colm < 0.5;
		win = false;
		if (opn) wall = vec3(0.08, 0.08, 0.09) + 0.05 * step(0.6, fy / fh);
	}
	if (style == 25) {
		float fin = step(abs(fract(x / 2.0) - 0.5) * 2.0, 0.18);
		wall *= 0.85 + 0.25 * fin;
	}
	// plinth (and anything below the building base on a slope)
	if (y < 0.6) wall = mix(wall, vec3(0.32, 0.31, 0.3), 0.75);
	// shop sign band
	if (shop && shop_flag > 0.5 && fy > fh - 0.8 && fy < fh - 0.2 && fract(x / (bay * 3.0)) > 0.08) {
		float sg = h21(vec2(floor(x / (bay * 3.0)), seed * 11.0));
		vec3 sc = sg < 0.3 ? vec3(0.7, 0.12, 0.1) : sg < 0.55 ? vec3(0.12, 0.3, 0.6) : sg < 0.75 ? vec3(0.15, 0.45, 0.25) : vec3(0.15, 0.15, 0.17);
		wall = sc;
		emit = sc * night * 0.9 * step(0.35, sg);
	}
	vec3 col = wall;
	if (win) {
		float wxa = shop ? sx - 0.3 : fx, wxw = shop ? bay - 0.6 : ww;
		float fya = shop ? fy - 0.45 : fy - sill, fyh = shop ? fh - 1.3 : wh;
		float fr = min(min(wxa, wxw - wxa), min(fya, fyh - fya));
		float frame = 1.0 - smoothstep(0.04, 0.08, fr);
		// mullions: vertical split for old windows, transom on Stalinist ones
		if (!shop && (style == 2 || style == 10 || style == 1 || style == 12)) {
			frame = max(frame, 1.0 - smoothstep(0.02, 0.04, abs(wxa - wxw * 0.5)));
			if (style == 2 || style == 10) frame = max(frame, 1.0 - smoothstep(0.02, 0.04, abs(fya - fyh * 0.72)));
		}
		float cell = h21(vec2(bay_i * 1.7 + seed * 31.0, floor_i * 3.1));
		vec3 glass = mix(vec3(0.08, 0.1, 0.12), vec3(0.22, 0.26, 0.3), cell * 0.6);
		glass = mix(glass, vec3(0.35, 0.32, 0.28), step(0.72, cell) * 0.5);
		if (shop) {
			// lit shop interior behind the glass: back wall, shelves, a bright ceiling strip
			float k = h21(vec2(floor(x / (bay * 2.0)), seed * 3.0));
			vec3 inside = k < 0.33 ? vec3(0.62, 0.55, 0.45) : k < 0.66 ? vec3(0.7, 0.72, 0.74) : vec3(0.55, 0.45, 0.38);
			float shelf = step(0.6, fract(fy * 2.2)) * step(fy, fh * 0.55);
			inside *= 0.75 + 0.25 * shelf;
			inside = mix(inside, vec3(1.0, 0.97, 0.9), step(fh - 1.15, fy) * 0.6);
			glass = mix(inside * 0.8, vec3(0.2, 0.24, 0.28), 0.35);
		}
		vec3 fcol = style == 2 || style == 10 ? vec3(0.92, 0.9, 0.85) : (shop ? vec3(0.18, 0.18, 0.2) : vec3(0.85, 0.85, 0.82));
		if (style == 14) fcol = vec3(0.4, 0.38, 0.35);
		// window frames differ between buildings: white PVC, brown, anthracite, old painted wood
		float fv = fract(seed * 17.3);
		if (!shop && style != 3 && style != 14) {
			if (fv > 0.62 && fv < 0.82) fcol = vec3(0.36, 0.25, 0.17);
			else if (fv >= 0.82 && fv < 0.92) fcol = vec3(0.22, 0.23, 0.25);
			else if (fv >= 0.92) fcol = vec3(0.75, 0.72, 0.62);
		}
		col = mix(glass, fcol, frame);
		rough = mix(0.08, 0.6, frame);
		metal = (1.0 - frame) * 0.15;
		float lit = step(0.7, h21(vec2(bay_i + seed * 13.0, floor_i + seed * 7.0)));
		if (shop) lit = 1.0;
		emit = vec3(1.0, 0.78, 0.5) * lit * night * (shop ? 2.4 : 1.8) * (1.0 - frame);
	}
	if (style == 3 && !win) col = mix(col, vec3(0.25, 0.27, 0.3), 0.3);
	apply_wet(col, rough, win ? 0.0 : 0.5);
	ALBEDO = col;
	ROUGHNESS = rough;
	METALLIC = metal;
	EMISSION = emit;
	}
}
