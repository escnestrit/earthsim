// Tree crowns: instance tint * base green, world-space noise for clumps, darker inside/underneath.
shader_type spatial;
render_mode cull_back;
#include "res://shaders/common.gdshaderinc"

uniform vec3 base_color = vec3(0.22, 0.32, 0.12);
varying vec3 wpos;
varying vec3 lpos;

void vertex() {
	wpos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	lpos = VERTEX;
}

void fragment() {
	float n = fbm(wpos.xz * 1.7 + wpos.y * 1.3);
	float clump = smoothstep(0.35, 0.7, vnoise(wpos.xz * 0.8 + wpos.y * 0.6));
	vec3 col = base_color * COLOR.rgb * (0.65 + 0.6 * n) * (0.8 + 0.3 * clump);
	// lower part of the crown is in self-shadow
	col *= mix(0.55, 1.0, smoothstep(-0.6, 0.6, NORMAL.y * 0.5 + (VIEW_MATRIX * vec4(0.0, 1.0, 0.0, 0.0)).y * 0.0 + 0.3));
	ALBEDO = col;
	ROUGHNESS = 0.9;
	SPECULAR = 0.2;
	BACKLIGHT = vec3(0.12, 0.16, 0.04);
}
