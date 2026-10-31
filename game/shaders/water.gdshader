shader_type spatial;
render_mode cull_back;
#include "res://shaders/common.gdshaderinc"

varying vec3 wpos;

void vertex() {
	wpos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
	vec2 w = wpos.xz;
	float t = TIME * 0.4;
	float e = 0.15;
	float a = fbm(w * 0.35 + vec2(t, t * 0.6));
	float bx = fbm((w + vec2(e, 0.0)) * 0.35 + vec2(t, t * 0.6));
	float by = fbm((w + vec2(0.0, e)) * 0.35 + vec2(t, t * 0.6));
	vec3 nw = normalize(vec3((a - bx) * 2.0, 1.0, (a - by) * 2.0));
	NORMAL = normalize((VIEW_MATRIX * vec4(nw, 0.0)).xyz);
	ALBEDO = vec3(0.05, 0.09, 0.1);
	ROUGHNESS = 0.04;
	SPECULAR = 0.9;
}
