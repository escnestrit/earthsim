// Grass clumps (MultiMesh): wind sway, colour from the instance, shrinks away with distance.
shader_type spatial;
render_mode cull_disabled;
#include "res://shaders/common.gdshaderinc"

varying float hgt;

void vertex() {
	hgt = UV.y;
	vec3 wp0 = (MODEL_MATRIX * vec4(0.0, 0.0, 0.0, 1.0)).xyz;
	float d = length(wp0 - CAMERA_POSITION_WORLD);
	float k = 1.0 - smoothstep(45.0, 75.0, d);
	VERTEX *= k;
	float w = sin(TIME * 1.7 + wp0.x * 0.35 + wp0.z * 0.27) * 0.5 + sin(TIME * 2.9 + wp0.x * 0.9) * 0.25;
	VERTEX.x += w * 0.06 * hgt * hgt;
	VERTEX.z += w * 0.04 * hgt * hgt;
	NORMAL = vec3(0.0, 1.0, 0.0);
}

void fragment() {
	vec3 base = vec3(0.16, 0.26, 0.07);
	vec3 tip = vec3(0.36, 0.48, 0.16);
	vec3 c = mix(base, tip, hgt) * COLOR.rgb;
	int bi = int(biome + 0.5);
	if (bi == 2) c = mix(c, vec3(0.45, 0.42, 0.22), 0.45);
	if (bi == 5) c = mix(c, vec3(0.4, 0.42, 0.33), 0.4);
	float r = 0.9;
	apply_wet(c, r, 0.2);
	ALBEDO = c;
	ROUGHNESS = r;
	SPECULAR = 0.2;
	BACKLIGHT = vec3(0.25, 0.3, 0.1);
}
