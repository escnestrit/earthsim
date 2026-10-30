// Grass tufts: crossed alpha-tested cards that sway in the wind and fade out with distance.
shader_type spatial;
render_mode cull_disabled, diffuse_burley, specular_schlick_ggx;

uniform sampler2D blades : source_color, filter_linear_mipmap_anisotropic, repeat_disable;
uniform float fade_start = 35.0;
uniform float fade_end = 55.0;
uniform vec3 tint : source_color = vec3(1.0);

global uniform float snow_cover;
global uniform float wetness;
varying float h;
varying vec3 wpos;

void vertex() {
	h = 1.0 - UV.y;
	vec3 world = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	float t = TIME * 1.6;
	float gust = sin(world.x * 0.07 + t * 0.6) * 0.5 + 0.5;
	vec2 sway = vec2(sin(t + world.x * 0.9 + world.z * 0.4), cos(t * 1.2 + world.z * 0.8)) * (0.05 + 0.08 * gust) * h * h;
	VERTEX.xz += sway;
	// under snow only the tips show
	VERTEX.y *= 1.0 - 0.85 * snow_cover;
	// light the tuft like the ground it grows from
	NORMAL = normalize(mix(NORMAL, vec3(0.0, 1.0, 0.0), 0.75));
	wpos = world;
}

void fragment() {
	vec4 c = texture(blades, UV);
	float d = length(wpos - CAMERA_POSITION_WORLD);
	float fade = 1.0 - smoothstep(fade_start, fade_end, d);
	// dithered fade so the edge of the grass doesn't pop
	float n = fract(sin(dot(FRAGCOORD.xy, vec2(12.9898, 78.233))) * 43758.5453);
	if (c.a < 0.45 || n > fade) {
		discard;
	}
	ALBEDO = c.rgb * COLOR.rgb * tint * 1.15 * (1.0 - 0.2 * wetness);
	ALBEDO = mix(ALBEDO, vec3(0.9, 0.92, 0.95), snow_cover * 0.6 * (1.0 - h));
	ROUGHNESS = mix(0.8, 0.4, wetness);
	SPECULAR = 0.2;
	AO = mix(0.45, 1.0, h);
	BACKLIGHT = vec3(0.25, 0.35, 0.1);
}
