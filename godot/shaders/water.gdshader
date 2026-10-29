// Lakes and rivers: two scrolling wave normal maps, depth-tinted, refracting the bed, Fresnel reflections.
shader_type spatial;
render_mode blend_mix, depth_draw_always, cull_back, diffuse_burley, specular_schlick_ggx;

uniform sampler2D wave1 : hint_normal, filter_linear_mipmap, repeat_enable;
uniform sampler2D wave2 : hint_normal, filter_linear_mipmap, repeat_enable;
uniform sampler2D screen_tex : hint_screen_texture, filter_linear_mipmap;
uniform sampler2D depth_tex : hint_depth_texture, filter_nearest;
uniform vec3 shallow : source_color = vec3(0.16, 0.28, 0.26);
uniform vec3 deep : source_color = vec3(0.03, 0.08, 0.1);

varying vec3 wpos;

void vertex() {
	wpos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
	float t = TIME;
	vec2 uv = wpos.xz;
	vec3 n1 = texture(wave1, uv / 40.0 + vec2(t * 0.012, t * 0.008)).xyz;
	vec3 n2 = texture(wave2, uv / 13.0 - vec2(t * 0.02, -t * 0.015)).xyz;
	vec3 n = normalize(mix(n1, n2, 0.5) * 2.0 - 1.0);
	NORMAL_MAP = n * 0.5 + 0.5;
	NORMAL_MAP_DEPTH = 0.6;
	// water depth from the scene depth behind the surface
	float d = texture(depth_tex, SCREEN_UV).r;
	vec4 vp = INV_PROJECTION_MATRIX * vec4(SCREEN_UV * 2.0 - 1.0, d, 1.0);
	vp.xyz /= vp.w;
	float depth = clamp(VERTEX.z - vp.z, 0.0, 50.0);
	float k = 1.0 - exp(-depth * 0.35);
	vec2 off = n.xy * 0.02 * clamp(depth, 0.0, 1.0);
	vec3 bed = texture(screen_tex, SCREEN_UV + off).rgb;
	vec3 col = mix(bed * shallow * 2.5, deep, k);
	ALBEDO = vec3(0.0);
	EMISSION = col * 0.6;
	ROUGHNESS = 0.03;
	METALLIC = 0.0;
	SPECULAR = 0.6;
	ALPHA = 1.0;
}
