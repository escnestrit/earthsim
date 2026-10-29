// Ground: eight material layers blended by the chunk's splat map, with height-based blending and
// anti-tiling (two scales mixed by a noise), normal and roughness/occlusion maps.
shader_type spatial;
render_mode blend_mix, depth_draw_opaque, cull_back, diffuse_burley, specular_schlick_ggx;

uniform sampler2D splat0 : filter_linear, repeat_disable;
uniform sampler2D splat1 : filter_linear, repeat_disable;
uniform sampler2DArray layer_albedo : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2DArray layer_normal : hint_normal, filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2DArray layer_orm : filter_linear_mipmap_anisotropic, repeat_enable;
// metres covered by one texture repeat, per layer: grass, forest, sand, asphalt, pavers, farm, dirt, pitch
const float SCALE[8] = float[](3.0, 4.0, 4.0, 4.0, 3.0, 6.0, 3.0, 8.0);

varying vec3 wpos;

float hash12(vec2 p) {
	vec3 p3 = fract(vec3(p.xyx) * 0.1031);
	p3 += dot(p3, p3.yzx + 33.33);
	return fract((p3.x + p3.y) * p3.z);
}
float vnoise(vec2 p) {
	vec2 i = floor(p), f = fract(p);
	f = f * f * (3.0 - 2.0 * f);
	return mix(mix(hash12(i), hash12(i + vec2(1, 0)), f.x), mix(hash12(i + vec2(0, 1)), hash12(i + vec2(1, 1)), f.x), f.y);
}

void vertex() {
	wpos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
	vec4 a = texture(splat0, UV);
	vec4 b = texture(splat1, UV);
	float w[8] = float[](a.r, a.g, a.b, a.a, b.r, b.g, b.b, b.a);
	// break straight splat edges with noise
	float n1 = vnoise(wpos.xz * 0.35), n2 = vnoise(wpos.xz * 0.05);
	float mixk = smoothstep(0.3, 0.7, vnoise(wpos.xz * 0.02 + 17.0));
	vec3 alb = vec3(0.0);
	vec3 nrm = vec3(0.0);
	vec3 orm = vec3(0.0);
	float tot = 0.0;
	for (int i = 0; i < 8; i++) {
		if (w[i] < 0.01) continue;
		vec2 uv = wpos.xz / SCALE[i];
		vec2 uv2 = uv * 0.37 + vec2(0.31, 0.77);
		vec4 c1 = texture(layer_albedo, vec3(uv, float(i)));
		vec4 c2 = texture(layer_albedo, vec3(uv2, float(i)));
		vec4 o1 = texture(layer_orm, vec3(uv, float(i)));
		vec3 nn = texture(layer_normal, vec3(uv, float(i))).xyz;
		// height-aware weight: the texture's own relief (occlusion channel) sharpens transitions
		float hw = w[i] * (0.5 + o1.r) * (0.75 + 0.5 * n1);
		hw = pow(hw, 1.6);
		alb += mix(c1.rgb, c2.rgb, mixk * 0.6) * hw;
		nrm += nn * hw;
		orm += o1.rgb * hw;
		tot += hw;
	}
	tot = max(tot, 1e-4);
	alb /= tot;
	nrm /= tot;
	orm /= tot;
	// large-scale colour variation
	alb *= 0.88 + 0.24 * n2;
	ALBEDO = alb;
	NORMAL_MAP = nrm;
	NORMAL_MAP_DEPTH = 1.0;
	AO = orm.r;
	AO_LIGHT_AFFECT = 0.5;
	ROUGHNESS = orm.g;
	METALLIC = 0.0;
	SPECULAR = 0.4;
}
