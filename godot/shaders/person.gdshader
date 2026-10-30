// Pedestrians: limbs swing with the walk phase (INSTANCE_CUSTOM.a), UV.y is a material code; the shirt takes INSTANCE_CUSTOM.rgb.
shader_type spatial;
render_mode cull_back;

varying vec3 tint;

void vertex() {
	float part = UV.x;
	float ph = INSTANCE_CUSTOM.a;
	int code = int(UV.y + 0.5);
	// a per-person skin / trouser shade from the shirt colour
	float k = fract(dot(INSTANCE_CUSTOM.rgb, vec3(12.9898, 78.233, 37.719)));
	vec3 skin = mix(vec3(0.85, 0.66, 0.53), vec3(0.55, 0.38, 0.27), k * k);
	vec3 pants = k < 0.4 ? vec3(0.13, 0.15, 0.24) : k < 0.7 ? vec3(0.2, 0.2, 0.2) : vec3(0.38, 0.33, 0.26);
	tint = code == 1 ? INSTANCE_CUSTOM.rgb : code == 2 ? pants : code == 3 ? vec3(0.06) : code == 4 ? vec3(0.1, 0.07, 0.05) : skin;
	if (part > 0.5) {
		float sgn = (part < 1.5 || part > 3.5) ? 1.0 : -1.0;
		bool arm = part > 2.5;
		float pivot = arm ? 1.42 : 0.92;
		float a = sin(ph) * sgn * (arm ? 0.45 : 0.5);
		vec2 q = VERTEX.xy - vec2(0.0, pivot);
		float c = cos(a), s = sin(a);
		VERTEX.xy = vec2(c * q.x - s * q.y, s * q.x + c * q.y) + vec2(0.0, pivot);
		NORMAL.xy = vec2(c * NORMAL.x - s * NORMAL.y, s * NORMAL.x + c * NORMAL.y);
	}
	// a little bob with each step
	VERTEX.y += abs(sin(ph)) * 0.03;
}

void fragment() {
	ALBEDO = tint;
	ROUGHNESS = 0.8;
	SPECULAR = 0.25;
}
