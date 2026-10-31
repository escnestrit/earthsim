// Traffic light lenses. INSTANCE_CUSTOM.x = phase offset (s). 40 s cycle:
// green 0-16, amber 16-19, red 19-37, red+amber 37-40.
shader_type spatial;
render_mode cull_back;
global uniform float signal_time;  // driven by the traffic system so lights and cars agree

varying float lamp;
varying float phase;

void vertex() {
	lamp = UV.y;
	phase = INSTANCE_CUSTOM.x;
}

void fragment() {
	float t = mod(signal_time + phase, 40.0);
	int k = int(lamp + 0.5);
	bool on = false;
	if (k == 0) on = t >= 19.0;
	else if (k == 1) on = (t >= 16.0 && t < 19.0) || t >= 37.0;
	else on = t < 16.0;
	vec3 c = k == 0 ? vec3(1.0, 0.1, 0.05) : k == 1 ? vec3(1.0, 0.6, 0.0) : vec3(0.1, 1.0, 0.4);
	ALBEDO = on ? c : c * 0.08;
	EMISSION = on ? c * 3.0 : vec3(0.0);
	ROUGHNESS = 0.3;
}
