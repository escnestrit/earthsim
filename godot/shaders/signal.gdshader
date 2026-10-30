// Traffic light lamps. All lights share one 60 s cycle: the two crossing directions (heads facing along x or
// along z in the world) alternate, with an all-red gap. Vertex colour: r 1 red / 0.5 amber / 0 green, g 1 =
// pedestrian lamp (green while the crossing traffic has green).
shader_type spatial;
render_mode cull_disabled;

varying float lit;
varying vec3 lamp;

void vertex() {
	float code = COLOR.r;
	bool ped = COLOR.g > 0.5;
	vec3 fx = MODEL_MATRIX[0].xyz;
	float grp = abs(fx.x) >= abs(fx.z) ? 0.0 : 1.0;
	float t = mod(TIME + grp * 30.0, 60.0);          // this approach
	float tc = mod(TIME + (1.0 - grp) * 30.0, 60.0);  // the crossing one
	lit = 0.0;
	if (!ped) {
		if (code > 0.75) { lit = t >= 27.0 ? 1.0 : 0.0; lamp = vec3(1.0, 0.08, 0.04); }
		else if (code > 0.25) { lit = (t >= 24.0 && t < 27.0) ? 1.0 : 0.0; lamp = vec3(1.0, 0.55, 0.0); }
		else { lit = t < 24.0 ? 1.0 : 0.0; lamp = vec3(0.1, 1.0, 0.25); }
	} else {
		bool walk = tc < 21.0 || (tc < 24.0 && fract(TIME * 2.0) < 0.5);
		if (code > 0.5) { lit = walk ? 0.0 : 1.0; lamp = vec3(1.0, 0.08, 0.04); }
		else { lit = walk ? 1.0 : 0.0; lamp = vec3(0.1, 1.0, 0.25); }
	}
}

void fragment() {
	ALBEDO = lamp * (0.15 + 0.1 * (1.0 - lit));
	EMISSION = lamp * lit * 4.0;
	ROUGHNESS = 0.2;
}
