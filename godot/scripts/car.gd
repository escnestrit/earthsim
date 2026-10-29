# The player's car: a ray-cast vehicle with an engine curve, automatic gearbox, speed-sensitive steering,
# ABS-like braking and handbrake. Stage 1 of the vehicle; the soft-body car replaces it later.
extends VehicleBody3D
class_name PlayerCar

const GEARS := [3.6, 2.1, 1.4, 1.05, 0.85, 0.7]
const FINAL := 3.9
const WHEEL_R := 0.33
const IDLE := 850.0
const REDLINE := 6500.0

var gear := 1
var rpm := IDLE
var throttle := 0.0
var brake_in := 0.0
var steer_in := 0.0
var handbrake := false
var wheels: Array[VehicleWheel3D] = []
var front: Array[VehicleWheel3D] = []
var visual: Node3D
var forward_sign := 1.0   # which body axis the vehicle drives along (+z or -z), measured at start

func setup(body_mesh: Mesh, wheel_mesh: Mesh) -> void:
	mass = 1450.0
	center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
	center_of_mass = Vector3(0, 0.45, 0.1)
	can_sleep = false
	continuous_cd = true
	# body collision: lower body and cabin; the car's forward is +z
	var lower := CollisionShape3D.new()
	var b1 := BoxShape3D.new()
	b1.size = Vector3(1.76, 0.55, 4.35)
	lower.shape = b1
	lower.position = Vector3(0, 0.62, 0)
	add_child(lower)
	var cab := CollisionShape3D.new()
	var b2 := BoxShape3D.new()
	b2.size = Vector3(1.5, 0.48, 2.3)
	cab.shape = b2
	cab.position = Vector3(0, 1.15, -0.25)
	add_child(cab)
	visual = MeshInstance3D.new()
	(visual as MeshInstance3D).mesh = body_mesh
	# the model faces +x; turn it to +z
	visual.rotation = Vector3(0, -PI / 2, 0)
	add_child(visual)
	for fx in [1.35, -1.35]:
		for side in [0.78, -0.78]:
			var w := VehicleWheel3D.new()
			w.position = Vector3(side, 0.42, fx)
			w.wheel_radius = WHEEL_R
			w.wheel_rest_length = 0.16
			w.suspension_travel = 0.22
			w.suspension_stiffness = 42.0
			w.suspension_max_force = 9000.0
			w.damping_compression = 0.45
			w.damping_relaxation = 0.6
			w.wheel_friction_slip = 2.6
			w.wheel_roll_influence = 0.12
			w.use_as_steering = fx > 0
			w.use_as_traction = true
			var wm := MeshInstance3D.new()
			wm.mesh = wheel_mesh
			wm.rotation = Vector3(0, 0, PI / 2)
			w.add_child(wm)
			add_child(w)
			wheels.append(w)
			if fx > 0:
				front.append(w)

func speed_kmh() -> float:
	return linear_velocity.length() * 3.6

func forward_speed() -> float:
	return linear_velocity.dot(global_transform.basis.z) * forward_sign

func _physics_process(dt: float) -> void:
	var up := Input.is_action_pressed("ui_up") or Input.is_key_pressed(KEY_W)
	var down := Input.is_action_pressed("ui_down") or Input.is_key_pressed(KEY_S)
	var left := Input.is_action_pressed("ui_left") or Input.is_key_pressed(KEY_A)
	var right := Input.is_action_pressed("ui_right") or Input.is_key_pressed(KEY_D)
	handbrake = Input.is_key_pressed(KEY_SPACE)
	var v := forward_speed()
	# throttle / brake / reverse like an automatic
	var want_t := 0.0
	var want_b := 0.0
	if up:
		if v < -0.5:
			want_b = 1.0
		else:
			want_t = 1.0
			if gear < 1:
				gear = 1
	if down:
		if v > 0.5:
			want_b = 1.0
		else:
			want_t = 1.0
			gear = -1
	throttle = move_toward(throttle, want_t, dt * 4.0)
	brake_in = move_toward(brake_in, want_b, dt * 6.0)
	# steering: slower and smaller at speed
	var target := (1.0 if left else 0.0) - (1.0 if right else 0.0)
	var rate := 2.2 if abs(target) > 0.0 else 3.2
	steer_in = move_toward(steer_in, target, dt * rate)
	var max_angle := lerpf(0.6, 0.12, clampf(abs(v) / 40.0, 0.0, 1.0))
	steering = steer_in * max_angle * forward_sign
	# gearbox
	var wheel_rpm: float = abs(v) / WHEEL_R * 60.0 / TAU
	if gear > 0:
		rpm = max(IDLE, wheel_rpm * GEARS[gear - 1] * FINAL)
		if rpm > 5800.0 and gear < GEARS.size():
			gear += 1
		elif gear > 1 and rpm < 2200.0:
			var lower_rpm: float = wheel_rpm * GEARS[gear - 2] * FINAL
			if lower_rpm < 5000.0:
				gear -= 1
	else:
		rpm = max(IDLE, wheel_rpm * 3.3 * FINAL)
	# torque curve (Nm) of a 2.0 l petrol engine
	var x := clampf(rpm / REDLINE, 0.0, 1.0)
	var torque := 140.0 + 70.0 * sin(x * PI * 0.9)
	if rpm >= REDLINE:
		torque = 0.0
	var ratio: float = (GEARS[gear - 1] if gear > 0 else 3.3) * FINAL
	var force := torque * ratio / WHEEL_R * 0.88 * throttle
	var per_wheel := force * (1.0 if gear > 0 else -1.0) * forward_sign / 4.0
	var b := brake_in * 9.0
	for w in wheels:
		var rear := not w.use_as_steering
		w.brake = b + (30.0 if handbrake and rear else 0.0)
		w.wheel_friction_slip = 1.3 if handbrake and rear else 2.6
		w.engine_force = per_wheel
	# rolling resistance and drag
	var vel := linear_velocity
	apply_central_force(-vel * vel.length() * 0.42 - vel * 12.0)

func reset_upright() -> void:
	var p := global_position
	var yaw := global_rotation.y
	global_transform = Transform3D(Basis(Vector3.UP, yaw), p + Vector3(0, 1.5, 0))
	linear_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
