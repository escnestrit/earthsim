## Car physics: rigid body chassis (Jolt) + 4 ray-cast wheels with spring/damper suspension,
## anti-roll bars, a Pacejka-style tyre model with a friction ellipse, wheel spin dynamics
## (sub-stepped), engine torque curve, clutch, automatic/manual gearbox, open differential,
## brakes with ABS, traction control, aerodynamic drag and rolling resistance.
## Godot frame: forward = -Z, right = +X, up = +Y.
extends RigidBody3D

signal reset_requested
signal impact(local_pos: Vector3, local_dir: Vector3, impulse: float)

# ---- chassis
@export var car_mass := 1380.0
@export var com_height := 0.12           # centre of mass above the body origin (origin = wheel-centre height)
@export var wheelbase := 2.70
@export var track := 1.56
@export var wheel_radius := 0.325
@export var drag_area := 0.66             # Cd * A, m^2
# ---- suspension (per wheel)
@export var spring_k := Vector2(30000, 27000)   # front, rear N/m
@export var damp_bump := Vector2(2600, 2400)
@export var damp_rebound := Vector2(4200, 3800)
@export var arb_k := Vector2(14000, 7000)       # anti-roll bars
@export var rest_len := 0.42              # spring length at zero load (mount -> wheel centre)
@export var max_len := 0.50
@export var mount_h := 0.28               # mount point above the body origin
# ---- tyres
@export var mu := 1.05
@export var bx := 13.0
@export var cx := 1.6
@export var by := 11.0
@export var cy := 1.35
# ---- drivetrain
@export var rwd := true
@export var gears := [3.55, 2.10, 1.42, 1.06, 0.86, 0.71]
@export var reverse_ratio := 3.30
@export var final_drive := 3.91
@export var idle_rpm := 850.0
@export var redline := 6800.0
@export var brake_torque := Vector2(2600, 1500)  # max per wheel, front/rear
@export var handbrake_torque := 3200.0
@export var max_steer := deg_to_rad(36.0)
@export var wheel_override: Array = []   # explicit wheel centres (car local), FL, FR, RL, RR
@export var torque_scale := 1.0

# torque curve (rpm, Nm)
const TORQUE := [[0, 120], [1000, 150], [2000, 195], [3000, 232], [4000, 258], [5000, 255], [6000, 235], [6800, 205], [7500, 120]]
const SUBSTEPS := 8

var abs_on := true
var tcs_on := true
var auto_gear := true
var gear := 1                  # -1 reverse, 0 neutral, 1..n
var rpm := 900.0
var shift_timer := 0.0
var clutch := 1.0              # 1 engaged
var steer := 0.0               # current road-wheel steering angle (rad), positive = right
var input_throttle := 0.0
var input_brake := 0.0
var input_steer := 0.0         # -1 left .. 1 right
var input_handbrake := 0.0
var steer_direct := false      # wheel/gamepad: no speed-sensitive filtering
var speed := 0.0               # forward speed m/s
var surface_name := ""
var ready_to_drive := false
var wetness := 0.0             # 0 dry .. 1 wet roads (set by the game)
var steer_bias := 0.0          # misalignment after heavy front impacts (rad)
var _impact_cool := 0.0

class Wheel:
	var pos := Vector3.ZERO     # local wheel centre at rest (x, origin height, z)
	var front := true
	var left := true
	var driven := false
	var len := 0.42
	var prev_len := 0.42
	var contact := false
	var hit_pos := Vector3.ZERO
	var hit_n := Vector3.UP
	var load := 0.0
	var omega := 0.0
	var angle := 0.0           # spin angle for visuals
	var steer := 0.0
	var surf := 0
	var slip_ratio := 0.0
	var slip_angle := 0.0
	var fx := 0.0
	var fy := 0.0
	var node: Node3D

var wheels: Array[Wheel] = []
var _ray := PhysicsRayQueryParameters3D.new()

func _ready() -> void:
	mass = car_mass
	center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
	center_of_mass = Vector3(0, com_height, 0.05)
	inertia = Vector3(car_mass * 1.75, car_mass * 1.9, car_mass * 0.45)  # pitch (X), yaw (Y), roll (Z)
	can_sleep = false
	continuous_cd = true
	linear_damp_mode = RigidBody3D.DAMP_MODE_REPLACE
	angular_damp_mode = RigidBody3D.DAMP_MODE_REPLACE
	linear_damp = 0.0
	angular_damp = 0.05
	var hw := track * 0.5
	var hl := wheelbase * 0.5
	for i in 4:
		var w := Wheel.new()
		w.front = i < 2
		w.left = i % 2 == 0
		w.pos = Vector3(-hw if w.left else hw, 0.0, -hl if w.front else hl)
		if wheel_override.size() == 4:
			w.pos = wheel_override[i]
		w.driven = (not w.front) if rwd else w.front
		w.len = rest_len
		w.prev_len = rest_len
		wheels.append(w)
	_ray.exclude = [get_rid()]
	_ray.collide_with_areas = false
	contact_monitor = true
	max_contacts_reported = 8

func attach_wheel_nodes(nodes: Array) -> void:
	for i in 4:
		wheels[i].node = nodes[i]

func torque_at(r: float) -> float:
	for i in range(1, TORQUE.size()):
		if r <= TORQUE[i][0]:
			var a = TORQUE[i - 1]
			var b = TORQUE[i]
			return lerpf(a[1], b[1], (r - a[0]) / float(b[0] - a[0])) * torque_scale
	return 0.0

func ratio() -> float:
	if gear == 0:
		return 0.0
	return (gears[gear - 1] if gear > 0 else -reverse_ratio) * final_drive

func _magic(slip: float, b: float, c: float) -> float:
	return sin(c * atan(b * slip))

func _physics_process(dt: float) -> void:
	if not ready_to_drive:
		return
	_update_gearbox(dt)
	# steering
	var target := input_steer * max_steer
	if not steer_direct:
		target *= 1.0 / (1.0 + absf(speed) / 16.0)
		var rate := 2.2 if absf(target) > absf(steer) else 3.5
		steer = move_toward(steer, target, rate * dt)
	else:
		steer = target
	steer += steer_bias

func _integrate_forces(state: PhysicsDirectBodyState3D) -> void:
	if not ready_to_drive:
		return
	var dt := state.step
	var xf := state.transform
	var up := xf.basis.y
	var space := state.get_space_state()
	var lv := state.linear_velocity
	var av := state.angular_velocity
	var fwd := -xf.basis.z
	speed = lv.dot(fwd)
	# Ackermann steering angles
	var turn_r := wheelbase / tan(absf(steer)) if absf(steer) > 0.001 else 1e9
	var inner := atan(wheelbase / maxf(0.1, turn_r - track * 0.5))
	var outer := atan(wheelbase / (turn_r + track * 0.5))
	for w: Wheel in wheels:
		if w.front:
			var is_inner: bool = (steer > 0.0) != w.left  # steering right: the right wheel is inner
			w.steer = signf(steer) * (inner if is_inner else outer)
		else:
			w.steer = 0.0
	# ---- suspension ray casts
	var lens := []
	for w: Wheel in wheels:
		var mount := xf * (w.pos + Vector3(0, mount_h, 0))
		_ray.from = mount
		_ray.to = mount - up * (max_len + wheel_radius + mount_h * 0.0)
		var hit := space.intersect_ray(_ray)
		w.prev_len = w.len
		if hit.is_empty():
			w.contact = false
			w.len = move_toward(w.len, max_len, 2.0 * dt)
		else:
			w.contact = true
			w.hit_pos = hit["position"]
			w.hit_n = hit["normal"]
			w.len = clampf(mount.distance_to(w.hit_pos) - wheel_radius, 0.0, max_len)
			var col = hit["collider"]
			w.surf = col.get_meta("surface", 1) if col != null else 1
		lens.append(w.len)
	# ---- suspension forces
	for i in 4:
		var w: Wheel = wheels[i]
		if not w.contact:
			w.load = 0.0
			continue
		var ax := 0 if w.front else 1
		var comp := rest_len - w.len
		var vel := (w.prev_len - w.len) / dt  # compression speed
		var k: float = spring_k[ax]
		var c: float = damp_bump[ax] if vel > 0.0 else damp_rebound[ax]
		var f := k * comp + c * vel
		# anti-roll bar
		var other: Wheel = wheels[i ^ 1]
		f += arb_k[ax] * ((other.len if other.contact else max_len) - w.len)
		# bump stop
		if w.len < 0.06:
			f += (0.06 - w.len) * 300000.0
		f = maxf(f, 0.0)
		w.load = f
		state.apply_force(up * f, w.hit_pos - xf.origin)
	# ---- tyre forces with sub-stepped wheel spin
	var r := ratio()
	var wheel_inertia := 1.3
	var engine_inertia := 0.22
	var throttle := input_throttle
	var brake := input_brake
	if gear < 0:
		# reverse: the brake pedal drives, throttle brakes (handled in input mapping)
		pass
	if shift_timer > 0.0:
		throttle = 0.0
	var driven_count := 2
	# traction control: reduce throttle when a driven wheel slips a lot
	if tcs_on:
		for w: Wheel in wheels:
			if w.driven and w.contact and w.slip_ratio > 0.18 and absf(speed) > 1.0:
				throttle *= clampf(1.0 - (w.slip_ratio - 0.18) * 4.0, 0.15, 1.0)
	var mu_s: float = mu * (1.0 - 0.32 * wetness)
	var sub := dt / SUBSTEPS
	for w: Wheel in wheels:
		w.fx = 0.0
		w.fy = 0.0
	var engine_torque := 0.0
	for s in SUBSTEPS:
		# engine speed from the driven wheels when the clutch is engaged
		var wheel_omega := 0.0
		for w: Wheel in wheels:
			if w.driven:
				wheel_omega += w.omega * 0.5
		var rpm_wheels := absf(wheel_omega * r) * 60.0 / TAU
		var clutch_slip := false
		if gear == 0 or r == 0.0:
			clutch_slip = true
		elif rpm_wheels < idle_rpm + 300.0 and throttle > 0.02:
			clutch_slip = true  # launching
		elif rpm_wheels < idle_rpm and throttle <= 0.02:
			clutch_slip = true  # rolling to a stop: declutch
		var et := 0.0
		if clutch_slip:
			var free_rpm := idle_rpm + throttle * 2600.0
			rpm = lerpf(rpm, free_rpm, 0.02)
			et = throttle * torque_at(rpm) * (0.0 if gear == 0 else 1.0) * clampf((rpm - rpm_wheels) / 600.0, 0.0, 1.0)
		else:
			rpm = maxf(rpm_wheels, idle_rpm)
			var drag := (0.0 if throttle > 0.05 else 1.0) * (18.0 + rpm * 0.009)
			et = throttle * torque_at(rpm) - drag
			if rpm > redline:
				et = minf(et, -20.0)
		engine_torque = et
		for w: Wheel in wheels:
			var ax := 0 if w.front else 1
			if not w.contact:
				var t_free := 0.0
				if w.driven:
					t_free = et * r * 0.92 / driven_count
				w.omega += t_free / (wheel_inertia + (engine_inertia * r * r * 0.5 if w.driven and not clutch_slip else 0.0)) * sub
				w.omega *= 0.999
				continue
			# contact frame
			var n := w.hit_n
			var head := xf.basis * Vector3(sin(w.steer), 0, -cos(w.steer))
			var f_dir := (head - n * head.dot(n)).normalized()
			var s_dir := f_dir.cross(n).normalized()  # points to the right
			var rel := w.hit_pos - xf.origin
			var v := lv + av.cross(rel)
			var v_long := v.dot(f_dir)
			var v_lat := v.dot(s_dir)
			var ref := maxf(absf(v_long), 2.5)
			var kappa := (w.omega * wheel_radius - v_long) / ref
			var alpha := atan2(v_lat, maxf(absf(v_long), 1.0))
			var surf_mu := mu_s * (1.0 if w.surf != 1 else 0.72)
			var fz := w.load
			var dmax := surf_mu * fz
			var fx0 := dmax * _magic(kappa, bx, cx)
			var fy0 := -dmax * _magic(alpha, by, cy)
			# friction ellipse
			var e := sqrt((fx0 * fx0 + fy0 * fy0)) / maxf(dmax, 1.0)
			if e > 1.0:
				fx0 /= e
				fy0 /= e
			# low speed: damp lateral creep instead of the ill-defined slip angle
			var low := clampf(1.0 - absf(v_long) / 3.0, 0.0, 1.0)
			fy0 = lerpf(fy0, clampf(-v_lat * fz * 0.6, -dmax, dmax), low)
			# wheel torques
			var t_drive := 0.0
			if w.driven:
				t_drive = et * r * 0.92 / driven_count
			var bt: float = brake * brake_torque[ax]
			if not w.front:
				bt += input_handbrake * handbrake_torque
			if abs_on and brake > 0.0 and kappa < -0.12 and absf(v_long) > 2.0:
				bt *= 0.25
			var roll := fz * 0.012 * wheel_radius
			var inert := wheel_inertia + (engine_inertia * r * r * 0.5 if w.driven and not clutch_slip else 0.0)
			var t_net := t_drive - fx0 * wheel_radius
			var om := w.omega + t_net / inert * sub
			# brake + rolling resistance oppose rotation and never reverse it
			var t_stop := (bt + roll) / inert * sub
			if absf(om) <= t_stop:
				om = 0.0
			else:
				om -= signf(om) * t_stop
			w.omega = om
			w.slip_ratio = kappa
			w.slip_angle = alpha
			w.fx += fx0 / SUBSTEPS
			w.fy += fy0 / SUBSTEPS
	# apply averaged tyre forces
	for w: Wheel in wheels:
		if not w.contact:
			continue
		var n := w.hit_n
		var head := xf.basis * Vector3(sin(w.steer), 0, -cos(w.steer))
		var f_dir := (head - n * head.dot(n)).normalized()
		var s_dir := f_dir.cross(n).normalized()
		var force := f_dir * w.fx + s_dir * w.fy
		# hold still on slopes when braking at a standstill
		if (input_brake > 0.3 or input_handbrake > 0.3) and absf(speed) < 0.4:
			var v := lv + av.cross(w.hit_pos - xf.origin)
			force -= (v - n * v.dot(n)) * w.load * 0.25
		state.apply_force(force, w.hit_pos - xf.origin)
	# aerodynamic drag
	state.apply_central_force(-lv * lv.length() * 0.5 * 1.2 * drag_area)
	# body impacts -> deformation (reported to the game, which dents the mesh)
	_impact_cool -= dt
	if _impact_cool <= 0.0:
		var best := 0.0
		var bp := Vector3.ZERO
		var bn := Vector3.ZERO
		for i in state.get_contact_count():
			var imp := state.get_contact_impulse(i).length()
			var nrm := state.get_contact_local_normal(i)
			# ignore the floor: contacts whose normal points up are kerb scrapes / landings
			if nrm.dot(up) > 0.7 and imp < 9000.0:
				continue
			if imp > best:
				best = imp
				bp = state.get_contact_local_position(i)
				bn = nrm
		if best > 1800.0:
			var inv := xf.affine_inverse()
			var lp := inv * bp
			var ld := (xf.basis.inverse() * bn).normalized()
			impact.emit(lp, ld, best)
			if lp.z < -1.2 and best > 7000.0:
				steer_bias += clampf(best / 400000.0, 0.0, 0.03) * (1.0 if lp.x > 0.0 else -1.0)
			_impact_cool = 0.12
	for w: Wheel in wheels:
		w.angle = fmod(w.angle + w.omega * dt, TAU)

func _update_gearbox(dt: float) -> void:
	if shift_timer > 0.0:
		shift_timer -= dt
	if not auto_gear or shift_timer > 0.0:
		return
	if gear > 0:
		var up_rpm := 3000.0 + 3300.0 * input_throttle
		if rpm > up_rpm and gear < gears.size() and speed > 1.0:
			gear += 1
			shift_timer = 0.22
		elif gear > 1 and rpm < 1700.0 + 1200.0 * input_throttle:
			# downshift only if the lower gear keeps the engine below the redline
			var next_rpm: float = rpm * gears[gear - 2] / gears[gear - 1]
			if next_rpm < redline - 600:
				gear -= 1
				shift_timer = 0.18

func shift(d: int) -> void:
	var g := clampi(gear + d, -1, gears.size())
	gear = g
	shift_timer = 0.2

func wheel_visual_offset(i: int) -> float:
	return -(wheels[i].len - rest_len)

func telemetry() -> Dictionary:
	var c := 0
	for w: Wheel in wheels:
		c += 1 if w.contact else 0
	return {"speed": speed, "rpm": rpm, "gear": gear, "contacts": c,
		"slip": [wheels[0].slip_ratio, wheels[1].slip_ratio, wheels[2].slip_ratio, wheels[3].slip_ratio],
		"len": [wheels[0].len, wheels[1].len, wheels[2].len, wheels[3].len]}
