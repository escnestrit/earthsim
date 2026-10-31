## Traffic: cars driving the road graph around the player. Each car keeps its lane, follows the
## speed limit, keeps a gap to the car (or the player) ahead and stops at red lights, using the same
## signal phases as the visible traffic lights. Cars are kinematic bodies the player can hit.
extends Node3D

const CarModel := preload("res://scripts/car_model.gd")
const COLORS := [Color(0.85, 0.85, 0.86), Color(0.08, 0.08, 0.09), Color(0.55, 0.57, 0.6), Color(0.12, 0.2, 0.42),
	Color(0.55, 0.06, 0.06), Color(0.9, 0.9, 0.88), Color(0.25, 0.3, 0.25), Color(0.6, 0.52, 0.4)]

var world: EarthWorld
var player: Node3D
var max_cars := 26
var rmin := 70.0
var rmax := 420.0
var clock := 0.0            # shared with the traffic light shader (global "signal_time")
var cars := []
var _bodies := []           # one body mesh per colour
var _wheel: Mesh
var _shape := BoxShape3D.new()
var _rng := RandomNumberGenerator.new()
var _spawn_t := 0.0
var _info := {}             # edge -> edge_info cache

func setup(w: EarthWorld, p: Node3D) -> void:
	world = w
	player = p
	_rng.seed = 12345
	for c in COLORS:
		var m := CarModel.new()
		m.body_color = c
		_bodies.append(m.build_body())
	_wheel = CarModel.new().build_wheel(0.32, 0.2)
	_shape.size = Vector3(1.8, 1.3, 4.5)

# a new region: all cars go, new ones spawn on the new road graph
func reset(w: EarthWorld) -> void:
	for c in cars:
		c["body"].queue_free()
	cars.clear()
	_info.clear()
	world = w

func _edge(e: int) -> Dictionary:
	if not _info.has(e):
		_info[e] = world.edge_info(e)
	return _info[e]

# light state for traffic arriving at the end of the current edge: 0 green, 1 amber, 2 red
func light_state(yaw: float) -> int:
	var ns := absf(cos(yaw)) > 0.7071
	var t := fmod(clock + (0.0 if ns else 20.0), 40.0)
	if t < 16.0:
		return 0
	if t < 19.0:
		return 1
	return 2

func _lane_offset(info: Dictionary) -> float:
	var w: float = info["width"]
	var lanes: int = info["lanes"]
	if int(info["oneway"]) != 0:
		var n := maxi(1, lanes)
		var i := _rng.randi_range(0, n - 1)
		return w * 0.5 - (i + 0.5) * w / n
	if w < 5.5:
		return w * 0.25
	return w * 0.5 - minf(1.75, w * 0.25)

func _make_car(e: int, dir: int, s: float) -> void:
	var info := _edge(e)
	var body := AnimatableBody3D.new()
	body.sync_to_physics = false
	body.set_meta("surface", 2)
	var cs := CollisionShape3D.new()
	cs.shape = _shape
	cs.position = Vector3(0, 0.25, 0)
	body.add_child(cs)
	var ci := _rng.randi_range(0, COLORS.size() - 1)
	var mi := MeshInstance3D.new()
	mi.mesh = _bodies[ci]
	body.add_child(mi)
	var wheels := []
	for k in 4:
		var w := MeshInstance3D.new()
		w.mesh = _wheel
		var left := k % 2 == 0
		w.position = Vector3(-0.78 if left else 0.78, -0.02, -1.35 if k < 2 else 1.35)
		if not left:
			w.rotation.y = PI
		body.add_child(w)
		wheels.append(w)
	add_child(body)
	var c := {"body": body, "edge": e, "dir": dir, "s": s, "speed": 8.0, "lane": _lane_offset(info), "lane_cur": 0.0,
		"wheels": wheels, "spin": 0.0, "pos": Vector3.ZERO, "yaw": 0.0, "stopped": 0.0, "fresh": true}
	c["lane_cur"] = c["lane"]
	cars.append(c)

func _physics_process(dt: float) -> void:
	if world == null or player == null:
		return
	clock += dt
	RenderingServer.global_shader_parameter_set("signal_time", clock)
	var pp := player.global_position
	# despawn far cars, spawn new ones out of sight
	for i in range(cars.size() - 1, -1, -1):
		if (cars[i]["pos"] as Vector3).distance_to(pp) > rmax + 150.0 and not cars[i]["fresh"]:
			cars[i]["body"].queue_free()
			cars.remove_at(i)
	_spawn_t -= dt
	if cars.size() < max_cars and _spawn_t <= 0.0:
		_spawn_t = 0.3
		var sp: Array = world.traffic_spawns(pp, rmin, rmax, 2, _rng.randi())
		for d in sp:
			if cars.size() >= max_cars:
				break
			var lp: Dictionary = world.lane_pose(d["edge"], d["s"], d["dir"], 0.0)
			var free := true
			for c in cars:
				if (c["pos"] as Vector3).distance_to(lp["pos"]) < 12.0:
					free = false
			if free:
				_make_car(d["edge"], d["dir"], d["s"])
	for c in cars:
		_drive(c, dt, pp)

func _drive(c: Dictionary, dt: float, pp: Vector3) -> void:
	var info := _edge(c["edge"])
	var L: float = info["len"]
	var dir: int = c["dir"]
	var s: float = c["s"]
	var limit: float = float(info["speed"]) / 3.6 * 0.9
	var want := clampf(limit, 5.0, 22.0)
	var fwd := Vector3(-sin(c["yaw"]), 0, -cos(c["yaw"]))
	var pos: Vector3 = c["pos"]
	# car or player ahead in the lane
	var gap := 1e9
	for o in cars:
		if o == c:
			continue
		var op: Vector3 = o["pos"]
		var rel := op - pos
		var ahead := rel.dot(fwd)
		if ahead > 0.0 and ahead < 45.0 and absf(rel.dot(Vector3(-fwd.z, 0, fwd.x))) < 2.2:
			gap = minf(gap, ahead)
	var prel := pp - pos
	var pahead := prel.dot(fwd)
	if pahead > -1.0 and pahead < 45.0 and absf(prel.dot(Vector3(-fwd.z, 0, fwd.x))) < 2.4:
		gap = minf(gap, maxf(pahead, 0.0))
	if gap < 1e8:
		want = minf(want, maxf(0.0, (gap - 7.5) / 1.4))
	# red light at the end of this edge
	var to_end := (L - s) if dir > 0 else s
	var trim_end: float = info["trim_b"] if dir > 0 else info["trim_a"]
	var signals: bool = info["signals_b"] if dir > 0 else info["signals_a"]
	if signals:
		var stop_d := to_end - trim_end - 2.5
		if stop_d > -1.0 and stop_d < 40.0:
			var st := light_state(c["yaw"])
			if st == 2 or (st == 1 and stop_d > 6.0):
				want = minf(want, sqrt(maxf(0.0, 2.0 * 3.5 * maxf(0.0, stop_d - 0.5))))
	# hit by the player: stop for a while
	if pos.distance_to(pp) < 3.2:
		c["stopped"] = 4.0
	if c["stopped"] > 0.0:
		c["stopped"] -= dt
		want = 0.0
	var v: float = c["speed"]
	v = move_toward(v, want, (2.5 if want > v else 6.5) * dt)
	c["speed"] = v
	s += v * dt * dir
	# next edge
	var guard := 0
	while (s > L or s < 0.0) and guard < 4:
		guard += 1
		var over := (s - L) if s > L else -s
		var nx: Dictionary = world.next_edge(c["edge"], dir, _rng.randf())
		if nx.is_empty():
			# dead end: turn round
			dir = -dir
			s = L - over if s > L else over
			c["dir"] = dir
			break
		c["edge"] = nx["edge"]
		dir = nx["dir"]
		c["dir"] = dir
		info = _edge(c["edge"])
		L = info["len"]
		s = over if dir > 0 else L - over
		c["lane"] = _lane_offset(info)
	c["s"] = s
	c["lane_cur"] = lerpf(c["lane_cur"], c["lane"], clampf(dt * 1.5, 0.0, 1.0))
	var lp: Dictionary = world.lane_pose(c["edge"], s, dir, c["lane_cur"])
	if lp.is_empty():
		return
	var tp: Vector3 = lp["pos"] + Vector3(0, 0.34, 0)
	var ty: float = lp["yaw"]
	if c["fresh"]:
		c["pos"] = tp
		c["yaw"] = ty
		c["fresh"] = false
	else:
		# smooth the corners at junctions
		c["pos"] = (c["pos"] as Vector3).lerp(tp, clampf(dt * 8.0, 0.0, 1.0))
		c["yaw"] = lerp_angle(c["yaw"], ty, clampf(dt * 5.0, 0.0, 1.0))
	var pitch := atan(float(lp["slope"]))
	var b := Basis(Vector3.UP, c["yaw"]) * Basis(Vector3.RIGHT, pitch)
	(c["body"] as AnimatableBody3D).global_transform = Transform3D(b, c["pos"])
	c["spin"] = fmod(c["spin"] + v * dt / 0.32, TAU)
	for k in 4:
		var wn: MeshInstance3D = c["wheels"][k]
		wn.rotation.x = -c["spin"] if k % 2 == 0 else c["spin"]
