## Streams world chunks around a focus point: builds them on worker threads (C++ core),
## then creates meshes, collision and instanced props on the main thread within a time budget.
extends Node3D

const MaterialsScript := preload("res://scripts/materials.gd")
const PropsScript := preload("res://scripts/props.gd")

var world: EarthWorld
var mats
var props
var radius := 1000.0          # chunks whose centre is closer than this are loaded
var max_inflight := clampi(OS.get_processor_count() - 1, 2, 6)
var lead := Vector3.ZERO      # look-ahead offset (car velocity): build order favours chunks ahead
var budget_ms := 6.0
var focus := Vector3.ZERO
var chunks := {}              # Vector2i -> Node3D
var inflight := {}            # Vector2i -> task id
var done := []                # finished dictionaries waiting for the main thread
var _mutex := Mutex.new()
var _range := Rect2i()
var stats := {"built": 0, "build_ms": 0.0, "max_apply_ms": 0.0}
var far_enabled := true
var far_cells := {}           # Vector2i -> Node3D (far LOD)
var far_inflight := {}
var _far_range := Rect2i()

var legacy: Array[Node3D] = []   # chunks of the previous region, kept until the new ones cover them

func setup(w: EarthWorld) -> void:
	world = w
	mats = MaterialsScript.new()
	props = PropsScript.new()
	_range = world.chunk_range()
	_far_range = world.far_range()

# never leave worker threads running on a world that is being freed
func _exit_tree() -> void:
	for k in inflight:
		WorkerThreadPool.wait_for_task_completion(inflight[k])
	for k in far_inflight:
		WorkerThreadPool.wait_for_task_completion(far_inflight[k])
	inflight.clear()
	far_inflight.clear()

func busy() -> bool:
	return not inflight.is_empty() or not far_inflight.is_empty()

# Switch to another region. keep = true: the old chunks stay on screen (moved by `shift`, the
# offset between the two local frames) until new chunks replace them; false: everything goes.
func set_world(w: EarthWorld, keep: bool, shift: Vector3) -> void:
	for k in inflight:
		WorkerThreadPool.wait_for_task_completion(inflight[k])
	for k in far_inflight:
		WorkerThreadPool.wait_for_task_completion(far_inflight[k])
	inflight.clear()
	far_inflight.clear()
	_mutex.lock()
	done.clear()
	_mutex.unlock()
	for n in legacy:
		n.queue_free()
	legacy.clear()
	for k in chunks:
		var n: Node3D = chunks[k]
		if keep:
			n.position += shift
			n.set_meta("lamps", PackedVector3Array())
			legacy.append(n)
		else:
			n.queue_free()
	for k in far_cells:
		far_cells[k].queue_free()
	chunks.clear()
	far_cells.clear()
	world = w
	_range = world.chunk_range()
	_far_range = world.far_range()
	focus += shift

func _drop_legacy(k: Vector2i) -> void:
	if legacy.is_empty():
		return
	var s := world.chunk_size()
	var r := Rect2(k.x * s, -(k.y + 1) * s, s, s)
	for i in range(legacy.size() - 1, -1, -1):
		var p: Vector3 = legacy[i].position
		if r.has_point(Vector2(p.x, p.z)):
			legacy[i].queue_free()
			legacy.remove_at(i)

func chunk_of(p: Vector3) -> Vector2i:
	var s := world.chunk_size()
	return Vector2i(floori(p.x / s), floori(-p.z / s))

func is_loaded_at(p: Vector3) -> bool:
	return chunks.has(chunk_of(p))

func loading_count() -> int:
	return inflight.size() + done.size() + _near_missing()

func _near_missing() -> int:
	var n := 0
	for item in _wanted():
		if not chunks.has(item[1]):
			n += 1
	return n

func _process(_dt: float) -> void:
	if world == null:
		return
	# the far LOD is cut away inside the ring that is certainly covered by loaded chunks
	var covered := 0.0
	var s := world.chunk_size()
	var c := chunk_of(focus)
	var r := 0
	while r < 16:
		var full := true
		for dy in range(-r - 1, r + 2):
			for dx in range(-r - 1, r + 2):
				if absi(dx) <= r and absi(dy) <= r:
					continue
				var k := Vector2i(c.x + dx, c.y + dy)
				if _range.has_point(k) and not chunks.has(k):
					full = false
		if not full:
			break
		r += 1
	covered = maxf(0.0, r * s - s * 0.1) if chunks.has(c) else 0.0
	RenderingServer.global_shader_parameter_set("detail_center", focus)
	RenderingServer.global_shader_parameter_set("detail_radius", covered)
	_apply_finished()
	_schedule()
	_unload()
	if not legacy.is_empty() and _near_missing() == 0 and inflight.is_empty():
		for n in legacy:
			n.queue_free()
		legacy.clear()

func _wanted() -> Array:
	var s := world.chunk_size()
	var c := chunk_of(focus)
	var r := int(ceil(radius / s)) + 1
	var out := []
	for dy in range(-r, r + 1):
		for dx in range(-r, r + 1):
			var k := Vector2i(c.x + dx, c.y + dy)
			if not _range.has_point(k):
				continue
			var centre := Vector3((k.x + 0.5) * s, 0, -(k.y + 0.5) * s)
			var d := Vector2(centre.x - focus.x, centre.z - focus.z).length()
			if d <= radius:
				# priority: distance from the point the car will be at in a few seconds,
				# but never let a chunk right next to the car wait
				var dl := Vector2(centre.x - focus.x - lead.x, centre.z - focus.z - lead.z).length()
				out.append([minf(dl, d * 1.6) if d > s else 0.0, k])
	out.sort_custom(func(a, b): return a[0] < b[0])
	return out

func _schedule() -> void:
	var near_pending := false
	for item in _wanted():
		var k: Vector2i = item[1]
		if chunks.has(k) or inflight.has(k):
			continue
		near_pending = true
		if inflight.size() + far_inflight.size() >= max_inflight:
			break
		inflight[k] = WorkerThreadPool.add_task(_build.bind(k), false, "chunk")
	# far LOD only when the near ring is complete (and one at a time)
	if far_enabled and not near_pending and far_inflight.is_empty() and inflight.size() < max_inflight:
		var best := Vector2i(-99999, 0)
		var bd := 1e18
		var fs := world.far_size()
		for y in range(_far_range.position.y, _far_range.end.y):
			for x in range(_far_range.position.x, _far_range.end.x):
				var k := Vector2i(x, y)
				if far_cells.has(k):
					continue
				var c := Vector3((x + 0.5) * fs, 0, -(y + 0.5) * fs)
				var d := Vector2(c.x - focus.x, c.z - focus.z).length()
				if d < bd:
					bd = d
					best = k
		if best.x != -99999:
			far_inflight[best] = WorkerThreadPool.add_task(_build_far.bind(best), false, "far")

func far_total() -> int:
	return _far_range.size.x * _far_range.size.y

func _build_far(k: Vector2i) -> void:
	var d: Dictionary = world.build_far(k.x, k.y)
	d["far"] = true
	_mutex.lock()
	done.append(d)
	_mutex.unlock()

func _build(k: Vector2i) -> void:
	var t := Time.get_ticks_usec()
	var d: Dictionary = world.build_chunk(k.x, k.y)
	d["ms"] = (Time.get_ticks_usec() - t) / 1000.0
	_mutex.lock()
	done.append(d)
	_mutex.unlock()

func _apply_finished() -> void:
	var start := Time.get_ticks_usec()
	while true:
		_mutex.lock()
		var d = done.pop_front() if done.size() > 0 else null
		_mutex.unlock()
		if d == null:
			break
		var k := Vector2i(d["cx"], d["cy"])
		if d.has("far"):
			WorkerThreadPool.wait_for_task_completion(far_inflight[k])
			far_inflight.erase(k)
			far_cells[k] = _make_far(d)
			continue
		if inflight.has(k):
			WorkerThreadPool.wait_for_task_completion(inflight[k])
			inflight.erase(k)
		var t := Time.get_ticks_usec()
		chunks[k] = _make_node(d)
		_drop_legacy(k)
		var ms := (Time.get_ticks_usec() - t) / 1000.0
		# one chunk per frame keeps the frame time flat while streaming
		stats["max_apply_ms"] = max(stats["max_apply_ms"], ms)
		stats["built"] += 1
		stats["build_ms"] = d["ms"]
		# one chunk per frame normally; catch up (within the budget) when several are waiting
		_mutex.lock()
		var pending := done.size()
		_mutex.unlock()
		if pending < 3 or (Time.get_ticks_usec() - start) / 1000.0 > budget_ms * 0.5:
			break

func _unload() -> void:
	var s := world.chunk_size()
	for k in chunks.keys():
		var centre := Vector3((k.x + 0.5) * s, 0, -(k.y + 0.5) * s)
		if Vector2(centre.x - focus.x, centre.z - focus.z).length() > radius + s * 1.5:
			chunks[k].queue_free()
			chunks.erase(k)

func _make_far(d: Dictionary) -> Node3D:
	var n := Node3D.new()
	n.position = d["origin"]
	var mesh := ArrayMesh.new()
	var surf := 0
	for s in d["surfaces"]:
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, s[1])
		mesh.surface_set_material(surf, mats.far_list[s[0]])
		surf += 1
	var mi := MeshInstance3D.new()
	mi.mesh = mesh
	mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	n.add_child(mi)
	add_child(n)
	return n

var prof := {"mesh": 0.0, "col": 0.0, "inst": 0.0, "n": 0}
var _trunk := CylinderShape3D.new()
var _pole := CylinderShape3D.new()
var _car_box := BoxShape3D.new()
var _bin_box := BoxShape3D.new()
var _stop_box := BoxShape3D.new()
var _play_box := BoxShape3D.new()

func _init() -> void:
	_trunk.radius = 0.3
	_trunk.height = 4.0
	_pole.radius = 0.13
	_pole.height = 6.0
	_car_box.size = Vector3(1.8, 1.4, 4.3)
	_bin_box.size = Vector3(4.2, 1.2, 1.1)
	_stop_box.size = Vector3(4.0, 2.4, 0.25)
	_play_box.size = Vector3(2.4, 2.0, 2.4)

# trunks and poles are solid: shapes added straight to one static body per chunk
func _prop_collision(n: Node3D, inst: Array) -> void:
	var body := StaticBody3D.new()
	body.set_meta("surface", 2)
	n.add_child(body)
	var rid := body.get_rid()
	var groups := [[PropsScript.I_TREE, _trunk, 2.0], [PropsScript.I_LAMP, _pole, 3.0], [PropsScript.I_SIGNAL, _pole, 1.5]]
	for g in groups:
		var f: PackedFloat32Array = inst[g[0]]
		var sh: Shape3D = g[1]
		for i in f.size() / 6:
			var sc := f[i * 6 + 4] if g[0] == PropsScript.I_TREE else 1.0
			var p := Vector3(f[i * 6], f[i * 6 + 1] + g[2] * sc, f[i * 6 + 2])
			PhysicsServer3D.body_add_shape(rid, sh.get_rid(), Transform3D(Basis().scaled(Vector3(sc, sc, sc)), p))
	# boxes: parked cars, containers, shelters (back wall), playground equipment
	for g in [[PropsScript.I_CAR, _car_box, Vector3(0, 0.72, 0)], [PropsScript.I_BIN, _bin_box, Vector3(0, 0.6, 0)],
			[PropsScript.I_STOP, _stop_box, Vector3(0, 1.2, 0.75)], [PropsScript.I_PLAY, _play_box, Vector3(0, 1.0, 0)]]:
		var f: PackedFloat32Array = inst[g[0]]
		var sh: Shape3D = g[1]
		for i in f.size() / 6:
			var b := Basis(Vector3.UP, f[i * 6 + 3])
			var p := Vector3(f[i * 6], f[i * 6 + 1], f[i * 6 + 2]) + b * (g[2] as Vector3)
			PhysicsServer3D.body_add_shape(rid, sh.get_rid(), Transform3D(b, p))

func _make_node(d: Dictionary) -> Node3D:
	var t0 := Time.get_ticks_usec()
	var n := Node3D.new()
	n.name = "chunk_%d_%d" % [d["cx"], d["cy"]]
	n.position = d["origin"]
	var mesh := ArrayMesh.new()
	var surf := 0
	for s in d["surfaces"]:
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, s[1])
		mesh.surface_set_material(surf, mats.list[s[0]])
		surf += 1
	if surf > 0:
		var mi := MeshInstance3D.new()
		mi.mesh = mesh
		n.add_child(mi)
	var t1 := Time.get_ticks_usec()
	var use_hm: bool = d.has("hm")
	if use_hm:
		var hb := StaticBody3D.new()
		hb.set_meta("surface", 1)
		var hpm := PhysicsMaterial.new()
		hpm.friction = 0.75
		hb.physics_material_override = hpm
		var hs := HeightMapShape3D.new()
		hs.map_width = d["hm_size"]
		hs.map_depth = d["hm_size"]
		hs.map_data = d["hm"]
		var hcs := CollisionShape3D.new()
		hcs.shape = hs
		var sc: float = d["hm_scale"]
		hcs.scale = Vector3(sc, sc, sc)
		hb.add_child(hcs)
		n.add_child(hb)
	for g in [["col_road", 0, 1.0], ["col_ground", 1, 0.75], ["col_solid", 2, 0.6]]:
		if use_hm and g[0] == "col_ground":
			continue
		var faces: PackedVector3Array = d[g[0]]
		if faces.size() < 3:
			continue
		var body := StaticBody3D.new()
		body.set_meta("surface", g[1])
		var pm := PhysicsMaterial.new()
		pm.friction = g[2]
		body.physics_material_override = pm
		var shape := ConcavePolygonShape3D.new()
		shape.set_faces(faces)
		var cs := CollisionShape3D.new()
		cs.shape = shape
		body.add_child(cs)
		n.add_child(body)
	var t2 := Time.get_ticks_usec()
	var inst: Array = d["inst"]
	# lamp positions (world) for the night light pool
	var lp: PackedFloat32Array = inst[PropsScript.I_LAMP]
	var lamps := PackedVector3Array()
	for i in lp.size() / 6:
		lamps.append(Vector3(lp[i * 6], lp[i * 6 + 1], lp[i * 6 + 2]) + n.position)
	n.set_meta("lamps", lamps)
	_prop_collision(n, inst)
	var mm: Dictionary = d["mm"]
	# near: detailed meshes; far: cheap ones (switch ~250 m from the chunk centre)
	_mm(n, mm["tree_broad"], props.meshes[PropsScript.I_TREE], 0.0, 260.0)
	_mm(n, mm["tree_broad"], props.tree_lo, 240.0, 1000.0)
	_mm(n, mm["tree_conifer"], props.conifer, 0.0, 260.0)
	_mm(n, mm["tree_conifer"], props.conifer_lo, 240.0, 1000.0)
	_mm(n, mm["tree_palm"], props.palm, 0.0, 600.0)
	_mm(n, mm["lamp"], props.meshes[PropsScript.I_LAMP], 0.0, 450.0)
	_mm(n, mm["signal"], props.meshes[PropsScript.I_SIGNAL], 0.0, 450.0)
	_mm(n, mm["bench"], props.meshes[PropsScript.I_BENCH], 0.0, 200.0)
	_mm(n, mm["car"], props.meshes[PropsScript.I_CAR], 0.0, 380.0)
	_mm(n, mm["bin"], props.meshes[PropsScript.I_BIN], 0.0, 220.0)
	_mm(n, mm["stop"], props.meshes[PropsScript.I_STOP], 0.0, 300.0)
	for k in 3:
		_mm(n, mm["play%d" % k], props.play[k], 0.0, 220.0)
	add_child(n)
	var t3 := Time.get_ticks_usec()
	prof["mesh"] += (t1 - t0) / 1000.0
	prof["col"] += (t2 - t1) / 1000.0
	prof["inst"] += (t3 - t2) / 1000.0
	prof["n"] += 1
	return n

func _mm(n: Node3D, e: Dictionary, mesh: Mesh, vis_begin: float, vis_end: float) -> void:
	var count: int = e["count"]
	if count == 0 or mesh == null:
		return
	var mm := MultiMesh.new()
	mm.transform_format = MultiMesh.TRANSFORM_3D
	mm.use_colors = e["colors"]
	mm.use_custom_data = e["custom"]
	mm.mesh = mesh
	mm.instance_count = count
	mm.buffer = e["buffer"]
	var mmi := MultiMeshInstance3D.new()
	mmi.multimesh = mm
	mmi.visibility_range_begin = vis_begin
	mmi.visibility_range_end = vis_end
	if vis_begin > 0.0:
		mmi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	n.add_child(mmi)
