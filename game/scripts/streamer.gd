## Streams world chunks around a focus point: builds them on worker threads (C++ core),
## then creates meshes, collision and instanced props on the main thread within a time budget.
extends Node3D

const MaterialsScript := preload("res://scripts/materials.gd")
const PropsScript := preload("res://scripts/props.gd")

var world: EarthWorld
var mats
var props
var radius := 1000.0          # chunks whose centre is closer than this are loaded
var max_inflight := 2
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

func setup(w: EarthWorld) -> void:
	world = w
	mats = MaterialsScript.new()
	props = PropsScript.new()
	_range = world.chunk_range()
	_far_range = world.far_range()

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
				out.append([d, k])
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
		var ms := (Time.get_ticks_usec() - t) / 1000.0
		stats["built"] += 1
		stats["build_ms"] = d["ms"]
		stats["max_apply_ms"] = max(stats["max_apply_ms"], ms)
		if (Time.get_ticks_usec() - start) / 1000.0 > budget_ms:
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

func _make_node(d: Dictionary) -> Node3D:
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
	for g in [["col_road", 0, 1.0], ["col_ground", 1, 0.75], ["col_solid", 2, 0.6]]:
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
	var inst: Array = d["inst"]
	for kind in min(inst.size(), props.meshes.size()):
		var all: PackedFloat32Array = inst[kind]
		if props.meshes[kind] == null:
			continue
		# trees: variant 3 is a conifer, the rest broadleaf
		var groups := [[all, props.meshes[kind]]]
		if kind == PropsScript.I_TREE:
			var broad := PackedFloat32Array()
			var con := PackedFloat32Array()
			for i in all.size() / 6:
				var dst := con if int(all[i * 6 + 5]) == 3 else broad
				for j in 6:
					dst.append(all[i * 6 + j])
			groups = [[broad, props.meshes[kind]], [con, props.conifer]]
		for grp in groups:
			_add_instances(n, kind, grp[0], grp[1])
	add_child(n)
	return n

func _add_instances(n: Node3D, kind: int, f: PackedFloat32Array, mesh: Mesh) -> void:
		var count := f.size() / 6
		if count == 0:
			return
		var mm := MultiMesh.new()
		mm.transform_format = MultiMesh.TRANSFORM_3D
		mm.use_colors = kind == PropsScript.I_TREE
		mm.use_custom_data = kind == PropsScript.I_SIGNAL
		mm.mesh = mesh
		mm.instance_count = count
		for i in count:
			var p := Vector3(f[i * 6], f[i * 6 + 1], f[i * 6 + 2])
			var yaw := f[i * 6 + 3]
			var sc := f[i * 6 + 4]
			var b := Basis(Vector3.UP, yaw).scaled(Vector3.ONE * sc)
			mm.set_instance_transform(i, Transform3D(b, p))
			if mm.use_colors:
				var v := int(f[i * 6 + 5])
				var tints := [Color(0.85, 1.0, 0.8), Color(1.0, 1.0, 1.0), Color(0.75, 0.9, 0.7), Color(1.05, 0.95, 0.75)]
				mm.set_instance_color(i, tints[v % 4])
			if mm.use_custom_data:
				# perpendicular approaches get opposite phases of the 40 s cycle
				var ns: bool = abs(cos(yaw)) > 0.7071
				mm.set_instance_custom_data(i, Color(0.0 if ns else 20.0, 0, 0, 0))
		var mmi := MultiMeshInstance3D.new()
		mmi.multimesh = mm
		if kind == PropsScript.I_TREE:
			mmi.visibility_range_end = 900.0
		elif kind != PropsScript.I_TREE:
			mmi.visibility_range_end = 450.0
		n.add_child(mmi)
