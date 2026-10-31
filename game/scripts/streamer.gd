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

func setup(w: EarthWorld) -> void:
	world = w
	mats = MaterialsScript.new()
	props = PropsScript.new()
	_range = world.chunk_range()

func chunk_of(p: Vector3) -> Vector2i:
	var s := world.chunk_size()
	return Vector2i(floori(p.x / s), floori(-p.z / s))

func is_loaded_at(p: Vector3) -> bool:
	return chunks.has(chunk_of(p))

func loading_count() -> int:
	return inflight.size() + done.size()

func _process(_dt: float) -> void:
	if world == null:
		return
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
	for item in _wanted():
		if inflight.size() >= max_inflight:
			break
		var k: Vector2i = item[1]
		if chunks.has(k) or inflight.has(k):
			continue
		inflight[k] = WorkerThreadPool.add_task(_build.bind(k), false, "chunk")

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
		var f: PackedFloat32Array = inst[kind]
		var count := f.size() / 6
		if count == 0 or props.meshes[kind] == null:
			continue
		var mm := MultiMesh.new()
		mm.transform_format = MultiMesh.TRANSFORM_3D
		mm.use_colors = kind == PropsScript.I_TREE
		mm.use_custom_data = kind == PropsScript.I_SIGNAL
		mm.mesh = props.meshes[kind]
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
	add_child(n)
	return n
