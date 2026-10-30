# Streams the world around a focus point: asks the native streamer for chunks, turns them into meshes,
# collision and placed objects, and drops chunks that fell out of range.
extends Node3D
class_name EarthWorld

signal chunk_added(cx: int, cy: int, full: bool)

const I_TREE := 0
const I_CONIFER := 1
const I_BUSH := 2
const I_LAMP := 3
const I_CAR := 4
const I_PERSON := 8

var stream := EarthStream.new()
var mats := EarthMaterials.new()
var chunks := {}         # Vector2i -> Node3D
var full := {}           # Vector2i -> bool (has detail)
var models := {}         # "kind:variant" -> ArrayMesh
var focus := Vector3.ZERO
var budget_ms := 6.0
var lamp_lights: Array[OmniLight3D] = []
var traffic_mm: MultiMeshInstance3D
var traffic_count := 40
var people_mm: MultiMeshInstance3D
var people_count := 60
var lamp_points := {}     # Vector2i -> PackedVector3Array

func start(lat: float, lon: float, cache_dir: String, online := true, detail := 700.0, far := 2400.0, enrich := true) -> void:
	mats.build()
	_build_models()
	DirAccess.make_dir_recursive_absolute(cache_dir)
	stream.start(lat, lon, cache_dir, online, detail, far, enrich)
	for i in 24:
		var l := OmniLight3D.new()
		l.light_color = Color(1.0, 0.8, 0.55)
		l.omni_range = 18.0
		l.light_energy = 0.0
		l.shadow_enabled = false
		l.visible = false
		add_child(l)
		lamp_lights.append(l)

func _build_models() -> void:
	var variants := {I_TREE: 4, I_CONIFER: 3, I_BUSH: 2, I_LAMP: 1, I_CAR: 1}
	for kind in variants:
		for v in variants[kind]:
			var d: Dictionary = EarthCore.model(kind, v)
			var am := ArrayMesh.new()
			for mat in d:
				am.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, d[mat])
				am.surface_set_material(am.get_surface_count() - 1, mats.get_mat(mat))
			models["%d:%d" % [kind, v]] = am

func update_traffic(dt: float, player: Vector3, vel: Vector3) -> void:
	if traffic_mm == null:
		traffic_mm = MultiMeshInstance3D.new()
		var mm := MultiMesh.new()
		mm.transform_format = MultiMesh.TRANSFORM_3D
		mm.use_colors = true
		mm.mesh = models["%d:0" % I_CAR]
		traffic_mm.multimesh = mm
		add_child(traffic_mm)
	var a: PackedFloat32Array = stream.traffic(dt, player, vel, traffic_count)
	var n := a.size() / 8
	var mm2 := traffic_mm.multimesh
	if mm2.instance_count != n:
		mm2.instance_count = n
	for i in n:
		var o := i * 8
		var b := Basis(Vector3.UP, a[o + 3]) * Basis(Vector3(0, 0, 1), a[o + 4])
		mm2.set_instance_transform(i, Transform3D(b, Vector3(a[o], a[o + 1], a[o + 2])))
		mm2.set_instance_color(i, Color(a[o + 5], a[o + 6], a[o + 7]))
	# the multimesh's bounds must follow the cars
	traffic_mm.custom_aabb = AABB(player - Vector3(600, 300, 600), Vector3(1200, 600, 1200))

func update_people(dt: float, player: Vector3) -> void:
	if people_mm == null:
		var d: Dictionary = EarthCore.model(I_PERSON, 0)
		var am := ArrayMesh.new()
		var sm := ShaderMaterial.new()
		sm.shader = load("res://shaders/person.gdshader")
		for mat in d:
			am.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, d[mat])
			am.surface_set_material(am.get_surface_count() - 1, sm)
		people_mm = MultiMeshInstance3D.new()
		var mm := MultiMesh.new()
		mm.transform_format = MultiMesh.TRANSFORM_3D
		mm.use_custom_data = true
		mm.mesh = am
		people_mm.multimesh = mm
		add_child(people_mm)
	var a: PackedFloat32Array = stream.people(dt, player, people_count)
	var n := a.size() / 9
	var mm2 := people_mm.multimesh
	if mm2.instance_count != n:
		mm2.instance_count = n
	for i in n:
		var o := i * 9
		var b := Basis(Vector3.UP, a[o + 3]).scaled(Vector3.ONE * a[o + 8])
		mm2.set_instance_transform(i, Transform3D(b, Vector3(a[o], a[o + 1], a[o + 2])))
		mm2.set_instance_custom_data(i, Color(a[o + 4], a[o + 5], a[o + 6], a[o + 7]))
	people_mm.custom_aabb = AABB(player - Vector3(400, 200, 400), Vector3(800, 400, 800))

func ground(x: float, z: float) -> float:
	return stream.ground(x, z)

func _process(_dt: float) -> void:
	stream.update(focus)
	var t0 := Time.get_ticks_usec()
	while Time.get_ticks_usec() - t0 < budget_ms * 1000.0:
		var d: Dictionary = stream.poll()
		if d.is_empty():
			break
		_add_chunk(d)
	for k in stream.take_drops():
		_drop(k)
	_update_lamp_lights()

func _drop(k: Vector2i) -> void:
	if chunks.has(k):
		chunks[k].queue_free()
		chunks.erase(k)
		full.erase(k)
		lamp_points.erase(k)

func _mesh_instance(arrays: Array, mat: Material, name: String) -> MeshInstance3D:
	var am := ArrayMesh.new()
	am.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	am.surface_set_material(0, mat)
	var mi := MeshInstance3D.new()
	mi.mesh = am
	mi.name = name
	return mi

func _add_chunk(d: Dictionary) -> void:
	var k := Vector2i(d["cx"], d["cy"])
	var is_full: bool = not d["terrain_only"]
	var old: Node3D = chunks.get(k)
	lamp_points.erase(k)
	var root := Node3D.new()
	root.name = "chunk_%d_%d" % [k.x, k.y]
	var size: float = d["size"]
	var x0: float = d["x0"]
	var y0: float = d["y0"]
	# terrain
	var tmat := mats.terrain_material(d["splat0"], d["splat1"], d["splat_n"])
	if is_full:
		var hi := _mesh_instance(d["terrain_hi"], tmat, "terrain_hi")
		hi.visibility_range_end = 600.0
		hi.visibility_range_end_margin = 60.0
		hi.visibility_range_fade_mode = GeometryInstance3D.VISIBILITY_RANGE_FADE_DISABLED
		root.add_child(hi)
		var lo := _mesh_instance(d["terrain_lo"], tmat, "terrain_lo")
		lo.visibility_range_begin = 600.0
		lo.visibility_range_begin_margin = 60.0
		lo.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		root.add_child(lo)
	else:
		var lo2 := _mesh_instance(d["terrain_lo"], tmat, "terrain_lo")
		lo2.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		root.add_child(lo2)
	# collision lives in the native streamer (soft-body contacts); no engine physics bodies needed
	var body: StaticBody3D = null
	# built surfaces: one mesh, a surface per material
	var surf: Dictionary = d["surfaces"]
	if not surf.is_empty():
		var am := ArrayMesh.new()
		for mat in surf:
			am.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, surf[mat])
			am.surface_set_material(am.get_surface_count() - 1, _debug_mat(int(mat)))
		var mi := MeshInstance3D.new()
		mi.mesh = am
		mi.name = "built"
		mi.visibility_range_end = 2600.0
		root.add_child(mi)
	# grass in 64 m blocks, drawn only near the camera
	var grass: Array = d.get("grass", [])
	for gi in grass.size():
		var buf: PackedFloat32Array = grass[gi]
		var cnt := buf.size() / 16
		if cnt == 0:
			continue
		var gm := MultiMesh.new()
		gm.transform_format = MultiMesh.TRANSFORM_3D
		gm.use_colors = true
		gm.mesh = mats.grass_mesh
		gm.instance_count = cnt
		gm.buffer = buf
		var gmi := MultiMeshInstance3D.new()
		gmi.multimesh = gm
		gmi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		gmi.visibility_range_end = 75.0
		root.add_child(gmi)
	# placed objects
	var inst: Dictionary = d["instances"]
	for kind in inst:
		_place(root, body, int(kind), inst[kind], k)
	add_child(root)
	if old:
		old.queue_free()
	chunks[k] = root
	full[k] = is_full
	chunk_added.emit(k.x, k.y, is_full)

var debug_mats := {}
func _debug_mat(id: int) -> Material:
	if debug_mats.has(id):
		var m := StandardMaterial3D.new()
		m.albedo_color = debug_mats[id]
		return m
	return mats.get_mat(id)

func _place(root: Node3D, body: StaticBody3D, kind: int, a: PackedFloat32Array, key: Vector2i) -> void:
	var count := a.size() / 8
	if count == 0:
		return
	var nvar := {I_TREE: 4, I_CONIFER: 3, I_BUSH: 2}.get(kind, 1) as int
	var per := []
	for v in nvar:
		per.append([])
	for i in count:
		var o := i * 8
		var v := int(abs(a[o] * 13.1 + a[o + 2] * 7.7)) % nvar
		per[v].append(o)
	for v in nvar:
		var list: Array = per[v]
		if list.is_empty():
			continue
		var mm := MultiMesh.new()
		mm.transform_format = MultiMesh.TRANSFORM_3D
		mm.use_colors = kind == I_CAR
		mm.mesh = models["%d:%d" % [kind, v]]
		mm.instance_count = list.size()
		for j in list.size():
			var o: int = list[j]
			var pos := Vector3(a[o], a[o + 1], a[o + 2])
			var yaw: float = a[o + 3]
			var sc: float = a[o + 4]
			var b := Basis(Vector3.UP, yaw).scaled(Vector3(sc, sc, sc))
			# the model arrays are already in Godot axes (forward +x, up +y)
			mm.set_instance_transform(j, Transform3D(b, pos))
			if mm.use_colors:
				mm.set_instance_color(j, Color(a[o + 5], a[o + 6], a[o + 7]))
			if kind == I_LAMP:
				var lp: PackedVector3Array = lamp_points.get(key, PackedVector3Array())
				lp.append(pos + Basis(Vector3.UP, yaw) * Vector3(1.75, 7.6, 0))
				lamp_points[key] = lp
		var mmi := MultiMeshInstance3D.new()
		mmi.multimesh = mm
		mmi.visibility_range_end = 1400.0 if kind <= I_BUSH else 700.0
		if kind == I_BUSH:
			mmi.visibility_range_end = 400.0
		root.add_child(mmi)

func _collider(body: StaticBody3D, kind: int, pos: Vector3, yaw: float, sc: float) -> void:
	var cs := CollisionShape3D.new()
	match kind:
		I_TREE, I_CONIFER:
			var c := CylinderShape3D.new()
			c.radius = 0.25 * sc
			c.height = 4.0
			cs.shape = c
			cs.position = pos + Vector3(0, 2.0, 0)
		I_LAMP:
			var c2 := CylinderShape3D.new()
			c2.radius = 0.12
			c2.height = 8.0
			cs.shape = c2
			cs.position = pos + Vector3(0, 4.0, 0)
		I_CAR:
			var bx := BoxShape3D.new()
			bx.size = Vector3(4.4, 1.3, 1.78)
			cs.shape = bx
			cs.position = pos + Vector3(0, 0.75, 0)
			cs.rotation = Vector3(0, yaw, 0)
		_:
			cs.free()
			return
	body.add_child(cs)

func _update_lamp_lights() -> void:
	# lights on the nearest lamps at night
	if mats.night < 0.05:
		for l in lamp_lights:
			l.visible = false
		return
	var cam := get_viewport().get_camera_3d()
	if cam == null:
		return
	var p := cam.global_position
	var best := []
	for key in lamp_points:
		for q in lamp_points[key]:
			var d: float = q.distance_squared_to(p)
			if d < 90000.0:
				best.append([d, q])
	best.sort_custom(func(x, y): return x[0] < y[0])
	for i in lamp_lights.size():
		var l := lamp_lights[i]
		if i < best.size():
			l.global_position = best[i][1] - Vector3(0, 0.4, 0)
			l.light_energy = 2.5 * mats.night
			l.visible = true
		else:
			l.visible = false
