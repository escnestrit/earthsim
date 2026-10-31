## Procedural meshes for instanced street objects: trees, street lamps, traffic signals, benches.
extends RefCounted

const I_TREE := 0
const I_LAMP := 1
const I_SIGNAL := 2
const I_BENCH := 3
const I_STOP := 4
const I_CAR := 6
const I_BIN := 7
const I_PLAY := 8
const I_COUNT := 9

var meshes: Array[Mesh] = []
var conifer: Mesh
var tree_lo: Mesh      # far versions: one blob crown / one cone
var conifer_lo: Mesh
var lamp_head_mat: StandardMaterial3D
var signal_mat: ShaderMaterial
var play: Array[Mesh] = []
var palm: Mesh

func _init() -> void:
	meshes.resize(I_COUNT)
	meshes[I_TREE] = _tree()
	conifer = _conifer()
	tree_lo = _tree_lo()
	conifer_lo = _conifer_lo()
	meshes[I_LAMP] = _lamp()
	meshes[I_SIGNAL] = _signal()
	meshes[I_BENCH] = _bench()
	meshes[I_CAR] = _parked_car()
	meshes[I_BIN] = _bins()
	meshes[I_STOP] = _shelter()
	play = [_swing(), _slide(), _sandbox()]
	palm = _palm()

static func _mat(c: Color, rough := 0.8, metal := 0.0) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.albedo_color = c
	m.roughness = rough
	m.metallic = metal
	return m

# Appends a box (centre c, size s, rotated about Y by yaw) to SurfaceTool.
static func _box(st: SurfaceTool, c: Vector3, s: Vector3) -> void:
	var h := s * 0.5
	var faces := [
		[Vector3(1, 0, 0), Vector3(0, 0, -1), Vector3(0, 1, 0)],
		[Vector3(-1, 0, 0), Vector3(0, 0, 1), Vector3(0, 1, 0)],
		[Vector3(0, 1, 0), Vector3(1, 0, 0), Vector3(0, 0, -1)],
		[Vector3(0, -1, 0), Vector3(1, 0, 0), Vector3(0, 0, 1)],
		[Vector3(0, 0, 1), Vector3(1, 0, 0), Vector3(0, 1, 0)],
		[Vector3(0, 0, -1), Vector3(-1, 0, 0), Vector3(0, 1, 0)],
	]
	for f in faces:
		var n: Vector3 = f[0]
		var u: Vector3 = f[1]
		var v: Vector3 = f[2]
		var o := c + n * h * n.abs().dot(Vector3.ONE)
		var hu: float = (u.abs() * h).length()
		var hv: float = (v.abs() * h).length()
		var p := [o - u * hu - v * hv, o + u * hu - v * hv, o + u * hu + v * hv, o - u * hu + v * hv]
		# Godot front faces are clockwise as seen from outside
		for idx in [0, 2, 1, 0, 3, 2]:
			st.set_normal(n)
			st.set_uv(Vector2(p[idx].x + p[idx].z, p[idx].y))
			st.add_vertex(p[idx])

static func _cyl(st: SurfaceTool, base: Vector3, r0: float, r1: float, h: float, seg: int) -> void:
	for i in seg:
		var a0 := TAU * i / seg
		var a1 := TAU * (i + 1) / seg
		var d0 := Vector3(cos(a0), 0, sin(a0))
		var d1 := Vector3(cos(a1), 0, sin(a1))
		var p := [base + d0 * r0, base + d1 * r0, base + d1 * r1 + Vector3(0, h, 0), base + d0 * r1 + Vector3(0, h, 0)]
		var n := [d0, d1, d1, d0]
		for idx in [0, 2, 1, 0, 3, 2]:
			st.set_normal(n[idx])
			st.set_uv(Vector2(float(i) / seg, p[idx].y))
			st.add_vertex(p[idx])

# Noisy blob (subdivided icosphere displaced by value noise) for tree crowns.
static func _blob(st: SurfaceTool, c: Vector3, r: Vector3, seed: int) -> void:
	var sm := SphereMesh.new()
	sm.radius = 1.0
	sm.height = 2.0
	sm.radial_segments = 12
	sm.rings = 7
	var arr := sm.get_mesh_arrays()
	var verts: PackedVector3Array = arr[Mesh.ARRAY_VERTEX]
	var idx: PackedInt32Array = arr[Mesh.ARRAY_INDEX]
	var noise := FastNoiseLite.new()
	noise.seed = seed
	noise.frequency = 1.6
	var moved := PackedVector3Array()
	moved.resize(verts.size())
	for i in verts.size():
		var v := verts[i]
		var k := 1.0 + 0.28 * noise.get_noise_3dv(v)
		moved[i] = c + Vector3(v.x * r.x, v.y * r.y, v.z * r.z) * k
	for t in range(0, idx.size(), 3):
		var a := moved[idx[t]]
		var b := moved[idx[t + 1]]
		var d := moved[idx[t + 2]]
		for q in [a, b, d]:
			var n: Vector3 = (q - c).normalized()
			st.set_normal(n)
			st.add_vertex(q)

func _tree() -> Mesh:
	var trunk := SurfaceTool.new()
	trunk.begin(Mesh.PRIMITIVE_TRIANGLES)
	_cyl(trunk, Vector3.ZERO, 0.28, 0.17, 4.6, 8)
	_cyl(trunk, Vector3(0, 4.4, 0), 0.12, 0.06, 2.2, 6)
	var crown := SurfaceTool.new()
	crown.begin(Mesh.PRIMITIVE_TRIANGLES)
	var blobs := [[Vector3(0, 6.6, 0), Vector3(2.6, 2.3, 2.6)], [Vector3(1.6, 7.4, 0.7), Vector3(1.8, 1.7, 1.8)],
		[Vector3(-1.4, 7.8, -0.9), Vector3(1.9, 1.7, 1.8)], [Vector3(0.4, 8.9, -0.3), Vector3(1.6, 1.4, 1.6)],
		[Vector3(-0.8, 6.0, 1.5), Vector3(1.6, 1.4, 1.5)], [Vector3(1.0, 5.7, -1.6), Vector3(1.5, 1.3, 1.5)]]
	var seed := 3
	for b in blobs:
		_blob(crown, b[0], b[1], seed)
		seed += 4
	var m := trunk.commit()
	m.surface_set_material(0, _mat(Color(0.25, 0.21, 0.17), 0.95))
	crown.commit(m)
	var leaf := ShaderMaterial.new()
	leaf.shader = load("res://shaders/foliage.gdshader")
	m.surface_set_material(1, leaf)
	return m

func _tree_lo() -> Mesh:
	var trunk := SurfaceTool.new()
	trunk.begin(Mesh.PRIMITIVE_TRIANGLES)
	_cyl(trunk, Vector3.ZERO, 0.25, 0.15, 5.0, 5)
	var crown := SurfaceTool.new()
	crown.begin(Mesh.PRIMITIVE_TRIANGLES)
	var sm := SphereMesh.new()
	sm.radius = 1.0
	sm.height = 2.0
	sm.radial_segments = 8
	sm.rings = 4
	var arr := sm.get_mesh_arrays()
	var v: PackedVector3Array = arr[Mesh.ARRAY_VERTEX]
	var idx: PackedInt32Array = arr[Mesh.ARRAY_INDEX]
	for t in range(0, idx.size(), 3):
		for k in [0, 1, 2]:
			var q: Vector3 = v[idx[t + k]]
			crown.set_normal(q.normalized())
			crown.add_vertex(Vector3(0, 7.2, 0) + Vector3(q.x * 3.2, q.y * 2.9, q.z * 3.2))
	var m := trunk.commit()
	m.surface_set_material(0, meshes[I_TREE].surface_get_material(0))
	crown.commit(m)
	m.surface_set_material(1, meshes[I_TREE].surface_get_material(1))
	return m

func _conifer_lo() -> Mesh:
	var trunk := SurfaceTool.new()
	trunk.begin(Mesh.PRIMITIVE_TRIANGLES)
	_cyl(trunk, Vector3.ZERO, 0.2, 0.1, 2.0, 4)
	var crown := SurfaceTool.new()
	crown.begin(Mesh.PRIMITIVE_TRIANGLES)
	_cyl(crown, Vector3(0, 1.5, 0), 2.6, 0.05, 10.5, 7)
	var m := trunk.commit()
	m.surface_set_material(0, conifer.surface_get_material(0))
	crown.commit(m)
	m.surface_set_material(1, conifer.surface_get_material(1))
	return m

func _conifer() -> Mesh:
	var trunk := SurfaceTool.new()
	trunk.begin(Mesh.PRIMITIVE_TRIANGLES)
	_cyl(trunk, Vector3.ZERO, 0.2, 0.1, 3.0, 6)
	var crown := SurfaceTool.new()
	crown.begin(Mesh.PRIMITIVE_TRIANGLES)
	# stacked drooping cones
	for k in 6:
		var y := 1.6 + k * 1.75
		var r := 2.6 - k * 0.38
		_cyl(crown, Vector3(0, y, 0), r, 0.15, 2.6, 9)
	var m := trunk.commit()
	m.surface_set_material(0, _mat(Color(0.25, 0.2, 0.16), 0.95))
	crown.commit(m)
	var needles := ShaderMaterial.new()
	needles.shader = load("res://shaders/foliage.gdshader")
	needles.set_shader_parameter("base_color", Vector3(0.1, 0.2, 0.12))
	m.surface_set_material(1, needles)
	return m

func _lamp() -> Mesh:
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	_cyl(st, Vector3.ZERO, 0.11, 0.07, 8.5, 8)
	_box(st, Vector3(0, 8.45, -0.8), Vector3(0.08, 0.08, 1.6))
	var m := st.commit()
	m.surface_set_material(0, _mat(Color(0.32, 0.33, 0.34), 0.5, 0.6))
	var head := SurfaceTool.new()
	head.begin(Mesh.PRIMITIVE_TRIANGLES)
	_box(head, Vector3(0, 8.35, -1.6), Vector3(0.35, 0.16, 0.7))
	head.commit(m)
	lamp_head_mat = _mat(Color(0.9, 0.88, 0.8), 0.4)
	lamp_head_mat.emission_enabled = true
	lamp_head_mat.emission = Color(1.0, 0.82, 0.55)
	lamp_head_mat.emission_energy_multiplier = 0.0
	m.surface_set_material(1, lamp_head_mat)
	return m

func _signal() -> Mesh:
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	_cyl(st, Vector3.ZERO, 0.07, 0.06, 3.0, 8)
	_box(st, Vector3(0, 3.0, 0), Vector3(0.34, 1.0, 0.24))
	var m := st.commit()
	m.surface_set_material(0, _mat(Color(0.12, 0.12, 0.12), 0.6))
	# three lenses on the -Z face (towards the traffic); UV.y tells which (0 red, 1 amber, 2 green)
	var lens := SurfaceTool.new()
	lens.begin(Mesh.PRIMITIVE_TRIANGLES)
	for k in 3:
		var cy := 3.3 - k * 0.3
		var p := [Vector3(-0.1, cy - 0.1, -0.125), Vector3(0.1, cy - 0.1, -0.125), Vector3(0.1, cy + 0.1, -0.125), Vector3(-0.1, cy + 0.1, -0.125)]
		for idx in [0, 1, 2, 0, 2, 3]:
			lens.set_normal(Vector3(0, 0, -1))
			lens.set_uv(Vector2(0, k))
			lens.add_vertex(p[idx])
	lens.commit(m)
	signal_mat = ShaderMaterial.new()
	signal_mat.shader = load("res://shaders/signal.gdshader")
	m.surface_set_material(1, signal_mat)
	return m

func _bench() -> Mesh:
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	_box(st, Vector3(0, 0.45, 0), Vector3(1.6, 0.06, 0.45))
	_box(st, Vector3(0, 0.75, 0.22), Vector3(1.6, 0.4, 0.05))
	_box(st, Vector3(-0.7, 0.22, 0), Vector3(0.06, 0.45, 0.4))
	_box(st, Vector3(0.7, 0.22, 0), Vector3(0.06, 0.45, 0.4))
	var m := st.commit()
	m.surface_set_material(0, _mat(Color(0.35, 0.25, 0.16), 0.8))
	return m

func set_night(n: float) -> void:
	lamp_head_mat.emission_energy_multiplier = 4.0 * n

# box of any orientation (size s, transform t) appended to a SurfaceTool
static func _pbox(st: SurfaceTool, s: Vector3, t: Transform3D) -> void:
	var bm := BoxMesh.new()
	bm.size = s
	st.append_from(bm, 0, t)

static func _at(x: float, y: float, z: float) -> Transform3D:
	return Transform3D(Basis(), Vector3(x, y, z))

func _parked_car() -> Mesh:
	# low-poly parked car; front is -Z; the body takes the instance colour
	var body := SurfaceTool.new()
	body.begin(Mesh.PRIMITIVE_TRIANGLES)
	_pbox(body, Vector3(1.78, 0.62, 4.3), _at(0, 0.62, 0))
	_pbox(body, Vector3(1.7, 0.14, 1.2), _at(0, 0.98, -1.35))   # bonnet
	_pbox(body, Vector3(1.5, 0.07, 1.95), _at(0, 1.47, 0.25))   # roof
	var m := body.commit()
	var paint := _mat(Color(1, 1, 1), 0.35, 0.3)
	paint.vertex_color_use_as_albedo = true
	m.surface_set_material(0, paint)
	var glass := SurfaceTool.new()
	glass.begin(Mesh.PRIMITIVE_TRIANGLES)
	_pbox(glass, Vector3(1.56, 0.5, 2.15), _at(0, 1.2, 0.25))
	glass.commit(m)
	m.surface_set_material(1, _mat(Color(0.06, 0.07, 0.08), 0.1, 0.4))
	var tyres := SurfaceTool.new()
	tyres.begin(Mesh.PRIMITIVE_TRIANGLES)
	for sx in [-0.8, 0.8]:
		for sz in [-1.35, 1.35]:
			_pbox(tyres, Vector3(0.24, 0.64, 0.64), _at(sx, 0.32, sz))
	_pbox(tyres, Vector3(1.8, 0.16, 0.1), _at(0, 0.45, -2.16))   # bumpers
	_pbox(tyres, Vector3(1.8, 0.16, 0.1), _at(0, 0.45, 2.16))
	tyres.commit(m)
	m.surface_set_material(2, _mat(Color(0.05, 0.05, 0.05), 0.9))
	return m

func _bins() -> Mesh:
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	for i in 3:
		_pbox(st, Vector3(1.25, 1.05, 1.0), _at(-1.4 + i * 1.4, 0.6, 0))
	var m := st.commit()
	m.surface_set_material(0, _mat(Color(0.2, 0.36, 0.24), 0.6, 0.4))
	var lid := SurfaceTool.new()
	lid.begin(Mesh.PRIMITIVE_TRIANGLES)
	for i in 3:
		_pbox(lid, Vector3(1.3, 0.08, 1.05), _at(-1.4 + i * 1.4, 1.16, 0))
		_pbox(lid, Vector3(0.12, 0.12, 0.12), _at(-1.4 + i * 1.4 - 0.5, 0.06, 0.4))
		_pbox(lid, Vector3(0.12, 0.12, 0.12), _at(-1.4 + i * 1.4 + 0.5, 0.06, 0.4))
	lid.commit(m)
	m.surface_set_material(1, _mat(Color(0.12, 0.12, 0.12), 0.8))
	return m

func _shelter() -> Mesh:
	# bus shelter facing -Z (the road): posts, roof, glass back and sides, bench
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	for x in [-1.95, 1.95]:
		for z in [-0.75, 0.75]:
			_pbox(st, Vector3(0.08, 2.5, 0.08), _at(x, 1.25, z))
	_pbox(st, Vector3(4.3, 0.12, 1.8), _at(0, 2.55, 0))
	_pbox(st, Vector3(2.6, 0.06, 0.42), _at(0, 0.46, 0.45))
	_pbox(st, Vector3(0.06, 0.44, 0.3), _at(-1.1, 0.22, 0.45))
	_pbox(st, Vector3(0.06, 0.44, 0.3), _at(1.1, 0.22, 0.45))
	_pbox(st, Vector3(0.9, 0.5, 0.04), _at(1.3, 2.2, -0.76))   # route sign
	var m := st.commit()
	m.surface_set_material(0, _mat(Color(0.3, 0.38, 0.45), 0.5, 0.6))
	var g := SurfaceTool.new()
	g.begin(Mesh.PRIMITIVE_TRIANGLES)
	_pbox(g, Vector3(3.8, 2.0, 0.02), _at(0, 1.3, 0.75))
	_pbox(g, Vector3(0.02, 2.0, 1.3), _at(-1.95, 1.3, 0.05))
	_pbox(g, Vector3(0.02, 2.0, 1.3), _at(1.95, 1.3, 0.05))
	g.commit(m)
	var gm := _mat(Color(0.7, 0.8, 0.85, 0.35), 0.05, 0.2)
	gm.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	gm.cull_mode = BaseMaterial3D.CULL_DISABLED
	m.surface_set_material(1, gm)
	return m

func _swing() -> Mesh:
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	for x in [-1.4, 1.4]:
		_pbox(st, Vector3(0.08, 2.4, 0.08), Transform3D(Basis(Vector3.RIGHT, 0.32), Vector3(x, 1.15, -0.38)))
		_pbox(st, Vector3(0.08, 2.4, 0.08), Transform3D(Basis(Vector3.RIGHT, -0.32), Vector3(x, 1.15, 0.38)))
	_pbox(st, Vector3(2.9, 0.09, 0.09), _at(0, 2.25, 0))
	var m := st.commit()
	m.surface_set_material(0, _mat(Color(0.75, 0.2, 0.15), 0.6, 0.3))
	var s2 := SurfaceTool.new()
	s2.begin(Mesh.PRIMITIVE_TRIANGLES)
	for x in [-0.6, 0.6]:
		_pbox(s2, Vector3(0.02, 1.7, 0.02), _at(x - 0.2, 1.4, 0))
		_pbox(s2, Vector3(0.02, 1.7, 0.02), _at(x + 0.2, 1.4, 0))
		_pbox(s2, Vector3(0.5, 0.05, 0.22), _at(x, 0.55, 0))
	s2.commit(m)
	m.surface_set_material(1, _mat(Color(0.95, 0.75, 0.15), 0.6))
	return m

func _slide() -> Mesh:
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	for x in [-0.6, 0.6]:
		for z in [-0.6, 0.6]:
			_pbox(st, Vector3(0.1, 2.6, 0.1), _at(x, 1.3, z))
	_pbox(st, Vector3(1.3, 0.08, 1.3), _at(0, 1.4, 0))
	_pbox(st, Vector3(1.4, 0.5, 1.4), Transform3D(Basis(Vector3.UP, PI * 0.25).scaled(Vector3(1, 1, 1)), Vector3(0, 2.75, 0)))
	var m := st.commit()
	m.surface_set_material(0, _mat(Color(0.2, 0.45, 0.75), 0.6, 0.2))
	var s2 := SurfaceTool.new()
	s2.begin(Mesh.PRIMITIVE_TRIANGLES)
	_pbox(s2, Vector3(0.6, 0.06, 2.6), Transform3D(Basis(Vector3.RIGHT, -0.52), Vector3(0, 0.75, -1.65)))
	for k in 5:
		_pbox(s2, Vector3(0.6, 0.05, 0.2), _at(0, 0.28 * (k + 1), 0.7 + 0.25 * k * 0.0 + 0.1))
	s2.commit(m)
	m.surface_set_material(1, _mat(Color(0.95, 0.75, 0.15), 0.4, 0.3))
	return m

func _sandbox() -> Mesh:
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	_pbox(st, Vector3(2.6, 0.3, 0.12), _at(0, 0.15, -1.25))
	_pbox(st, Vector3(2.6, 0.3, 0.12), _at(0, 0.15, 1.25))
	_pbox(st, Vector3(0.12, 0.3, 2.6), _at(-1.25, 0.15, 0))
	_pbox(st, Vector3(0.12, 0.3, 2.6), _at(1.25, 0.15, 0))
	_pbox(st, Vector3(0.1, 2.2, 0.1), _at(-1.2, 1.1, -1.2))
	_pbox(st, Vector3(0.1, 2.2, 0.1), _at(1.2, 1.1, 1.2))
	_pbox(st, Vector3(0.1, 2.2, 0.1), _at(-1.2, 1.1, 1.2))
	_pbox(st, Vector3(0.1, 2.2, 0.1), _at(1.2, 1.1, -1.2))
	var m := st.commit()
	m.surface_set_material(0, _mat(Color(0.55, 0.38, 0.22), 0.8))
	var s2 := SurfaceTool.new()
	s2.begin(Mesh.PRIMITIVE_TRIANGLES)
	_pbox(s2, Vector3(2.4, 0.06, 2.4), _at(0, 0.2, 0))
	s2.commit(m)
	m.surface_set_material(1, _mat(Color(0.85, 0.76, 0.55), 1.0))
	var s3 := SurfaceTool.new()
	s3.begin(Mesh.PRIMITIVE_TRIANGLES)
	_pbox(s3, Vector3(1.8, 0.08, 1.8), Transform3D(Basis(Vector3.UP, PI * 0.25), Vector3(0, 2.25, 0)))
	s3.commit(m)
	m.surface_set_material(2, _mat(Color(0.2, 0.6, 0.3), 0.6))
	return m

func _palm() -> Mesh:
	var trunk := SurfaceTool.new()
	trunk.begin(Mesh.PRIMITIVE_TRIANGLES)
	var base := Vector3.ZERO
	for k in 6:
		var top := base + Vector3(0.12 * sin(k * 0.6), 1.45, 0.05 * k)
		_cyl(trunk, base, 0.2 - k * 0.015, 0.19 - k * 0.015, 1.5, 7)
		base = top
	var m := trunk.commit()
	m.surface_set_material(0, _mat(Color(0.42, 0.36, 0.28), 0.95))
	var fr := SurfaceTool.new()
	fr.begin(Mesh.PRIMITIVE_TRIANGLES)
	for k in 9:
		var a := TAU * k / 9.0
		var dir := Vector3(cos(a), 0, sin(a))
		var b := Basis(Vector3.UP, -a) * Basis(Vector3.FORWARD, 0.35)
		_pbox(fr, Vector3(2.6, 0.04, 0.55), Transform3D(b, base + dir * 1.2 + Vector3(0, -0.25, 0)))
	fr.commit(m)
	var leaf := _mat(Color(0.2, 0.38, 0.14), 0.8)
	leaf.cull_mode = BaseMaterial3D.CULL_DISABLED
	m.surface_set_material(1, leaf)
	return m
