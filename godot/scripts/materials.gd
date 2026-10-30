# Material library: one material per core material id (see native/src/core/mesh.hpp), built from the core's
# procedural textures. Shared by every chunk.
extends RefCounted
class_name EarthMaterials

const M_TERRAIN := 0
const M_ASPHALT := 1
const M_PAVERS := 2
const M_CURB := 3
const M_MARKING := 4
const M_WATER := 8
const M_GRAVEL := 10
const M_ROOF_FLAT := 20
const M_ROOF_TILE := 21
const M_ROOF_METAL := 22
const M_CONCRETE := 23
const M_PAINT := 24
const M_GLASS := 25
const M_METAL := 26
const M_GOLD := 27
const M_DARK := 28
const M_LAMP := 29
const M_BARK := 30
const M_LEAVES := 31
const M_CONIFER := 32
const M_CARPAINT := 33
const M_TIRE := 34
const M_SIGNAL := 35
const M_SCREEN := 36
const M_POSTER := 37
const M_FACADE := 100
const FACADE_STYLES := 20

# texture kinds (core/textures.hpp)
const T_GRASS := 0
const T_ASPHALT := 3
const T_PAVERS := 4
const T_CONCRETE := 8
const T_CURB := 9
const T_ROOF_TILE := 10
const T_ROOF_METAL := 11
const T_ROOF_FLAT := 12
const T_BARK := 13
const T_LEAVES := 14
const T_CONIFER := 15
const T_GRAVEL := 16

var mats := {}
var facade_mats: Array[ORMMaterial3D] = []
var terrain_shader: Shader
var layer_albedo: Texture2DArray
var layer_normal: Texture2DArray
var layer_orm: Texture2DArray
var tex_cache := {}
var night := 0.0
var grass_mat: ShaderMaterial
var grass_mesh: ArrayMesh

func _img(bytes: PackedByteArray, size: int, fmt := Image.FORMAT_RGBA8, mips := true) -> Image:
	var img := Image.create_from_data(size, size, false, fmt, bytes)
	if mips:
		img.generate_mipmaps()
	return img

func _tex(kind: int) -> Dictionary:
	if tex_cache.has(kind):
		return tex_cache[kind]
	var d: Dictionary = EarthCore.material_texture(kind)
	var s: int = d["size"]
	var r := {
		"albedo": ImageTexture.create_from_image(_img(d["albedo"], s)),
		"normal": ImageTexture.create_from_image(_img(d["normal"], s)),
		"orm": ImageTexture.create_from_image(_img(d["orm"], s)),
		"img": d,
	}
	tex_cache[kind] = r
	return r

func _orm(kind: int, triplanar_m := 0.0, tint_by_vertex := true) -> ORMMaterial3D:
	var t := _tex(kind)
	var m := ORMMaterial3D.new()
	m.albedo_texture = t["albedo"]
	m.normal_enabled = true
	m.normal_texture = t["normal"]
	m.orm_texture = t["orm"]
	m.vertex_color_use_as_albedo = tint_by_vertex
	m.texture_filter = BaseMaterial3D.TEXTURE_FILTER_LINEAR_WITH_MIPMAPS_ANISOTROPIC
	m.metallic_specular = 0.3
	if triplanar_m > 0.0:
		m.uv1_triplanar = true
		m.uv1_world_triplanar = true
		m.uv1_scale = Vector3.ONE / triplanar_m
		m.uv1_triplanar_sharpness = 4.0
	return m

func _plain(c: Color, rough: float, metal := 0.0, tint := true) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.albedo_color = c
	m.roughness = rough
	m.metallic = metal
	m.vertex_color_use_as_albedo = tint
	return m

func build() -> void:
	var t0 := Time.get_ticks_msec()
	mats[M_ASPHALT] = _orm(T_ASPHALT)
	mats[M_ASPHALT].uv1_scale = Vector3(1, 1, 1)
	mats[M_PAVERS] = _orm(T_PAVERS)
	mats[M_PAVERS].metallic_specular = 0.15
	mats[M_CURB] = _orm(T_CURB, 1.5)
	var mk := _plain(Color(0.93, 0.93, 0.9), 0.55)
	mk.normal_enabled = true
	mk.normal_texture = _tex(T_ASPHALT)["normal"]
	mk.uv1_triplanar = true
	mk.uv1_world_triplanar = true
	mk.uv1_scale = Vector3.ONE / 4.0
	mats[4] = mk
	mats[M_GRAVEL] = _orm(T_GRAVEL)
	mats[5] = _orm(T_GRASS, 3.0)   # grass on kerbed islands and medians
	mats[M_ROOF_FLAT] = _orm(T_ROOF_FLAT, 4.0)
	mats[M_ROOF_TILE] = _orm(T_ROOF_TILE)
	mats[M_ROOF_METAL] = _orm(T_ROOF_METAL)
	mats[M_CONCRETE] = _orm(T_CONCRETE, 3.0)
	mats[M_PAINT] = _plain(Color.WHITE, 0.5)
	var glass := _plain(Color(0.12, 0.15, 0.18), 0.04, 0.3)
	glass.metallic_specular = 0.9
	mats[M_GLASS] = glass
	mats[M_METAL] = _plain(Color.WHITE, 0.4, 0.7)
	mats[M_GOLD] = _plain(Color(1.0, 0.78, 0.35), 0.18, 1.0, false)
	mats[M_DARK] = _plain(Color(0.07, 0.07, 0.08), 0.35)
	var lamp := _plain(Color(1, 0.95, 0.85), 0.3, 0.0, false)
	lamp.emission_enabled = true
	lamp.emission = Color(1.0, 0.82, 0.55)
	lamp.emission_energy_multiplier = 0.0
	mats[M_LAMP] = lamp
	mats[M_BARK] = _orm(T_BARK)
	for k in [M_LEAVES, M_CONIFER]:
		var lt := _tex(T_LEAVES if k == M_LEAVES else T_CONIFER)
		var lm := StandardMaterial3D.new()
		lm.albedo_texture = lt["albedo"]
		lm.normal_enabled = true
		lm.normal_texture = lt["normal"]
		lm.roughness = 0.75
		lm.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA_SCISSOR
		lm.alpha_scissor_threshold = 0.45
		lm.alpha_antialiasing_mode = BaseMaterial3D.ALPHA_ANTIALIASING_ALPHA_TO_COVERAGE
		lm.backlight_enabled = true
		lm.backlight = Color(0.35, 0.45, 0.15)
		lm.texture_filter = BaseMaterial3D.TEXTURE_FILTER_LINEAR_WITH_MIPMAPS_ANISOTROPIC
		mats[k] = lm
	var paint := _plain(Color.WHITE, 0.25, 0.4)
	paint.clearcoat_enabled = true
	paint.clearcoat = 0.8
	paint.clearcoat_roughness = 0.1
	mats[M_CARPAINT] = paint
	mats[M_TIRE] = _plain(Color(0.06, 0.06, 0.06), 0.85, 0.0, false)
	mats[M_WATER] = _water()
	var sig := ShaderMaterial.new()
	sig.shader = load("res://shaders/signal.gdshader")
	mats[M_SIGNAL] = sig
	var scr := ShaderMaterial.new()
	scr.shader = load("res://shaders/screen.gdshader")
	mats[M_SCREEN] = scr
	var pos := ShaderMaterial.new()
	pos.shader = scr.shader
	pos.set_shader_parameter("still", 1.0)
	pos.set_shader_parameter("brightness", 0.9)
	mats[M_POSTER] = pos
	# facades: 13 styles x 3 layouts
	for s in FACADE_STYLES:
		for v in 3:
			var d: Dictionary = EarthCore.facade_texture(s, v)
			var sz: int = d["size"]
			var m := ORMMaterial3D.new()
			m.albedo_texture = ImageTexture.create_from_image(_img(d["albedo"], sz))
			m.vertex_color_use_as_albedo = true
			m.normal_enabled = true
			m.normal_texture = ImageTexture.create_from_image(_img(d["normal"], sz))
			m.orm_texture = ImageTexture.create_from_image(_img(d["orm"], sz))
			m.emission_enabled = true
			m.emission_texture = ImageTexture.create_from_image(_img(d["emission"], sz))
			m.emission = Color.WHITE
			m.emission_energy_multiplier = 0.0
			m.heightmap_enabled = true
			m.heightmap_texture = ImageTexture.create_from_image(_img(d["depth"], sz, Image.FORMAT_R8))
			m.heightmap_scale = 2.0
			m.heightmap_deep_parallax = true
			m.heightmap_min_layers = 4
			m.heightmap_max_layers = 16
			m.texture_filter = BaseMaterial3D.TEXTURE_FILTER_LINEAR_WITH_MIPMAPS_ANISOTROPIC
			mats[M_FACADE + s * 3 + v] = m
			facade_mats.append(m)
	_terrain_layers()
	_grass()
	print("materials built in ", Time.get_ticks_msec() - t0, " ms")

func _grass() -> void:
	grass_mat = ShaderMaterial.new()
	grass_mat.shader = load("res://shaders/grass.gdshader")
	var d: Dictionary = EarthCore.material_texture(18)
	var img := Image.create_from_data(d["size"], d["size"], false, Image.FORMAT_RGBA8, d["albedo"])
	img.generate_mipmaps()
	grass_mat.set_shader_parameter("blades", ImageTexture.create_from_image(img))
	# three crossed cards, 0.7 m wide, 0.45 m high
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	for k in 3:
		var a := k * PI / 3.0
		var dx := Vector3(cos(a), 0, sin(a)) * 0.35
		var nrm := Vector3(-sin(a), 0, cos(a))
		var p0 := -dx
		var p1 := dx
		var p2 := dx + Vector3(0, 0.45, 0)
		var p3 := -dx + Vector3(0, 0.45, 0)
		for v in [[p0, Vector2(0, 1)], [p1, Vector2(1, 1)], [p2, Vector2(1, 0)], [p0, Vector2(0, 1)], [p2, Vector2(1, 0)], [p3, Vector2(0, 0)]]:
			st.set_normal(nrm)
			st.set_uv(v[1])
			st.set_color(Color.WHITE)
			st.add_vertex(v[0])
	grass_mesh = st.commit()
	grass_mesh.surface_set_material(0, grass_mat)

func _water() -> ShaderMaterial:
	var sm := ShaderMaterial.new()
	sm.shader = load("res://shaders/water.gdshader")
	for i in 2:
		var n := FastNoiseLite.new()
		n.seed = 7 + i
		n.frequency = 0.02 if i == 0 else 0.05
		n.fractal_octaves = 4
		var nt := NoiseTexture2D.new()
		nt.width = 512
		nt.height = 512
		nt.seamless = true
		nt.as_normal_map = true
		nt.bump_strength = 6.0
		nt.noise = n
		sm.set_shader_parameter("wave%d" % (i + 1), nt)
	return sm

func _terrain_layers() -> void:
	var alb: Array[Image] = []
	var nrm: Array[Image] = []
	var orm: Array[Image] = []
	for k in 8:
		var d: Dictionary = _tex(k)["img"]
		var s: int = d["size"]
		alb.append(_img(d["albedo"], s))
		nrm.append(_img(d["normal"], s))
		orm.append(_img(d["orm"], s))
	layer_albedo = Texture2DArray.new()
	layer_albedo.create_from_images(alb)
	layer_normal = Texture2DArray.new()
	layer_normal.create_from_images(nrm)
	layer_orm = Texture2DArray.new()
	layer_orm.create_from_images(orm)
	terrain_shader = load("res://shaders/terrain.gdshader")

func terrain_material(splat0: PackedByteArray, splat1: PackedByteArray, n: int) -> ShaderMaterial:
	var m := ShaderMaterial.new()
	m.shader = terrain_shader
	m.set_shader_parameter("splat0", ImageTexture.create_from_image(Image.create_from_data(n, n, false, Image.FORMAT_RGBA8, splat0)))
	m.set_shader_parameter("splat1", ImageTexture.create_from_image(Image.create_from_data(n, n, false, Image.FORMAT_RGBA8, splat1)))
	m.set_shader_parameter("layer_albedo", layer_albedo)
	m.set_shader_parameter("layer_normal", layer_normal)
	m.set_shader_parameter("layer_orm", layer_orm)
	return m

func get_mat(id: int) -> Material:
	if mats.has(id):
		return mats[id]
	return mats.get(M_CONCRETE)

# rain: paving and concrete darken and turn glossy (0 dry .. 1 soaked)
func set_wet(w: float) -> void:
	for id in [M_ASPHALT, M_PAVERS, M_CURB, M_CONCRETE, 4, M_ROOF_FLAT, M_ROOF_TILE, M_ROOF_METAL]:
		if not mats.has(id):
			continue
		var m: BaseMaterial3D = mats[id]
		if not m.has_meta("dry_rough"):
			m.set_meta("dry_rough", m.roughness)
			m.set_meta("dry_col", m.albedo_color)
		m.roughness = lerpf(m.get_meta("dry_rough"), 0.18, w)
		m.albedo_color = (m.get_meta("dry_col") as Color) * lerpf(1.0, 0.7, w)

# 0 = day, 1 = night: lit windows and lamps
func set_night(k: float) -> void:
	night = k
	for m in facade_mats:
		m.emission_energy_multiplier = k * 0.8
	mats[M_LAMP].emission_energy_multiplier = k * 8.0
