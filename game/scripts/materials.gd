## Materials for the world mesh surfaces, indexed by the core's material ids (es::Mat).
extends RefCounted

const M_ASPHALT := 0
const M_JUNCTION := 1
const M_PAVEMENT := 2
const M_KERB := 3
const M_CONCRETE := 4
const M_TERRAIN := 5
const M_WALL := 6
const M_ROOF := 7
const M_WATER := 8
const M_PATH := 9
const M_RAIL := 10
const M_MARKING := 11
const COUNT := 12

var list: Array[Material] = []
var far_list: Array[Material] = []   # same shaders, cut away inside the detail radius

func _init() -> void:
	list.resize(COUNT)
	var road := load("res://shaders/road.gdshader") as Shader
	var surf := load("res://shaders/surface.gdshader") as Shader
	var a := ShaderMaterial.new()
	a.shader = road
	list[M_ASPHALT] = a
	var j := ShaderMaterial.new()
	j.shader = road
	j.set_shader_parameter("junction", true)
	list[M_JUNCTION] = j
	for pair in [[M_PAVEMENT, 0], [M_KERB, 1], [M_CONCRETE, 2], [M_PATH, 3], [M_RAIL, 4]]:
		var m := ShaderMaterial.new()
		m.shader = surf
		m.set_shader_parameter("kind", pair[1])
		list[pair[0]] = m
	for pair in [[M_TERRAIN, "terrain"], [M_WALL, "facade"], [M_ROOF, "roof"], [M_WATER, "water"], [M_MARKING, "marking"]]:
		var m := ShaderMaterial.new()
		m.shader = load("res://shaders/%s.gdshader" % pair[1])
		list[pair[0]] = m
	far_list = list.duplicate()
	for k in [M_TERRAIN, M_WALL, M_ROOF, M_WATER]:
		var fm: ShaderMaterial = list[k].duplicate()
		fm.set_shader_parameter("far_lod", true)
		far_list[k] = fm
