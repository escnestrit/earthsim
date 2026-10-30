extends Node3D
# test only: terrain material at grazing angle, all pavers / all grass halves
func _ready():
	var m := EarthMaterials.new()
	m.build()
	var we := WorldEnvironment.new()
	var env := Environment.new()
	var sky := Sky.new()
	sky.sky_material = ProceduralSkyMaterial.new()
	env.background_mode = Environment.BG_SKY
	env.sky = sky
	env.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	we.environment = env
	add_child(we)
	var sun := DirectionalLight3D.new()
	sun.rotation = Vector3(-0.8, 0.7, 0)
	sun.shadow_enabled = OS.get_environment("TV_SHADOW") == "1"
	sun.directional_shadow_mode = DirectionalLight3D.SHADOW_PARALLEL_4_SPLITS
	add_child(sun)
	var n := 256
	var s0 := PackedByteArray(); s0.resize(n * n * 4)
	var s1 := PackedByteArray(); s1.resize(n * n * 4)
	for j in n:
		for i in n:
			var q := (j * n + i) * 4
			if i < n / 2: s1[q] = 255
			else: s0[q] = 255
	var mode := OS.get_environment("TV_MODE")
	var mat: Material = m.terrain_material(s0, s1, n)
	if mode == "pavers": mat = m.get_mat(2)
	var pm := PlaneMesh.new()
	pm.size = Vector2(256, 256)
	pm.subdivide_width = 64
	pm.subdivide_depth = 64
	var mi := MeshInstance3D.new()
	mi.mesh = pm
	mi.material_override = mat
	add_child(mi)
	var cam := Camera3D.new()
	cam.position = Vector3(0, 1.7, 100)
	add_child(cam)
	cam.look_at(Vector3(0, 0, 0))
	await get_tree().create_timer(1.5).timeout
	await RenderingServer.frame_post_draw
	get_viewport().get_texture().get_image().save_png("/tmp/terrain_%s.png" % mode)
	get_tree().quit()
