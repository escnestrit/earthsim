extends Node3D
# test only: renders the procedural models (car, trees, lamp) to /tmp/models.png
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
	env.tonemap_mode = Environment.TONE_MAPPER_FILMIC
	we.environment = env
	add_child(we)
	var sun := DirectionalLight3D.new()
	sun.rotation = Vector3(-0.8, 0.7, 0)
	sun.shadow_enabled = true
	add_child(sun)
	var ground := MeshInstance3D.new()
	var pm := PlaneMesh.new()
	pm.size = Vector2(60, 60)
	ground.mesh = pm
	ground.material_override = m.get_mat(1)
	add_child(ground)
	var items := [[4, 0, Vector3(0, 0, 0)], [0, 1, Vector3(-9, 0, -6)], [1, 1, Vector3(8, 0, -8)], [3, 0, Vector3(4, 0, -3)], [2, 0, Vector3(-4, 0, -3)]]
	for it in items:
		var d: Dictionary = EarthCore.model(it[0], it[1])
		var am := ArrayMesh.new()
		for mat in d:
			am.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, d[mat])
			var mm: Material = m.get_mat(mat)
			if mat == 33:
				var p := (mm as StandardMaterial3D).duplicate() as StandardMaterial3D
				p.vertex_color_use_as_albedo = false
				p.albedo_color = Color(0.1, 0.25, 0.55)
				mm = p
			am.surface_set_material(am.get_surface_count() - 1, mm)
		var mi := MeshInstance3D.new()
		mi.mesh = am
		mi.position = it[2]
		mi.rotation.y = 0.5
		add_child(mi)
	var cam := Camera3D.new()
	cam.position = Vector3(3.5, 2.6, 6.5)
	add_child(cam)
	cam.look_at(Vector3(0, 0.8, 0))
	await get_tree().create_timer(1.5).timeout
	await RenderingServer.frame_post_draw
	get_viewport().get_texture().get_image().save_png("/tmp/models.png")
	get_tree().quit()
