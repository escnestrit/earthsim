extends Node3D
# test only: pedestrians with the walk shader at several phases -> /tmp/people.png
func _ready():
	var we := WorldEnvironment.new()
	var env := Environment.new()
	env.background_mode = Environment.BG_COLOR
	env.background_color = Color(0.6, 0.7, 0.85)
	env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	env.ambient_light_color = Color(0.5, 0.5, 0.55)
	we.environment = env
	add_child(we)
	var sun := DirectionalLight3D.new()
	sun.rotation = Vector3(-0.8, 0.7, 0)
	add_child(sun)
	var g := MeshInstance3D.new()
	var pm := PlaneMesh.new()
	pm.size = Vector2(30, 30)
	g.mesh = pm
	add_child(g)
	var d: Dictionary = EarthCore.model(8, 0)
	var am := ArrayMesh.new()
	var sm := ShaderMaterial.new()
	sm.shader = load("res://shaders/person.gdshader")
	for mat in d:
		am.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, d[mat])
		am.surface_set_material(am.get_surface_count() - 1, sm)
	var mmi := MultiMeshInstance3D.new()
	var mm := MultiMesh.new()
	mm.transform_format = MultiMesh.TRANSFORM_3D
	mm.use_custom_data = true
	mm.mesh = am
	mm.instance_count = 5
	var cols := [Color(0.15, 0.2, 0.45), Color(0.6, 0.1, 0.1), Color(0.85, 0.85, 0.82), Color(0.3, 0.45, 0.25), Color(0.7, 0.55, 0.15)]
	for i in 5:
		var b := Basis(Vector3.UP, 0.3 * i - 0.6)
		mm.set_instance_transform(i, Transform3D(b, Vector3(i * 1.2 - 2.4, 0, 0)))
		var c: Color = cols[i]
		c.a = i * 1.2
		mm.set_instance_custom_data(i, c)
	mmi.multimesh = mm
	add_child(mmi)
	var cam := Camera3D.new()
	cam.position = Vector3(0.5, 1.4, 5.0)
	cam.look_at(Vector3(0, 0.9, 0))
	add_child(cam)
	await get_tree().process_frame
	await get_tree().process_frame
	await get_tree().process_frame
	get_viewport().get_texture().get_image().save_png("/tmp/people.png")
	get_tree().quit()
