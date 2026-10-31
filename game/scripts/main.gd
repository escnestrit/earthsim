## Entry point: environment, world streaming, the car, camera, HUD, input and automated
## test/screenshot modes (used by CI and by the developer on a headless machine).
##
## Command line (after `--`):
##   --data=DIR            data folder (default: next to the exe, or ../data_local in the editor)
##   --lat=.. --lon=..     start near a coordinate
##   --time=HOUR           time of day, 0..24
##   --rain                wet roads
##   --radius=M            streaming radius
##   --shot=FILE.png       render once everything around is loaded, save, quit
##   --view=x,y,z,yaw,pitch  free camera for --shot (Godot coordinates, degrees)
##   --test=drive          scripted driving test, prints a report, exit code 0/1
extends Node3D

const StreamerScript := preload("res://scripts/streamer.gd")
const CarScript := preload("res://scripts/car.gd")
const CarModel := preload("res://scripts/car_model.gd")
const CameraScript := preload("res://scripts/camera_rig.gd")
const AudioScript := preload("res://scripts/car_audio.gd")
const MenuScript := preload("res://scripts/menu.gd")
const ControlsScript := preload("res://scripts/controls.gd")

var args := {}
var world: EarthWorld
var streamer
var car: RigidBody3D
var cam: Camera3D
var sun: DirectionalLight3D
var env: Environment
var sky_mat: ProceduralSkyMaterial
var hud: Label
var help: Label
var hour := 13.0
var spawned := false
var reverse_hold := 0.0
var _frames_loaded := 0
var _test_t := 0.0
var _test := {}
var _wheel_nodes := []
var _head_light: SpotLight3D
var _lights_on := false
var menu
var controls
var monitor: Label
var _lamp_lights: Array[OmniLight3D] = []
var _lamp_t := 0.0
var _tail_mat: StandardMaterial3D
var _body_parts := []
var _wheel_rest := []
var _dent_total := 0.0
var _night := 0.0

func _ready() -> void:
	for a in OS.get_cmdline_user_args():
		var s: String = a.trim_prefix("--")
		var i := s.find("=")
		if i > 0:
			args[s.substr(0, i)] = s.substr(i + 1)
		else:
			args[s] = "1"
	hour = float(args.get("time", "13"))
	_setup_environment()
	_setup_world()
	if world == null:
		return
	_setup_car()
	_setup_hud()
	_setup_menu()
	controls = ControlsScript.new()
	controls.load_config()
	_setup_lamp_lights()
	if not args.has("test") and not args.has("shot") and DisplayServer.get_name() != "headless":
		var au := AudioStreamPlayer.new()
		au.set_script(AudioScript)
		au.car = car
		add_child(au)
	set_time(hour)
	if args.has("rain"):
		set_wet(1.0)

func _find_data() -> String:
	var c := []
	if args.has("data"):
		c.append(args["data"])
	if OS.has_environment("EARTHSIM_DATA"):
		c.append(OS.get_environment("EARTHSIM_DATA"))
	c.append(OS.get_executable_path().get_base_dir().path_join("data"))
	c.append(ProjectSettings.globalize_path("res://").path_join("../data"))
	c.append(ProjectSettings.globalize_path("res://").path_join("../data_local"))
	for d in c:
		if FileAccess.file_exists(String(d).path_join("region.json")):
			return d
	return ""

func _setup_world() -> void:
	var dir := _find_data()
	if dir == "":
		_fatal("Не найдены данные карты (папка data с region.json рядом с exe).")
		return
	world = EarthWorld.new()
	var t := Time.get_ticks_msec()
	if not world.load(dir):
		_fatal("Ошибка загрузки данных:\n" + "\n".join(world.get_log()))
		world = null
		return
	print("[earthsim] world loaded in %d ms from %s" % [Time.get_ticks_msec() - t, dir])
	for l in world.get_log():
		print("  ", l)
	streamer = StreamerScript.new()
	add_child(streamer)
	streamer.setup(world)
	if args.has("radius"):
		streamer.radius = float(args["radius"])
	if args.has("shot") and not args.has("far"):
		streamer.far_enabled = false
	var focus := Vector3.ZERO
	if args.has("lat") and args.has("lon"):
		focus = world.latlon_to_world(float(args["lat"]), float(args["lon"]))
	focus.y = world.ground_height(focus)
	streamer.focus = focus

func _fatal(msg: String) -> void:
	push_error(msg)
	var l := Label.new()
	l.text = msg
	l.position = Vector2(40, 40)
	l.add_theme_font_size_override("font_size", 22)
	add_child(l)
	if args.has("test") or args.has("shot"):
		get_tree().quit(2)

func _setup_environment() -> void:
	env = Environment.new()
	env.background_mode = Environment.BG_SKY
	var sky := Sky.new()
	sky_mat = ProceduralSkyMaterial.new()
	sky_mat.sky_top_color = Color(0.32, 0.5, 0.78)
	sky_mat.sky_horizon_color = Color(0.68, 0.76, 0.84)
	sky_mat.ground_horizon_color = Color(0.6, 0.65, 0.7)
	sky_mat.ground_bottom_color = Color(0.25, 0.26, 0.28)
	sky.sky_material = sky_mat
	env.sky = sky
	env.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	env.reflected_light_source = Environment.REFLECTION_SOURCE_SKY
	env.tonemap_mode = Environment.TONE_MAPPER_FILMIC
	env.tonemap_white = 6.0
	env.ssao_enabled = true
	env.ssao_radius = 1.2
	env.glow_enabled = true
	env.glow_bloom = 0.05
	env.fog_enabled = true
	env.fog_mode = Environment.FOG_MODE_DEPTH
	env.fog_depth_begin = 300.0
	env.fog_depth_end = 7000.0
	env.fog_depth_curve = 0.6
	env.fog_density = 0.85
	env.fog_light_color = Color(0.7, 0.77, 0.85)
	env.fog_sky_affect = 0.0
	var we := WorldEnvironment.new()
	we.environment = env
	add_child(we)
	sun = DirectionalLight3D.new()
	sun.shadow_enabled = true
	sun.directional_shadow_max_distance = 220.0
	sun.directional_shadow_mode = DirectionalLight3D.SHADOW_PARALLEL_4_SPLITS
	sun.light_angular_distance = 0.5
	add_child(sun)

var wet := 0.0

func set_wet(w: float) -> void:
	wet = w
	RenderingServer.global_shader_parameter_set("wetness", w)
	if car:
		car.wetness = w

func set_time(h: float) -> void:
	hour = fposmod(h, 24.0)
	var day := (hour - 6.0) / 12.0  # 0 sunrise .. 1 sunset
	var elev := sin(day * PI) * deg_to_rad(52.0)
	var n := 1.0 - smoothstep(-0.10, 0.10, elev)
	if day <= 0.0 or day >= 1.0:
		n = 1.0
		elev = -0.2
	var az := lerpf(deg_to_rad(-100.0), deg_to_rad(100.0), clampf(day, 0.0, 1.0))
	RenderingServer.global_shader_parameter_set("night", n)
	_night = n
	var warm := (1.0 - smoothstep(0.0, 0.35, elev)) * (1.0 - n)
	if n < 0.5:
		sun.rotation = Vector3(-maxf(elev, deg_to_rad(3.0)), az + PI, 0)
		sun.light_color = Color(1.0, 0.93, 0.85).lerp(Color(1.0, 0.6, 0.35), warm)
		sun.light_energy = lerpf(1.6, 0.3, n * 2.0)
	else:
		# moonlight
		sun.rotation = Vector3(deg_to_rad(-38.0), deg_to_rad(40.0), 0)
		sun.light_color = Color(0.62, 0.72, 1.0)
		sun.light_energy = 0.16
	var day_top := Color(0.32, 0.5, 0.78)
	var day_hor := Color(0.68, 0.76, 0.84)
	# city sky at night is not black: light pollution on the horizon
	var night_top := Color(0.03, 0.04, 0.08)
	var night_hor := Color(0.17, 0.14, 0.13)
	sky_mat.sky_top_color = day_top.lerp(night_top, n)
	sky_mat.sky_horizon_color = day_hor.lerp(night_hor, n).lerp(Color(0.95, 0.6, 0.4), warm * 0.6)
	sky_mat.ground_horizon_color = sky_mat.sky_horizon_color
	sky_mat.ground_bottom_color = Color(0.25, 0.26, 0.28).lerp(Color(0.04, 0.04, 0.05), n)
	env.fog_light_color = sky_mat.sky_horizon_color
	env.ambient_light_energy = lerpf(1.0, 0.9, n)
	env.background_energy_multiplier = 1.0
	env.tonemap_exposure = lerpf(1.0, 1.7, n)
	if streamer:
		streamer.props.set_night(n)
	if _head_light:
		_head_light.visible = n > 0.3

const CAR_MODEL := "res://models/car_concept.glb"

func _setup_car() -> void:
	car = RigidBody3D.new()
	car.set_script(CarScript)
	car.freeze = true
	var model_scene: PackedScene = load(CAR_MODEL) if ResourceLoader.exists(CAR_MODEL) else null
	var model_root: Node3D = null
	var wheel_nodes := []
	if model_scene:
		# Car Concept (CC BY 4.0, Eric Chadwick / Darmstadt Graphics Group): faces +Z, ground at y=0.
		model_root = model_scene.instantiate()
		var names := ["WheelFrontL", "WheelFrontR", "WheelRearL", "WheelRearR"]
		for nm in names:
			wheel_nodes.append(model_root.find_child(nm, true, false))
		if wheel_nodes.has(null):
			model_root = null
	if model_root:
		var wheel_y := 0.384
		var mx := Transform3D(Basis(Vector3.UP, PI), Vector3(0, -wheel_y, 0))
		# wheel centres in car space from the model itself
		var wp := []
		for wn in wheel_nodes:
			wp.append(mx * _model_xform(wn).origin)
		car.wheel_override = wp
		car.wheelbase = absf(wp[2].z - wp[0].z)
		car.track = absf(wp[1].x - wp[0].x)
		car.wheel_radius = 0.40
		car.car_mass = 1650.0
		car.torque_scale = 1.75
		car.com_height = 0.05
		car.rest_len = 0.40
		car.max_len = 0.48
		car.mount_h = 0.26
		car.spring_k = Vector2(36000, 34000)
		car.damp_bump = Vector2(3000, 2900)
		car.damp_rebound = Vector2(4800, 4600)
		car.arb_k = Vector2(18000, 11000)
		car.mu = 1.15
		car.drag_area = 0.62
	add_child(car)
	var lower := CollisionShape3D.new()
	var b1 := BoxShape3D.new()
	b1.size = Vector3(2.1, 0.55, 4.25) if model_root else Vector3(1.78, 0.62, 4.6)
	lower.shape = b1
	lower.position = Vector3(0, 0.12, 0.1) if model_root else Vector3(0, 0.25, 0)
	car.add_child(lower)
	var upper := CollisionShape3D.new()
	var b2 := BoxShape3D.new()
	b2.size = Vector3(1.6, 0.4, 2.0) if model_root else Vector3(1.4, 0.5, 2.3)
	upper.shape = b2
	upper.position = Vector3(0, 0.55, 0.3) if model_root else Vector3(0, 0.85, 0.25)
	car.add_child(upper)
	if model_root:
		var holder := Node3D.new()
		holder.transform = Transform3D(Basis(Vector3.UP, PI), Vector3(0, -0.384, 0))
		car.add_child(holder)
		holder.add_child(model_root)
		# no logos / trademarks: hide the plate and the steering wheel emblem
		for nm in ["License Plate", "InteriorSteeringEmblem"]:
			var n: Node = model_root.find_child(nm, true, false)
			if n:
				n.visible = false
		# wheels become direct children of the car so physics can drive them
		_wheel_rest = []
		for i in 4:
			var wn: Node3D = wheel_nodes[i]
			var g := holder.transform * _model_xform(wn)
			wn.get_parent().remove_child(wn)
			car.add_child(wn)
			wn.transform = g
			_wheel_rest.append(g)
		_wheel_nodes = wheel_nodes
		car.attach_wheel_nodes(wheel_nodes)
		# glass: glTF transmission is not portable across renderers -> our own tinted glass
		var glass := StandardMaterial3D.new()
		glass.albedo_color = Color(0.06, 0.08, 0.1, 0.72)
		glass.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
		glass.metallic = 0.3
		glass.roughness = 0.04
		glass.specular_mode = BaseMaterial3D.SPECULAR_SCHLICK_GGX
		for m in model_root.find_children("*", "MeshInstance3D", true, false):
			var nm := String(m.name)
			if (nm.containsn("window") or nm.containsn("windshield") or nm.containsn("glass")) and not nm.containsn("gasket") and not nm.containsn("wiper"):
				for si in (m as MeshInstance3D).mesh.get_surface_count():
					(m as MeshInstance3D).set_surface_override_material(si, glass)
		# deformable body panels
		_body_parts = []
		for m in model_root.find_children("Body*", "MeshInstance3D", true, false):
			var mi3: MeshInstance3D = m
			if String(mi3.name).containsn("wiper"):
				continue
			var surf := []
			var mats := []
			for si in mi3.mesh.get_surface_count():
				surf.append(mi3.mesh.surface_get_arrays(si))
				mats.append(mi3.get_active_material(si))
			_body_parts.append({"mi": mi3, "xf": holder.transform * _model_xform(mi3), "orig": surf, "cur": surf.duplicate(true), "mats": mats})
		var tl: MeshInstance3D = model_root.find_child("BodyTaillights", true, false)
		if tl:
			var m0 = tl.get_active_material(0)
			if m0 is StandardMaterial3D:
				_tail_mat = m0.duplicate()
				_tail_mat.emission_enabled = true
				_tail_mat.emission = Color(1.0, 0.05, 0.03)
				tl.set_surface_override_material(0, _tail_mat)
		car.impact.connect(_on_impact)
	else:
		var model := CarModel.new()
		var mi := MeshInstance3D.new()
		mi.mesh = model.build_body()
		_tail_mat = model.tail_mat
		car.add_child(mi)
		var surf := []
		var mats := []
		for si in mi.mesh.get_surface_count():
			surf.append(mi.mesh.surface_get_arrays(si))
			mats.append(mi.mesh.surface_get_material(si))
		_body_parts = [{"mi": mi, "xf": Transform3D(), "orig": surf, "cur": surf.duplicate(true), "mats": mats}]
		car.impact.connect(_on_impact)
		var wm: ArrayMesh = model.build_wheel(0.325, 0.215)
		var nodes := []
		for i in 4:
			var w := MeshInstance3D.new()
			w.mesh = wm
			car.add_child(w)
			nodes.append(w)
		_wheel_nodes = nodes
		car.attach_wheel_nodes(nodes)
	_head_light = SpotLight3D.new()
	_head_light.position = Vector3(0, 0.2, -2.35)
	_head_light.spot_range = 70.0
	_head_light.spot_angle = 32.0
	_head_light.light_energy = 9.0
	_head_light.spot_attenuation = 0.8
	_head_light.rotation = Vector3(deg_to_rad(-4.0), 0, 0)
	_head_light.shadow_enabled = true
	_head_light.visible = false
	car.add_child(_head_light)
	cam = Camera3D.new()
	cam.set_script(CameraScript)
	add_child(cam)
	cam.target = car
	cam.current = true
	if args.has("view"):
		var v: PackedStringArray = String(args["view"]).split(",")
		cam.target = null
		cam.position = Vector3(float(v[0]), float(v[1]), float(v[2])) + streamer.focus
		cam.rotation = Vector3(deg_to_rad(float(v[4])), deg_to_rad(float(v[3])), 0)
		cam.far = 4000.0

func _setup_hud() -> void:
	var layer := CanvasLayer.new()
	add_child(layer)
	hud = Label.new()
	hud.position = Vector2(16, 12)
	hud.add_theme_font_size_override("font_size", 18)
	hud.add_theme_color_override("font_outline_color", Color(0, 0, 0, 0.8))
	hud.add_theme_constant_override("outline_size", 5)
	layer.add_child(hud)
	help = Label.new()
	help.position = Vector2(16, 140)
	help.add_theme_font_size_override("font_size", 16)
	help.add_theme_color_override("font_outline_color", Color(0, 0, 0, 0.8))
	help.add_theme_constant_override("outline_size", 5)
	help.text = "W/S газ/тормоз (S на месте — назад) · A/D руль · Пробел ручник · R на дорогу · C камера\nQ/E передачи (ручная) · M авто/ручная · T трекшн · B ABS · N +1 час · F2 дождь · F12 скриншот\nTab / Esc — меню (телепорт, время, дождь, дальность) · F4 руль/геймпад · F1 скрыть подсказку"
	help.visible = not args.has("shot")
	layer.add_child(help)
	monitor = Label.new()
	monitor.position = Vector2(16, 230)
	monitor.add_theme_font_size_override("font_size", 15)
	monitor.add_theme_color_override("font_outline_color", Color(0, 0, 0, 0.9))
	monitor.add_theme_constant_override("outline_size", 5)
	monitor.visible = false
	layer.add_child(monitor)
	if args.has("shot"):
		hud.visible = false

func _setup_menu() -> void:
	menu = CanvasLayer.new()
	menu.set_script(MenuScript)
	add_child(menu)
	menu.teleport.connect(_teleport)
	menu.time_changed.connect(set_time)
	menu.rain_changed.connect(func(on): set_wet(1.0 if on else 0.0))
	menu.radius_changed.connect(func(r): streamer.radius = r)

func _teleport(lat: float, lon: float) -> void:
	var p: Vector3 = world.latlon_to_world(lat, lon)
	p.y = world.ground_height(p)
	spawned = false
	car.ready_to_drive = false
	car.freeze = true
	car.global_position = p + Vector3(0, 2, 0)
	streamer.focus = p

# street lamps light the road at night: a small pool of real lights follows the car to the nearest lamps
func _setup_lamp_lights() -> void:
	for i in 10:
		var l := OmniLight3D.new()
		l.omni_range = 22.0
		l.light_energy = 2.2
		l.light_color = Color(1.0, 0.8, 0.55)
		l.shadow_enabled = false
		l.visible = false
		add_child(l)
		_lamp_lights.append(l)

func _update_lamp_lights(dt: float) -> void:
	_lamp_t -= dt
	if _lamp_t > 0.0:
		return
	_lamp_t = 0.4
	var on := _night > 0.4
	var c: Vector3 = car.global_position if spawned else streamer.focus
	var cand := []
	if on:
		for k in streamer.chunks:
			var node: Node3D = streamer.chunks[k]
			if node.global_position.distance_to(c) > 400.0:
				continue
			var lamps: PackedVector3Array = node.get_meta("lamps", PackedVector3Array())
			for lp in lamps:
				var d := lp.distance_squared_to(c)
				if d < 120.0 * 120.0:
					cand.append([d, lp])
		cand.sort_custom(func(a, b): return a[0] < b[0])
	for i in _lamp_lights.size():
		var l := _lamp_lights[i]
		if i < cand.size():
			l.global_position = cand[i][1] + Vector3(0, 8.0, 0)
			l.visible = true
		else:
			l.visible = false

# Dent the body around a contact point. Displacement falls off smoothly with distance and is
# capped so the shell never folds through itself.
func _on_impact(lp: Vector3, ld: Vector3, impulse: float) -> void:
	var r := clampf(0.3 + impulse / 9000.0, 0.3, 1.1)
	var depth := clampf(impulse / 45000.0, 0.01, 0.28)
	# push inwards: towards the car centre along the contact normal
	var dir := ld
	if dir.dot(-lp.normalized()) < 0.0:
		dir = -dir
	for part in _body_parts:
		var inv: Transform3D = (part["xf"] as Transform3D).affine_inverse()
		var pl: Vector3 = inv * lp
		var dl: Vector3 = (inv.basis * dir).normalized()
		var changed := false
		var cur: Array = part["cur"]
		for si in cur.size():
			var arr: Array = cur[si]
			var v: PackedVector3Array = arr[Mesh.ARRAY_VERTEX]
			var o: PackedVector3Array = part["orig"][si][Mesh.ARRAY_VERTEX]
			var r2 := r * r
			for k in v.size():
				var dd := v[k].distance_squared_to(pl)
				if dd < r2:
					var w := 1.0 - dd / r2
					var nv := v[k] + dl * depth * w * w
					var off := nv - o[k]
					if off.length_squared() > 0.38 * 0.38:
						nv = o[k] + off.normalized() * 0.38
					v[k] = nv
					changed = true
			arr[Mesh.ARRAY_VERTEX] = v
		if changed:
			_rebuild_part(part)
	_dent_total += depth

func _rebuild_part(part: Dictionary) -> void:
	var m := ArrayMesh.new()
	var cur: Array = part["cur"]
	for si in cur.size():
		var arr: Array = (cur[si] as Array).duplicate()
		# imported meshes may carry LOD/skin data we do not keep
		arr.resize(Mesh.ARRAY_MAX)
		m.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)
		m.surface_set_material(si, part["mats"][si])
	(part["mi"] as MeshInstance3D).mesh = m

func _repair() -> void:
	if _dent_total > 0.0:
		for part in _body_parts:
			part["cur"] = (part["orig"] as Array).duplicate(true)
			_rebuild_part(part)
	car.steer_bias = 0.0
	_dent_total = 0.0

static func _model_xform(n: Node) -> Transform3D:
	# transform of n relative to the scene root it was imported in
	var t := Transform3D()
	var c: Node = n
	while c != null and c is Node3D and c.get_parent() != null:
		t = (c as Node3D).transform * t
		c = c.get_parent()
	return t

func _spawn(near: Vector3) -> void:
	var s: Dictionary = world.spawn_near(near)
	if s.is_empty():
		push_error("no spawn point")
		return
	var p: Vector3 = s["pos"]
	car.global_transform = Transform3D(Basis(Vector3.UP, s["yaw"]), p + Vector3(0, 0.45, 0))
	car.linear_velocity = Vector3.ZERO
	car.angular_velocity = Vector3.ZERO
	for w in car.wheels:
		w.omega = 0.0
	car.gear = 1
	car.freeze = false
	car.ready_to_drive = true
	spawned = true
	if not _body_parts.is_empty():
		_repair()
	if cam:
		cam.snap()

func _process(dt: float) -> void:
	if world == null:
		return
	var focus: Vector3 = car.global_position if spawned else streamer.focus
	if args.has("view"):
		focus = cam.global_position
	streamer.focus = focus
	if args.get("test", "") in ["drive", "crash"]:
		if not spawned:
			_make_test_ground()
			if args["test"] == "crash":
				var wall := StaticBody3D.new()
				var cs := CollisionShape3D.new()
				var bx := BoxShape3D.new()
				bx.size = Vector3(12, 3, 1)
				cs.shape = bx
				wall.add_child(cs)
				var wm := MeshInstance3D.new()
				var bm := BoxMesh.new()
				bm.size = bx.size
				wm.mesh = bm
				wall.add_child(wm)
				wall.position = Vector3(0, 1001.5, 1800 - 45)
				add_child(wall)
	elif not spawned and streamer.is_loaded_at(streamer.focus) and streamer.loading_count() == 0:
		_spawn(streamer.focus)
	_update_wheels()
	_update_lamp_lights(dt)
	if _tail_mat:
		_tail_mat.emission_energy_multiplier = (4.0 if car.input_brake > 0.1 else 0.6) + 1.5 * _night
	_read_input(dt)
	_update_hud()
	if args.has("shot") and args.get("test", "") != "crash":
		_shot_logic()
	if args.get("test", "") == "drive":
		_drive_test(dt)
	if args.get("test", "") == "world":
		_world_test(dt)
	if args.get("test", "") == "crash":
		_crash_test(dt)
	if args.get("test", "") == "city" or args.has("autodrive"):
		_autodrive(dt)

func _update_wheels() -> void:
	if car == null or not car.ready_to_drive:
		return
	for i in 4:
		var w = car.wheels[i]
		var n: Node3D = _wheel_nodes[i]
		var p: Vector3 = w.pos + Vector3(0, car.mount_h - w.len, 0)
		if _wheel_rest.size() == 4:
			# imported model: keep its own wheel orientation, add steering and spin
			var rest: Transform3D = _wheel_rest[i]
			var spin := Basis(Vector3.RIGHT, -w.angle)
			n.transform = Transform3D(Basis(Vector3.UP, -w.steer) * rest.basis * spin, p)
			continue
		var b := Basis(Vector3.UP, -w.steer) * Basis(Vector3.RIGHT, -w.angle)
		if not w.left:
			b = b * Basis(Vector3.UP, PI)
		n.transform = Transform3D(b, p)

func _read_input(dt: float) -> void:
	if car == null or args.has("test") or args.has("autodrive"):
		return
	var thr := 1.0 if (Input.is_key_pressed(KEY_W) or Input.is_key_pressed(KEY_UP)) else 0.0
	var brk := 1.0 if (Input.is_key_pressed(KEY_S) or Input.is_key_pressed(KEY_DOWN)) else 0.0
	var st := 0.0
	if Input.is_key_pressed(KEY_A) or Input.is_key_pressed(KEY_LEFT):
		st -= 1.0
	if Input.is_key_pressed(KEY_D) or Input.is_key_pressed(KEY_RIGHT):
		st += 1.0
	var hb := 1.0 if Input.is_key_pressed(KEY_SPACE) else 0.0
	car.steer_direct = false
	# steering wheel / gamepad (axes from controls.cfg)
	var j: Dictionary = controls.read()
	if j["active"]:
		if absf(j["steer"]) > 0.0:
			st = j["steer"]
			car.steer_direct = true
		thr = maxf(thr, j["throttle"])
		brk = maxf(brk, j["brake"])
	hb = maxf(hb, j["handbrake"])
	for b in j["pressed"]:
		match b:
			"shift_up":
				car.auto_gear = false
				car.shift(1)
			"shift_down":
				car.auto_gear = false
				car.shift(-1)
			"reset":
				_spawn(car.global_position)
	if monitor and monitor.visible:
		monitor.text = controls.monitor_text()
	# fell through or far off the world: back onto the road
	if spawned and car.global_position.y < world.ground_height(car.global_position) - 6.0:
		_spawn(car.global_position)
	# automatic reverse: hold brake at a standstill
	if car.auto_gear:
		if car.gear >= 1 and absf(car.speed) < 0.6 and brk > 0.5 and thr < 0.1:
			reverse_hold += dt
			if reverse_hold > 0.35:
				car.gear = -1
		elif car.gear == -1 and absf(car.speed) < 0.6 and thr > 0.5 and brk < 0.1:
			reverse_hold += dt
			if reverse_hold > 0.2:
				car.gear = 1
		else:
			reverse_hold = 0.0
	if car.gear == -1:
		var t := thr
		thr = brk
		brk = t
	car.input_throttle = thr
	car.input_brake = brk
	car.input_steer = st
	car.input_handbrake = hb

func _unhandled_key_input(e: InputEvent) -> void:
	if not (e is InputEventKey and e.pressed and not e.echo):
		return
	match e.keycode:
		KEY_R:
			if world:
				_spawn(car.global_position)
		KEY_C:
			cam.cycle()
		KEY_Q:
			car.auto_gear = false
			car.shift(-1)
		KEY_E:
			car.auto_gear = false
			car.shift(1)
		KEY_M:
			car.auto_gear = not car.auto_gear
		KEY_T:
			car.tcs_on = not car.tcs_on
		KEY_B:
			car.abs_on = not car.abs_on
		KEY_N:
			set_time(hour + 1.0)
		KEY_F1:
			help.visible = not help.visible
		KEY_F4:
			monitor.visible = not monitor.visible
		KEY_F2:
			set_wet(0.0 if wet > 0.5 else 1.0)
		KEY_F12:
			var img := get_viewport().get_texture().get_image()
			var path := OS.get_executable_path().get_base_dir().path_join("screenshot_%d.png" % Time.get_unix_time_from_system())
			img.save_png(path)
			print("[earthsim] screenshot ", path)
		KEY_ESCAPE, KEY_TAB:
			if menu.visible:
				menu.hide_menu()
			else:
				menu.show_menu(hour, streamer.radius, wet > 0.5)

func _update_hud() -> void:
	if hud == null or not hud.visible:
		return
	var t := ""
	t += "FPS %d   чанки %d (+%d)\n" % [Engine.get_frames_per_second(), streamer.chunks.size(), streamer.loading_count()]
	if spawned:
		var g := "R" if car.gear < 0 else ("N" if car.gear == 0 else str(car.gear))
		t += "%3d км/ч   %s   %4d об/мин   %s%s%s\n" % [int(absf(car.speed) * 3.6), g, int(car.rpm), "A" if car.auto_gear else "M", "  ABS" if car.abs_on else "", "  TCS" if car.tcs_on else ""]
		var r: Dictionary = world.road_at(car.global_position)
		if not r.is_empty():
			t += "%s  (%s, %d км/ч)\n" % [r["name"] if r["name"] != "" else "—", r["class"], r["speed"]]
		var ll: Vector2 = world.world_to_latlon(car.global_position)
		t += "%.5f, %.5f   %02d:%02d" % [ll.x, ll.y, int(hour), int(fmod(hour, 1.0) * 60)]
	else:
		t += "загрузка мира…"
	hud.text = t

var _ready_since := -1.0

func _shot_logic() -> void:
	var ready: bool = streamer.loading_count() == 0 and streamer.chunks.size() > 0 and (spawned or args.has("view"))
	if args.has("far"):
		ready = ready and streamer.far_inflight.is_empty() and streamer.far_cells.size() >= streamer.far_total()
	var now := Time.get_ticks_msec() / 1000.0
	if not ready:
		_ready_since = -1.0
	elif _ready_since < 0.0:
		_ready_since = now
	_frames_loaded = _frames_loaded + 1 if ready else 0
	if ready and _frames_loaded >= 3 and now - _ready_since >= float(args.get("shot_wait", "2")):
		var img := get_viewport().get_texture().get_image()
		img.save_png(String(args["shot"]))
		print("[earthsim] shot saved ", args["shot"])
		get_tree().quit(0)

# --------------------------------------------------------------- automated driving test

func _make_test_ground() -> void:
	# physics-only test: a large flat asphalt plane high above the map
	var body := StaticBody3D.new()
	body.set_meta("surface", 0)
	var cs := CollisionShape3D.new()
	var bx := BoxShape3D.new()
	bx.size = Vector3(4000, 2, 4000)
	cs.shape = bx
	body.add_child(cs)
	body.position = Vector3(0, 999, 0)
	add_child(body)
	var mi := MeshInstance3D.new()
	var pm := PlaneMesh.new()
	pm.size = Vector2(4000, 4000)
	mi.mesh = pm
	mi.position = Vector3(0, 1000.01, 0)
	add_child(mi)
	car.global_transform = Transform3D(Basis(), Vector3(0, 1000.45, 1800))
	car.freeze = false
	car.ready_to_drive = true
	car.gear = 1
	spawned = true

func _drive_test(dt: float) -> void:
	if not spawned:
		_test_t += dt
		if _test_t > 120.0:
			print("[test] FAIL: world/spawn not ready in 120 s")
			get_tree().quit(1)
		return
	if _test.is_empty():
		_test = {"t": 0.0, "t100": -1.0, "min_contacts": 4, "max_speed": 0.0, "airborne": 0.0, "min_clear": 99.0, "max_clear": -99.0, "start": car.global_position, "phase": "accel", "brake_from": 0.0, "brake_dist": 0.0, "brake_start": Vector3.ZERO, "max_lat_g": 0.0, "fell": false, "yaw_rate_max": 0.0}
		_test_t = 0.0
		return
	_test_t += dt
	var tel: Dictionary = car.telemetry()
	var kmh: float = tel["speed"] * 3.6
	_test["max_speed"] = maxf(_test["max_speed"], kmh)
	if tel["contacts"] < 4:
		_test["airborne"] += dt
	var g := 1000.0
	var clear: float = car.global_position.y - g
	_test["min_clear"] = minf(_test["min_clear"], clear)
	_test["max_clear"] = maxf(_test["max_clear"], clear)
	if clear < -1.0:
		_test["fell"] = true
	var ph: String = _test["phase"]
	if ph == "accel":
		car.input_throttle = 1.0
		car.input_brake = 0.0
		car.input_steer = 0.0
		if _test["t100"] < 0.0 and kmh >= 100.0:
			_test["t100"] = _test_t
		if kmh >= 100.0 or _test_t > 20.0:
			_test["phase"] = "brake"
			_test["brake_from"] = kmh
			_test["brake_start"] = car.global_position
	elif ph == "brake":
		car.input_throttle = 0.0
		car.input_brake = 1.0
		if absf(kmh) < 1.0:
			_test["brake_dist"] = car.global_position.distance_to(_test["brake_start"])
			_test["phase"] = "turn"
			_test["turn_t"] = _test_t
	elif ph == "turn":
		car.input_brake = 0.0
		car.input_throttle = 0.45 if kmh < 35.0 else 0.0
		car.input_steer = 0.8
		var lat: float = absf((car.linear_velocity.cross(car.angular_velocity)).length()) / 9.81
		_test["max_lat_g"] = maxf(_test["max_lat_g"], lat)
		if _test_t - _test["turn_t"] > 10.0:
			_test["phase"] = "done"
	if ph == "done" or _test_t > 60.0:
		var ok := true
		var why := []
		if _test["fell"]:
			ok = false
			why.append("fell through the ground")
		if _test["t100"] < 0.0 or _test["t100"] > 14.0:
			ok = false
			why.append("0-100 too slow: %.1f" % _test["t100"])
		if _test["brake_dist"] <= 0.0 or _test["brake_dist"] > 60.0:
			ok = false
			why.append("braking distance %.1f m" % _test["brake_dist"])
		if _test["min_clear"] < -0.2 or _test["max_clear"] > 1.5:
			ok = false
			why.append("body height vs ground out of range")
		print("[test] drive: 0-100 %.2f s, max %.0f km/h, brake 100-0 %.1f m, airborne %.2f s, clearance %.2f..%.2f m, max lat %.2f g -> %s %s" % [
			_test["t100"], _test["max_speed"], _test["brake_dist"], _test["airborne"], _test["min_clear"], _test["max_clear"], _test["max_lat_g"], "PASS" if ok else "FAIL", ", ".join(why)])
		get_tree().quit(0 if ok else 1)

# --------------------------------------------------------------- streaming smoke test

func _world_test(dt: float) -> void:
	_test_t += dt
	if spawned and streamer.loading_count() == 0 and streamer.chunks.size() > 0:
		var n: int = streamer.chunks.size()
		var g: float = world.ground_height(car.global_position)
		var clear: float = car.global_position.y - g
		var ok := n >= 20 and clear > -0.3 and clear < 2.0
		print("[test] world: %d chunks loaded in %.1f s, last build %.0f ms, worst apply %.0f ms, car clearance %.2f m -> %s" % [n, _test_t, streamer.stats["build_ms"], streamer.stats["max_apply_ms"], clear, "PASS" if ok else "FAIL"])
		get_tree().quit(0 if ok else 1)
	elif _test_t > 240.0:
		print("[test] world: FAIL timeout, %d chunks" % streamer.chunks.size())
		get_tree().quit(1)

# --------------------------------------------------------------- autopilot / city driving test

var _ad := {}

func _autodrive(dt: float) -> void:
	if not spawned:
		_test_t += dt
		if _test_t > 120.0 and args.get("test", "") == "city":
			print("[test] city: FAIL world/spawn not ready")
			get_tree().quit(1)
		return
	if _ad.is_empty():
		_ad = {"t": 0.0, "dist": 0.0, "last": car.global_position, "air": 0.0, "min_clear": 99.0, "max_clear": -99.0, "fell": false, "resets": 0, "stuck": 0.0, "max_roll": 0.0}
	_ad["t"] += dt
	var fwd := -car.global_transform.basis.z
	var look := clampf(absf(car.speed) * 1.1, 7.0, 30.0)
	var f: Dictionary = world.follow(car.global_position, fwd, look)
	var want_kmh := float(args.get("speed", "50"))
	if f.is_empty():
		car.input_throttle = 0.0
		car.input_brake = 1.0
	else:
		var tgt: Vector3 = f["target"]
		var to := tgt - car.global_position
		to.y = 0.0
		var ang := fwd.signed_angle_to(to.normalized(), Vector3.UP)  # + = target to the left
		car.steer_direct = true
		car.input_steer = clampf(-ang * 2.2, -1.0, 1.0)
		var lim := float(f["limit"]) if int(f["limit"]) > 0 else want_kmh
		var v := minf(want_kmh, lim + 5.0)
		# slow down for curves
		v = minf(v, lerpf(v, 22.0, clampf(absf(ang) * 2.5, 0.0, 1.0)))
		var kmh: float = car.speed * 3.6
		car.input_throttle = clampf((v - kmh) * 0.15, 0.0, 1.0)
		car.input_brake = clampf((kmh - v - 4.0) * 0.1, 0.0, 1.0)
	_ad["dist"] += car.global_position.distance_to(_ad["last"])
	_ad["last"] = car.global_position
	var tel: Dictionary = car.telemetry()
	if tel["contacts"] < 3:
		_ad["air"] += dt
	_ad["max_roll"] = maxf(_ad["max_roll"], absf(car.global_transform.basis.x.y))
	if absf(car.speed) < 0.5:
		_ad["stuck"] += dt
	else:
		_ad["stuck"] = 0.0
	if _ad["stuck"] > 4.0:
		_ad["resets"] += 1
		_ad["stuck"] = 0.0
		_spawn(car.global_position + fwd * 30.0)
	if int(_ad["t"] / 10.0) != int((_ad["t"] - dt) / 10.0):
		print("[autodrive] t=%.0f s dist=%.0f m speed=%.0f km/h pos=%s chunks=%d" % [_ad["t"], _ad["dist"], car.speed * 3.6, car.global_position, streamer.chunks.size()])
	if int(_ad["t"] * 2.0) != int((_ad["t"] - dt) * 2.0):
		var g: float = world.ground_height(car.global_position)
		var clear: float = car.global_position.y - g
		_ad["min_clear"] = minf(_ad["min_clear"], clear)
		_ad["max_clear"] = maxf(_ad["max_clear"], clear)
		if clear < -1.5:
			_ad["fell"] = true
	if args.get("test", "") == "city" and _ad["t"] > float(args.get("duration", "90")):
		var ok: bool = not _ad["fell"] and _ad["dist"] > _ad["t"] * 6.0 and _ad["air"] < 2.0 and _ad["max_roll"] < 0.5
		print("[test] city: drove %.0f m in %.0f s, wheels off the ground %.2f s, clearance %.2f..%.2f m, max roll %.2f, resets %d -> %s" % [_ad["dist"], _ad["t"], _ad["air"], _ad["min_clear"], _ad["max_clear"], _ad["max_roll"], _ad["resets"], "PASS" if ok else "FAIL"])
		get_tree().quit(0 if ok else 1)

# --------------------------------------------------------------- crash test: drive into a wall

var _crash_t := 0.0
var _crash_vmax := 0.0

func _crash_test(dt: float) -> void:
	if not spawned:
		return
	_crash_t += dt
	car.input_throttle = 1.0 if _crash_t < 4.5 else 0.0
	car.input_brake = 0.0 if _crash_t < 4.5 else 1.0
	_crash_vmax = maxf(_crash_vmax, car.speed * 3.6)
	if _crash_t > 4.0 and cam.target == car and cam.mode != 2:
		cam.mode = 2
		cam._yaw = PI * 0.4
		cam._pitch = -0.55
		cam._dist = 4.6
	if _crash_t > 8.0:
		var up: float = car.global_transform.basis.y.y
		var v: float = car.linear_velocity.length()
		var ok: bool = _dent_total > 0.02 and up > 0.8 and v < 3.0 and is_finite(car.global_position.x)
		print("[test] crash: impact at %.0f km/h, dent depth sum %.2f m, upright %.2f, speed after %.1f m/s, steer bias %.3f -> %s" % [_crash_vmax, _dent_total, up, v, car.steer_bias, "PASS" if ok else "FAIL"])
		if args.has("shot"):
			if _crash_t > 8.5:
				get_viewport().get_texture().get_image().save_png(String(args["shot"]))
				print("[earthsim] shot saved ", args["shot"])
				get_tree().quit(0)
		else:
			get_tree().quit(0 if ok else 1)
