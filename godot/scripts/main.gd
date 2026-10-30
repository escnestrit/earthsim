# Game entry: sky and light, the streamed world, the player's car, cameras, HUD and options.
# Command line (after "--"): --lat=.. --lon=.. --data=<cache dir> --offline --noenrich --time=<hour>
#   --shot=<png> [--shot-after=<s>] [--cam=x,y,z,yaw_deg,pitch_deg]  (take a screenshot and quit: for tests)
extends Node3D

var world: EarthWorld
var car: Node3D          # EarthCar (built-in) or EarthJCar (a BeamNG vehicle)
var builtin_car: EarthCar
var jcar: EarthJCar
var vehicles := []
var vehicle_i := -1
var car_msg := ""
var car_msg_t := 0.0
var spawn := Vector3.ZERO
var spawn_yaw := 0.0
var cam_grounded := false
var last_car_pos := Vector3.ZERO
var cam: Camera3D
var sun: DirectionalLight3D
var env: Environment
var hud: Label
var help: Label
var args := {}
var hour := 13.0
var cam_mode := 0           # 0 chase, 1 hood, 2 free
var cam_yaw := 0.0
var cam_pitch := 0.25
var cam_dist := 7.0
var free_pos := Vector3.ZERO
var started := false
var spawn_ready := false
var shot_t := -1.0
var t_since := 0.0
var fps_acc := 0.0

func _ready() -> void:
	for a in OS.get_cmdline_user_args() + OS.get_cmdline_args():
		if a.begins_with("--"):
			var kv := a.substr(2).split("=", true, 1)
			args[kv[0]] = kv[1] if kv.size() > 1 else "1"
	var lat := float(args.get("lat", "53.9024"))
	var lon := float(args.get("lon", "27.5618"))
	hour = float(args.get("time", "13"))
	var cache: String = args.get("data", OS.get_environment("EARTHSIM_DATA"))
	if cache == "":
		cache = ProjectSettings.globalize_path("user://cache")
	_setup_env()
	world = EarthWorld.new()
	world.name = "World"
	add_child(world)
	var detail := float(args.get("detail", "700"))
	var far := float(args.get("far", "2400"))
	if args.has("debugmat"):
		var cols := [Color.RED, Color.GREEN, Color.YELLOW, Color.MAGENTA, Color.CYAN]
		var ids: PackedStringArray = (args["debugmat"] as String).split(",")
		for i in ids.size():
			world.debug_mats[int(ids[i])] = cols[i % cols.size()]
	world.start(lat, lon, cache, not args.has("offline"), detail, far, not args.has("noenrich"))
	world.chunk_added.connect(_on_chunk)
	_setup_car()
	_setup_hud()
	if args.has("shot"):
		shot_t = float(args.get("shot-after", "20"))
	if args.has("cammode"):
		cam_mode = int(args["cammode"])
	if args.has("cam"):
		var c: PackedStringArray = (args["cam"] as String).split(",")
		cam_mode = 2
		free_pos = Vector3(float(c[0]), float(c[1]), float(c[2]))
		cam_yaw = deg_to_rad(float(c[3]))
		cam_pitch = deg_to_rad(float(c[4]))

func _setup_env() -> void:
	var we := WorldEnvironment.new()
	env = Environment.new()
	var sky := Sky.new()
	var sm := PhysicalSkyMaterial.new()
	sm.rayleigh_coefficient = 2.0
	sm.mie_coefficient = 0.005
	sm.turbidity = 8.0
	sm.ground_color = Color(0.25, 0.25, 0.22)
	sm.energy_multiplier = 1.0
	sky.sky_material = sm
	if not args.has("physsky"):
		var shm := ShaderMaterial.new()
		shm.shader = load("res://shaders/sky.gdshader")
		sky.sky_material = shm
	elif args.has("procsky"):
		var ps := ProceduralSkyMaterial.new()
		ps.sky_top_color = Color(0.25, 0.45, 0.8)
		ps.sky_horizon_color = Color(0.7, 0.78, 0.88)
		ps.ground_horizon_color = Color(0.6, 0.65, 0.7)
		ps.ground_bottom_color = Color(0.2, 0.2, 0.2)
		ps.sun_angle_max = 20.0
		sky.sky_material = ps
	env.background_mode = Environment.BG_SKY
	env.sky = sky
	env.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	env.reflected_light_source = Environment.REFLECTION_SOURCE_SKY
	env.tonemap_mode = Environment.TONE_MAPPER_AGX
	env.tonemap_exposure = 1.0
	env.ssao_enabled = true
	env.ssao_radius = 1.5
	env.ssao_intensity = 2.0
	env.ssil_enabled = true
	env.ssr_enabled = true
	env.ssr_max_steps = 48
	env.glow_enabled = true
	env.glow_intensity = 0.4
	env.glow_bloom = 0.05
	env.fog_enabled = true
	env.fog_mode = Environment.FOG_MODE_DEPTH
	env.fog_density = 0.0006
	env.fog_aerial_perspective = 0.85
	env.fog_sky_affect = 0.0
	env.fog_depth_begin = 200.0
	env.fog_depth_end = 3500.0
	env.fog_light_color = Color(0.7, 0.78, 0.9)
	env.sdfgi_enabled = not args.has("nogi")
	env.sdfgi_use_occlusion = true
	env.sdfgi_cascades = 6
	env.sdfgi_min_cell_size = 0.4
	env.adjustment_enabled = true
	env.adjustment_saturation = 1.08
	we.environment = env
	add_child(we)
	sun = DirectionalLight3D.new()
	sun.shadow_enabled = true
	sun.directional_shadow_mode = DirectionalLight3D.SHADOW_PARALLEL_4_SPLITS
	sun.directional_shadow_max_distance = 600.0
	sun.light_angular_distance = 0.5
	sun.shadow_blur = 1.2
	add_child(sun)
	_apply_time()

func _apply_time() -> void:
	# sun path of a mid-latitude summer day: south at noon
	var t := (hour - 6.0) / 12.0 * PI
	var elev := sin(t) * deg_to_rad(58.0)
	var az := PI + (hour - 12.0) / 12.0 * PI
	sun.rotation = Vector3(-elev, az, 0)
	var day := clampf(sin(t) * 3.0 + 0.3, 0.0, 1.0)
	sun.light_energy = 1.6 * day + 0.02
	sun.light_color = Color(1.0, lerpf(0.65, 0.96, day), lerpf(0.45, 0.9, day))
	var night := 1.0 - clampf(sin(t) * 4.0 + 0.6, 0.0, 1.0)
	env.background_energy_multiplier = lerpf(1.0, 0.08, night)
	env.ambient_light_energy = lerpf(1.0, 0.25, night)
	if world:
		world.mats.set_night(night)

func _setup_car() -> void:
	car = EarthCar.new()
	car.name = "Car"
	add_child(car)
	var mats := {}
	for mat in [EarthMaterials.M_CARPAINT, EarthMaterials.M_GLASS, EarthMaterials.M_DARK, EarthMaterials.M_LAMP, EarthMaterials.M_TIRE]:
		mats[mat] = world.mats.get_mat(mat)
	# the player's windows are see-through from inside
	var g := (world.mats.get_mat(EarthMaterials.M_GLASS) as StandardMaterial3D).duplicate() as StandardMaterial3D
	g.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	g.albedo_color = Color(0.1, 0.13, 0.15, 0.35)
	g.cull_mode = BaseMaterial3D.CULL_DISABLED
	mats[EarthMaterials.M_GLASS] = g
	var paint := (world.mats.get_mat(EarthMaterials.M_CARPAINT) as StandardMaterial3D).duplicate() as StandardMaterial3D
	paint.vertex_color_use_as_albedo = false
	paint.albedo_color = Color(0.55, 0.06, 0.05)
	mats[EarthMaterials.M_CARPAINT] = paint
	var wheel := CylinderMesh.new()
	wheel.top_radius = 0.33
	wheel.bottom_radius = 0.33
	wheel.height = 0.22
	wheel.radial_segments = 24
	wheel.material = world.mats.get_mat(EarthMaterials.M_TIRE)
	car.setup(world.stream, EarthCore.model(4, 1), mats, wheel)
	builtin_car = car
	# BeamNG vehicles the player owns: a mods folder next to the game, and BeamNG.drive's own folder
	var dirs := PackedStringArray([OS.get_executable_path().get_base_dir().path_join("mods")])
	if args.has("mods"):
		dirs.append(args["mods"])
	vehicles = EarthJCar.find_vehicles(dirs)
	if not args.has("car") and not args.has("builtin"):
		# a vehicle the player put in the mods folder next to the game is driven by default
		var mods_dir := dirs[0]
		for i in vehicles.size():
			if (vehicles[i]["path"] as String).begins_with(mods_dir):
				_load_vehicle(i)
				break
	if args.has("car"):
		var want: String = args["car"]
		for i in vehicles.size():
			if vehicles[i]["name"] == want or vehicles[i]["path"] == want:
				_load_vehicle(i)
		if vehicle_i < 0 and FileAccess.file_exists(want):
			vehicles.append({"path": want, "name": want.get_file().get_basename()})
			_load_vehicle(vehicles.size() - 1)
	spawn = Vector3(float(args.get("x", "0")), 0, float(args.get("z", "0")))
	spawn_yaw = deg_to_rad(float(args.get("yaw", "0")))
	cam = Camera3D.new()
	cam.far = 6000.0
	cam.near = 0.1
	cam.fov = 70.0
	add_child(cam)
	cam.make_current()

func _load_vehicle(i: int) -> void:
	if jcar:
		jcar.queue_free()
		jcar = null
	vehicle_i = i
	if i < 0 or i >= vehicles.size():
		return
	jcar = EarthJCar.new()
	jcar.name = "JCar"
	add_child(jcar)
	jcar.load(world.stream, vehicles[i]["path"])
	_say("Загрузка: " + vehicles[i]["name"] + "…")

func _say(t: String) -> void:
	car_msg = t
	car_msg_t = 6.0
	print(t)

func _poll_vehicle() -> void:
	if jcar == null or car == jcar:
		return
	var st: String = jcar.poll()
	if st == "ready":
		# swap in where the built-in car is (or at the spawn road)
		var p: Vector3 = builtin_car.global_position if spawn_ready else spawn
		var f: Vector3 = builtin_car.forward()
		var r: Dictionary = world.stream.nearest_road(p, 60.0)
		var pos: Vector3 = r["pos"] if not r.is_empty() else p
		var yaw: float = r["yaw"] if not r.is_empty() else atan2(-f.z, f.x)
		if world.stream.collision_ready(pos) and jcar.place(pos, yaw):
			car = jcar
			builtin_car.visible = false
			builtin_car.process_mode = Node.PROCESS_MODE_DISABLED
			spawn_ready = true
			_say(vehicles[vehicle_i]["name"] + " загружен.\n" + jcar.notes())
			if args.has("auto"):
				var a: PackedStringArray = (args["auto"] as String).split(",")
				jcar.set_auto_input(float(a[0]), float(a[1]) if a.size() > 1 else 0.0, 0.0)
	elif st.begins_with("error"):
		_say("Не удалось загрузить " + vehicles[vehicle_i]["name"] + ": " + st)
		jcar.queue_free()
		jcar = null

func _next_vehicle() -> void:
	if vehicles.is_empty():
		_say("Машины BeamNG не найдены (папка mods рядом с игрой или установленный BeamNG.drive)")
		return
	if car == jcar and jcar:
		car = builtin_car
		builtin_car.visible = true
		builtin_car.process_mode = Node.PROCESS_MODE_INHERIT
	var n := vehicle_i + 1
	if n >= vehicles.size():
		if jcar:
			jcar.queue_free()
			jcar = null
		vehicle_i = -1
		_say("Встроенная машина")
		return
	_load_vehicle(n)

func _setup_hud() -> void:
	var cl := CanvasLayer.new()
	add_child(cl)
	hud = Label.new()
	hud.position = Vector2(12, 8)
	hud.add_theme_font_size_override("font_size", 15)
	hud.add_theme_color_override("font_outline_color", Color.BLACK)
	hud.add_theme_constant_override("outline_size", 4)
	cl.add_child(hud)
	help = Label.new()
	help.text = "WASD / стрелки — ехать, Space — ручник, R — поставить на колёса, C — камера, V — машина BeamNG, T — время суток, G — GI вкл/выкл, F1 — помощь"
	help.add_theme_font_size_override("font_size", 14)
	help.add_theme_color_override("font_outline_color", Color.BLACK)
	help.add_theme_constant_override("outline_size", 4)
	help.anchor_top = 1.0
	help.anchor_bottom = 1.0
	help.position = Vector2(12, -30)
	cl.add_child(help)

func _on_chunk(_cx: int, _cy: int, _full: bool) -> void:
	pass

func _unhandled_input(e: InputEvent) -> void:
	if e is InputEventKey and e.pressed and not e.echo:
		match e.keycode:
			KEY_V:
				_next_vehicle()
			KEY_R:
				car.reset()
			KEY_C:
				cam_mode = (cam_mode + 1) % 3
				free_pos = cam.global_position
			KEY_T:
				hour = fmod(hour + 1.5, 24.0)
				_apply_time()
			KEY_G:
				env.sdfgi_enabled = not env.sdfgi_enabled
			KEY_F1:
				help.visible = not help.visible
	if e is InputEventMouseMotion and Input.is_mouse_button_pressed(MOUSE_BUTTON_RIGHT):
		cam_yaw -= e.relative.x * 0.005
		cam_pitch = clampf(cam_pitch + e.relative.y * 0.005, -1.2, 1.4)
	if e is InputEventMouseButton and e.pressed:
		if e.button_index == MOUSE_BUTTON_WHEEL_UP:
			cam_dist = maxf(3.0, cam_dist * 0.9)
		elif e.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			cam_dist = minf(40.0, cam_dist * 1.1)

func _process(dt: float) -> void:
	t_since += dt
	_poll_vehicle()
	car_msg_t -= dt
	if not spawn_ready and world.stream.collision_ready(spawn):
		var r: Dictionary = world.stream.nearest_road(spawn, 400.0)
		if not r.is_empty():
			spawn_ready = car.place(r["pos"], r["yaw"])
		elif t_since > 20.0:
			spawn_ready = car.place(spawn, spawn_yaw)
		if spawn_ready and args.has("auto"):
			var a: PackedStringArray = (args["auto"] as String).split(",")
			car.set_auto_input(float(a[0]), float(a[1]) if a.size() > 1 else 0.0, 0.0)
	var focus_p: Vector3 = car.global_position if spawn_ready else spawn
	world.focus = focus_p if cam_mode != 2 else cam.global_position
	_update_camera(dt)
	if spawn_ready:
		var cv := (car.global_position - last_car_pos) / maxf(dt, 1e-4)
		last_car_pos = car.global_position
		world.update_traffic(dt, car.global_position, cv)
	fps_acc = lerpf(fps_acc, 1.0 / maxf(dt, 1e-4), 0.05)
	var ll := world.stream.latlon(car.global_position)
	var gear_s: String = "R" if car.gear() < 0 else str(car.gear())
	hud.text = "%d км/ч   передача %s   %d об/мин\n%.5f, %.5f   %02d:%02d   FPS %d\n%s" % [
		roundi(car.speed_kmh()), gear_s, roundi(car.rpm()), ll.x, ll.y, int(hour), int(fmod(hour, 1.0) * 60), roundi(fps_acc), world.stream.status()]
	if car_msg_t > 0.0:
		hud.text += "\n" + car_msg
	if shot_t > 0.0 and int(t_since / 5.0) != int((t_since - dt) / 5.0):
		print("t=%.0f fps=%d car %s %.0f km/h dmg %.2f | %s" % [t_since, fps_acc, car.global_position, car.speed_kmh(), car.damage(), world.stream.status()])
	if shot_t > 0.0 and t_since > shot_t:
		shot_t = -1.0
		await RenderingServer.frame_post_draw
		var img := get_viewport().get_texture().get_image()
		img.save_png(args["shot"])
		print("screenshot saved ", args["shot"], " status: ", world.stream.status())
		get_tree().quit()

func _update_camera(dt: float) -> void:
	if cam_mode == 2:
		if args.has("cam") and not cam_grounded:
			var gh := world.ground(free_pos.x, free_pos.z)
			if not is_nan(gh):
				free_pos.y += gh
				cam_grounded = true
		var f := Vector3(-sin(cam_yaw) * cos(cam_pitch), -sin(cam_pitch), -cos(cam_yaw) * cos(cam_pitch))
		var mv := Vector3.ZERO
		if not args.has("cam"):
			if Input.is_key_pressed(KEY_I): mv += f
			if Input.is_key_pressed(KEY_K): mv -= f
			if Input.is_key_pressed(KEY_J): mv -= f.cross(Vector3.UP).normalized()
			if Input.is_key_pressed(KEY_L): mv += f.cross(Vector3.UP).normalized()
		free_pos += mv * dt * (60.0 if Input.is_key_pressed(KEY_SHIFT) else 15.0)
		cam.global_position = free_pos
		cam.look_at(free_pos + f, Vector3.UP)
		return
	var tr := car.global_transform
	var fwd: Vector3 = car.forward()
	fwd.y = 0
	if fwd.length() < 0.1:
		fwd = Vector3.FORWARD
	fwd = fwd.normalized()
	if cam_mode == 1:
		cam.global_position = tr * car.eye()
		cam.global_basis = tr.basis * Basis(Vector3.UP, -PI / 2) * Basis(Vector3.RIGHT, -0.08)
		return
	var back: Vector3 = fwd.rotated(Vector3.UP, cam_yaw)
	var target: Vector3 = (car.global_position if spawn_ready else spawn) + Vector3(0, 1.3, 0)
	var want: Vector3 = target - back * cam_dist * cos(cam_pitch) + Vector3(0, cam_dist * sin(cam_pitch), 0)
	var g := world.ground(want.x, want.z)
	if not is_nan(g):
		want.y = maxf(want.y, g + 0.6)
	cam.global_position = cam.global_position.lerp(want, clampf(dt * 6.0, 0.0, 1.0)) if t_since > 0.5 else want
	cam.look_at(target, Vector3.UP)
