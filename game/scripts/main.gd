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
const UiScript := preload("res://scripts/ui.gd")
const FetcherScript := preload("res://scripts/fetcher.gd")
const REGION_RADIUS := 1800.0     # metres of map loaded around a place picked anywhere on Earth
const HOP_MARGIN := 650.0         # load the next region when the car gets this close to the edge
const ControlsScript := preload("res://scripts/controls.gd")
const TrafficScript := preload("res://scripts/traffic.gd")

var args := {}
var world: EarthWorld
var streamer
var car: RigidBody3D
var cam: Camera3D
var sun: DirectionalLight3D
var env: Environment
var sky_mat: ProceduralSkyMaterial
var ui
var fetcher
var world_dynamic := false        # region downloaded at runtime (not a bundled one)
var _switching := false
var _hop_retry := 0.0
var _region_box := Rect2()        # loaded region in Godot x/z
var _place := ""                  # human readable "where am I"
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
var traffic
var monitor: Label
var _lamp_lights: Array[OmniLight3D] = []
var _lamp_t := 0.0
var _tail_mat: StandardMaterial3D
var _body_parts := []
var _wheel_rest := []
var _dent_total := 0.0
var _fall_t := 0.0
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
	_setup_fetcher()
	_setup_ui()
	await _setup_world()
	if world == null:
		return
	_setup_car()
	_setup_monitor()
	controls = ControlsScript.new()
	controls.load_config()
	_setup_lamp_lights()
	if not args.has("notraffic") and args.get("test", "") in ["", "traffic"]:
		traffic = Node3D.new()
		traffic.set_script(TrafficScript)
		add_child(traffic)
		traffic.setup(world, car)
		if args.has("traffic"):
			traffic.max_cars = int(args["traffic"])
	if not args.has("test") and not args.has("shot") and DisplayServer.get_name() != "headless":
		var au := AudioStreamPlayer.new()
		au.set_script(AudioScript)
		au.car = car
		add_child(au)
	set_time(hour)
	if args.has("rain"):
		set_wet(1.0)

var regions := []        # names of the regions found next to the exe (data/<name>/region.json)

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
		var dir := String(d)
		if FileAccess.file_exists(dir.path_join("region.json")):
			return dir
		# several regions: data/<name>/region.json
		var da := DirAccess.open(dir)
		if da == null:
			continue
		regions = []
		for sub in da.get_directories():
			if FileAccess.file_exists(dir.path_join(sub).path_join("region.json")):
				regions.append(sub)
		if regions.is_empty():
			continue
		regions.sort()
		var want: String = args.get("region", "minsk")
		if not regions.has(want):
			want = regions[0]
		args["region"] = want
		return dir.path_join(want)
	return ""

func _setup_fetcher() -> void:
	fetcher = FetcherScript.new()
	if args.has("overture_url"):
		fetcher.overture_url = args["overture_url"]
	if args.has("terrarium_url"):
		fetcher.terrarium_url = args["terrarium_url"]
	if args.has("cache"):
		fetcher.cache_dir = args["cache"]
	add_child(fetcher)
	fetcher.progress.connect(_on_fetch_progress)

func _setup_ui() -> void:
	ui = UiScript.new()
	add_child(ui)
	ui.setup(fetcher)
	ui.teleport.connect(_teleport)
	ui.time_changed.connect(set_time)
	ui.rain_changed.connect(func(on): set_wet(1.0 if on else 0.0))
	ui.radius_changed.connect(func(r): streamer.radius = r)
	ui.assist_changed.connect(_on_assist)
	ui.respawn.connect(func(): _spawn(car.global_position))
	ui.quit_game.connect(func(): get_tree().quit())
	if args.has("shot") and not args.has("ui"):
		ui.visible = false

var _loading_title := ""

func _on_assist(n: String, on: bool) -> void:
	if n == "abs":
		car.abs_on = on
	elif n == "tcs":
		car.tcs_on = on
	elif n == "auto":
		car.auto_gear = on

func _on_fetch_progress(done: int, total: int, text: String) -> void:
	if ui.loading.visible:
		ui.show_loading(_loading_title, text, float(done) / maxf(1.0, total))
	elif _switching:
		ui.set_status("Подгружаю карту впереди… %d / %d" % [done, total])

# Bundled regions (offline, next to the exe) that contain a point
func _bundled_for(lat: float, lon: float) -> String:
	var base := _data_base()
	if base == "":
		return ""
	var dirs := []
	if FileAccess.file_exists(base.path_join("region.json")):
		dirs.append(base)
	for r in regions:
		dirs.append(base.path_join(r))
	for d in dirs:
		var j = JSON.parse_string(FileAccess.get_file_as_string(String(d).path_join("region.json")))
		if j == null:
			continue
		var dl := (float(j["radius_m"]) - 700.0) / 111320.0
		if absf(lat - float(j["lat"])) < dl and absf(lon - float(j["lon"])) < dl / cos(deg_to_rad(lat)):
			return d
	return ""

func _data_base() -> String:
	var c := []
	if args.has("data"):
		c.append(args["data"])
	if OS.has_environment("EARTHSIM_DATA"):
		c.append(OS.get_environment("EARTHSIM_DATA"))
	c.append(OS.get_executable_path().get_base_dir().path_join("data"))
	c.append(ProjectSettings.globalize_path("res://").path_join("../data"))
	c.append(ProjectSettings.globalize_path("res://").path_join("../data_local"))
	for d in c:
		if DirAccess.dir_exists_absolute(d):
			if FileAccess.file_exists(String(d).path_join("region.json")) or not DirAccess.get_directories_at(d).is_empty():
				return d
	return ""

# Load a world on a worker thread (the frame keeps running: loading screen, HUD)
func _load_threaded(root: String, spec: String) -> EarthWorld:
	var w := EarthWorld.new()
	var res := [false]
	var tid := WorkerThreadPool.add_task(func(): res[0] = w.load(root) if spec == "" else w.load_spec(root, spec), true, "world load")
	while not WorkerThreadPool.is_task_completed(tid):
		await get_tree().process_frame
	WorkerThreadPool.wait_for_task_completion(tid)
	for l in w.get_log():
		print("  ", l)
	return w if res[0] else null

# Make a world for any point: bundled data if it covers it, otherwise download into the cache.
func _world_for(lat: float, lon: float, quiet: bool) -> EarthWorld:
	var b := _bundled_for(lat, lon)
	if b != "" and not args.has("force_fetch"):
		return await _load_threaded(b, "")
	_loading_title = "Загружаю карту: %.4f, %.4f" % [lat, lon]
	if not quiet:
		ui.show_loading(_loading_title, "Подключение…", 0.0)
	var spec: String = await fetcher.prepare_region(lat, lon, REGION_RADIUS)
	if spec == "":
		print("[earthsim] fetch failed: ", fetcher.last_error)
		return null
	if not quiet:
		ui.show_loading(_loading_title, "Строю дороги и здания…", 1.0)
	return await _load_threaded(fetcher.cache_dir, spec)

func _setup_world() -> void:
	var dir := _find_data()
	var want := args.has("lat") and args.has("lon")
	var lat := float(args.get("lat", "53.9024"))
	var lon := float(args.get("lon", "27.5618"))
	if dir != "" and not args.has("force_fetch") and (not want or _bundled_for(lat, lon) != ""):
		if want:
			dir = _bundled_for(lat, lon)
		world = await _load_threaded(dir, "")
	else:
		world = await _world_for(lat, lon, false)
		world_dynamic = world != null and _bundled_for(lat, lon) == ""
	ui.hide_loading()
	if world == null:
		_fatal("Не удалось загрузить карту.\n" + (fetcher.last_error if fetcher.last_error != "" else "Нет данных рядом с exe и нет доступа к интернету."))
		return
	streamer = StreamerScript.new()
	add_child(streamer)
	streamer.setup(world)
	if args.has("radius"):
		streamer.radius = float(args["radius"])
	if args.has("shot") and not args.has("far"):
		streamer.far_enabled = false
	var focus := Vector3.ZERO
	if want:
		focus = world.latlon_to_world(lat, lon)
	focus.y = world.ground_height(focus)
	streamer.focus = focus
	_after_world_change()

func _after_world_change() -> void:
	var info: Dictionary = world.region_info()
	var a: Vector3 = info["min"]
	var b: Vector3 = info["max"]
	_region_box = Rect2(Vector2(minf(a.x, b.x), minf(a.z, b.z)), Vector2(absf(b.x - a.x), absf(b.z - a.z)))
	RenderingServer.global_shader_parameter_set("biome", float(info.get("biome", 0)))
	_place = "%s · климат: %s · %s" % [info.get("zone_name", ""), info.get("biome_name", ""), "левостороннее движение" if info.get("lht", false) else "правостороннее движение"]
	print("[earthsim] region %.4f, %.4f: %s" % [info["lat"], info["lon"], _place])

# Swap the active world. keep = true while driving (no loading screen, the car carries on).
var _swap_err := 0.0   # metres between where the car was and where it is after a region switch

func _install_world(w: EarthWorld, keep: bool, shift: Vector3) -> void:
	var ll0: Vector2 = world.world_to_latlon(car.global_position)
	world = w
	if keep:
		car.global_position += shift
		var ll1: Vector2 = world.world_to_latlon(car.global_position)
		_swap_err = Vector2((ll1.x - ll0.x) * 111320.0, (ll1.y - ll0.y) * 111320.0 * cos(deg_to_rad(ll0.x))).length()
		if cam:
			cam.global_position += shift
	streamer.set_world(w, keep, shift)
	if traffic:
		traffic.reset(w)
	_after_world_change()

func _teleport(lat: float, lon: float) -> void:
	var p: Vector3 = world.latlon_to_world(lat, lon)
	var inside := _region_box.grow(-300.0).has_point(Vector2(p.x, p.z))
	if not inside:
		if _switching:
			ui.toast("Подожди, предыдущая загрузка ещё идёт")
			return
		_switching = true
		var w: EarthWorld = await _world_for(lat, lon, false)
		ui.hide_loading()
		_switching = false
		if w == null:
			ui.toast("Не удалось загрузить карту: " + fetcher.last_error, 6.0)
			return
		world_dynamic = _bundled_for(lat, lon) == ""
		_install_world(w, false, Vector3.ZERO)
		p = world.latlon_to_world(lat, lon)
	p.y = world.ground_height(p)
	spawned = false
	car.ready_to_drive = false
	car.freeze = true
	car.global_position = p + Vector3(0, 2, 0)
	streamer.focus = p

# Driving towards the edge of the loaded map: fetch the next region around the car in the
# background and switch to it without stopping.
func _check_hop(dt: float) -> void:
	if _switching or not spawned or args.has("test") and args["test"] != "fetch":
		return
	_hop_retry -= dt
	if _hop_retry > 0.0:
		return
	var p: Vector3 = car.global_position + streamer.lead * 0.5
	var r: Rect2 = _region_box
	var edge := minf(minf(p.x - r.position.x, r.end.x - p.x), minf(p.z - r.position.y, r.end.y - p.z))
	if edge > HOP_MARGIN:
		return
	_hop(p)

func _hop(p: Vector3) -> void:
	_switching = true
	var ll: Vector2 = world.world_to_latlon(p)
	ui.set_status("Подгружаю карту впереди…")
	var w: EarthWorld = await _world_for(ll.x, ll.y, true)
	ui.set_status("")
	_switching = false
	if w == null:
		_hop_retry = 30.0
		ui.toast("Нет данных дальше: " + fetcher.last_error, 5.0)
		return
	# the same point on Earth in both local frames
	var old_p: Vector3 = world.latlon_to_world(ll.x, ll.y)
	var new_p: Vector3 = w.latlon_to_world(ll.x, ll.y)
	var shift := new_p - old_p
	shift.y = 0.0
	world_dynamic = true
	_install_world(w, true, shift)

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
	# lower body: convex hull with raised overhangs (approach/departure angles) so kerbs meet the
	# wheels, not the body
	var lower := CollisionShape3D.new()
	var hull := ConvexPolygonShape3D.new()
	var hw := 1.02 if model_root else 0.86
	var fz := -2.25 if model_root else -2.3
	var rz := 2.3 if model_root else 2.35
	var pts := PackedVector3Array()
	for sx in [-1.0, 1.0]:
		pts.append(Vector3(sx * (hw - 0.12), -0.13, -1.45))   # floor between the axles
		pts.append(Vector3(sx * (hw - 0.12), -0.13, 1.35))
		pts.append(Vector3(sx * (hw - 0.2), 0.04, fz + 0.05))  # front bumper, lower lip
		pts.append(Vector3(sx * (hw - 0.2), 0.04, rz - 0.05))  # rear bumper, lower lip
		pts.append(Vector3(sx * (hw - 0.25), 0.32, fz))        # bumper tops
		pts.append(Vector3(sx * (hw - 0.25), 0.42, rz))
		pts.append(Vector3(sx * hw, 0.0, -1.7))               # sills
		pts.append(Vector3(sx * hw, 0.0, 1.6))
		pts.append(Vector3(sx * hw, 0.42, -1.6))              # beltline
		pts.append(Vector3(sx * hw, 0.45, 1.6))
	hull.points = pts
	lower.shape = hull
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

func _setup_monitor() -> void:
	monitor = Label.new()
	monitor.position = Vector2(18, 110)
	monitor.add_theme_font_size_override("font_size", 15)
	monitor.add_theme_color_override("font_outline_color", Color(0, 0, 0, 0.9))
	monitor.add_theme_constant_override("outline_size", 5)
	monitor.visible = false
	ui.root.add_child(monitor)

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
	if not spawned and ui:
		ui.show_hint()
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
	# stream ahead of the car: chunks in the direction of travel are built first
	if spawned and not args.has("view"):
		var v: Vector3 = car.linear_velocity
		v.y = 0.0
		streamer.lead = (v * 4.0).limit_length(320.0)
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
	if args.has("ui") and spawned and not ui.menu_open() and not _ui_opened:
		_ui_opened = true
		if args["ui"] != "hud":
			_open_menu(args["ui"])
	if args.has("shot") and args.get("test", "") != "crash":
		_shot_logic()
	if args.get("test", "") == "drive":
		_drive_test(dt)
	if args.get("test", "") == "world":
		_world_test(dt)
	if args.get("test", "") == "crash":
		_crash_test(dt)
	if args.get("test", "") == "traffic":
		_traffic_test(dt)
	if args.get("test", "") == "fetch":
		_fetch_test(dt)
	_check_hop(dt)
	if args.get("test", "") == "city" or args.has("autodrive"):
		_autodrive(dt)

# --------------------------------------------------------------- runtime map loading test
# Start anywhere (data downloaded by the game itself), drive-stream, switch to the next region
# without stopping, then teleport far away.
var _ft := {"phase": 0, "t": 0.0, "w": null}

func _fetch_test(dt: float) -> void:
	_ft["t"] += dt
	if _ft["t"] > 600.0:
		print("[test] fetch: FAIL timeout in phase %d" % _ft["phase"])
		get_tree().quit(1)
		return
	var loaded: bool = spawned and streamer.loading_count() == 0 and streamer.chunks.size() >= 9
	match int(_ft["phase"]):
		0:
			if loaded:
				var info: Dictionary = world.region_info()
				print("[fetch] start region: %d roads, %d buildings, %s" % [info["roads"], info["buildings"], _place])
				if int(info["roads"]) < 50:
					print("[test] fetch: FAIL too few roads")
					get_tree().quit(1)
					return
				_ft["phase"] = 1
				_ft["w"] = world
				var before: Vector3 = car.global_position
				_ft["before_ll"] = world.world_to_latlon(before)
				_hop(before + Vector3(REGION_RADIUS * 0.5, 0, 0))
		1:
			if world != _ft["w"] and not _switching and loaded:
				var drift := _swap_err
				var g: float = world.ground_height(car.global_position)
				var clear := car.global_position.y - g
				print("[fetch] hop: legacy chunks %d, position drift %.2f m, clearance %.2f m" % [streamer.legacy.size(), drift, clear])
				if drift > 5.0 or clear < -0.5 or clear > 3.0:
					print("[test] fetch: FAIL hop moved the car (drift %.1f m, clearance %.2f m)" % [drift, clear])
					get_tree().quit(1)
					return
				_ft["phase"] = 2
				if args.has("lat2"):
					_ft["w"] = world
					_teleport(float(args["lat2"]), float(args["lon2"]))
				else:
					_ft["phase"] = 3
		2:
			if world != _ft["w"] and loaded:
				_ft["phase"] = 3
		3:
			var info: Dictionary = world.region_info()
			print("[test] fetch: %d roads, %d buildings at %.3f, %.3f (%s), %.0f s -> PASS" % [info["roads"], info["buildings"], info["lat"], info["lon"], _place, _ft["t"]])
			get_tree().quit(0)

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
	# fell through the world (free fall without wheel contact): back onto the road
	if spawned:
		if car.telemetry()["contacts"] == 0 and car.linear_velocity.y < -6.0:
			_fall_t += dt
		else:
			_fall_t = 0.0
		if _fall_t > 2.5:
			_fall_t = 0.0
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
		KEY_G:
			car.auto_gear = not car.auto_gear
			ui.toast("Коробка: " + ("автомат" if car.auto_gear else "ручная"), 1.5)
		KEY_M:
			_open_menu("map")
		KEY_F3:
			ui.fps_label.visible = not ui.fps_label.visible
		KEY_T:
			car.tcs_on = not car.tcs_on
			ui.toast("Антипробуксовка " + ("вкл" if car.tcs_on else "выкл"), 1.5)
		KEY_B:
			car.abs_on = not car.abs_on
			ui.toast("ABS " + ("вкл" if car.abs_on else "выкл"), 1.5)
		KEY_N:
			set_time(hour + 1.0)
		KEY_F1:
			_open_menu("help")
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
			if ui.menu_open():
				ui.hide_menu()
			else:
				_open_menu("map" if e.keycode == KEY_TAB else "env")

func _open_menu(page: String) -> void:
	if ui.menu_open() and ui.current_page == page:
		ui.hide_menu()
		return
	var st := {"hour": hour, "radius": streamer.radius, "wet": wet > 0.5, "abs": car.abs_on, "tcs": car.tcs_on, "auto": car.auto_gear, "place_sub": _place}
	var ll: Vector2 = world.world_to_latlon(car.global_position if spawned else streamer.focus)
	st["car_ll"] = ll
	st["place"] = "%.4f, %.4f" % [ll.x, ll.y]
	var f: Vector3 = -car.global_transform.basis.z
	st["heading"] = atan2(f.x, -f.z)
	var a: Vector2 = world.world_to_latlon(Vector3(_region_box.position.x, 0, _region_box.end.y))
	var b: Vector2 = world.world_to_latlon(Vector3(_region_box.end.x, 0, _region_box.position.y))
	st["region_ll"] = [a.x, a.y, b.x, b.y]
	ui.show_menu(page, st)

var _hud_dt := 0.0

func _update_hud() -> void:
	if ui == null:
		return
	var dt := get_process_delta_time()
	_hud_dt += dt
	var d := {"driving": spawned}
	if spawned:
		d["speed"] = absf(car.speed) * 3.6
		d["gear"] = "R" if car.gear < 0 else ("N" if car.gear == 0 else str(car.gear))
		d["rpm"] = car.rpm
		d["auto"] = car.auto_gear
		d["abs"] = car.abs_on
		d["tcs"] = car.tcs_on
		if _hud_dt > 0.25:
			_hud_dt = 0.0
			var r: Dictionary = world.road_at(car.global_position)
			var ll: Vector2 = world.world_to_latlon(car.global_position)
			d["road"] = (r["name"] if r.get("name", "") != "" else "") if not r.is_empty() else ""
			if d["road"] == "":
				d["road"] = "Вне дорог" if r.is_empty() else String(r.get("class", "дорога")).capitalize()
			d["place"] = "%.5f, %.5f   ·   %02d:%02d%s" % [ll.x, ll.y, int(hour), int(fmod(hour, 1.0) * 60), ("   ·   %d км/ч" % r["speed"]) if not r.is_empty() and int(r.get("speed", 0)) > 0 else ""]
	else:
		d["road"] = "Загрузка мира…"
		d["place"] = "%d / %d фрагментов" % [streamer.chunks.size(), streamer.chunks.size() + streamer.loading_count()]
	ui.update_hud(d, dt)

var _ready_since := -1.0
var _ui_opened := false

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
		var pf: Dictionary = streamer.prof
		print("[prof] per chunk: mesh %.1f ms, collision %.1f ms, props %.1f ms" % [pf["mesh"] / pf["n"], pf["col"] / pf["n"], pf["inst"] / pf["n"]])
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
		if float(f.get("width", 7.0)) < 6.0:
			tgt = f["centre"]
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
		if args.has("verbose") and absf(car.speed) < 0.3 and _ad["t"] > 3.0:
			for c in car.debug_contacts:
				print("[ad] contact local=%s normal=%s surface=%d" % [c[0], c[1], c[2]])
			for wi in 4:
				var ww = car.wheels[wi]
				print("[ad] wheel %d contact=%s n=%s load=%.0f fx=%.0f fy=%.0f surf=%d len=%.3f hit=%s" % [wi, ww.contact, ww.hit_n, ww.load, ww.fx, ww.fy, ww.surf, ww.len, ww.hit_pos])
			var sp := get_world_3d().direct_space_state
			for hgt in [-0.25, 0.0, 0.3, 0.7]:
				for side in [-0.9, 0.0, 0.9]:
					var from: Vector3 = car.global_transform * Vector3(side, hgt, -1.8)
					var q := PhysicsRayQueryParameters3D.create(from, from + fwd * 2.0)
					q.exclude = [car.get_rid()]
					var r := sp.intersect_ray(q)
					if not r.is_empty():
						print("[ad] hit h=%.2f side=%.1f at %s surface=%s collider=%s shape=%d" % [hgt, side, r["position"], r["collider"].get_meta("surface", -1), r["collider"].get_parent().name, r["shape"]])
			print("[ad] t=%.1f spd=%.2f thr=%.2f brk=%.2f ang=%.2f to=%.1f tgt=%s rpm=%.0f gear=%d omega=%s" % [_ad["t"], car.speed, car.input_throttle, car.input_brake, ang, to.length(), tgt, car.rpm, car.gear, [car.wheels[2].omega, car.wheels[3].omega]])
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
		print("[autodrive] stuck at %s (lat/lon %s), speed %.1f, gear %d, contacts %d" % [car.global_position, world.world_to_latlon(car.global_position), car.speed, car.gear, car.telemetry()["contacts"]])
		for wi in 4:
			var ww = car.wheels[wi]
			print("[autodrive]   wheel %d contact=%s n=%s load=%.0f fx=%.0f fy=%.0f surf=%d len=%.3f omega=%.1f" % [wi, ww.contact, ww.hit_n, ww.load, ww.fx, ww.fy, ww.surf, ww.len, ww.omega])
		for c in car.debug_contacts:
			print("[autodrive]   body contact local=%s normal=%s surface=%d" % [c[0], c[1], c[2]])
		print("[autodrive]   up=%s lin_vel=%s" % [car.global_transform.basis.y, car.linear_velocity])
		_ad["resets"] += 1
		_ad["stuck"] = 0.0
		_spawn(car.global_position + fwd * 30.0)
	if int(_ad["t"] / 10.0) != int((_ad["t"] - dt) / 10.0):
		print("[autodrive] t=%.0f s dist=%.0f m speed=%.0f km/h pos=%s chunks=%d" % [_ad["t"], _ad["dist"], car.speed * 3.6, car.global_position, streamer.chunks.size()])
	if int(_ad["t"] * 2.0) != int((_ad["t"] - dt) * 2.0):
		var g: float = world.ground_height(car.global_position)
		var clear: float = car.global_position.y - g
		if clear < 0.15:
			print("[autodrive] low clearance %.2f at %s t=%.1f" % [clear, car.global_position, _ad["t"]])
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

# --------------------------------------------------------------- traffic test

var _tt := 0.0
var _tt_dist := {}
var _tt_maxv := 0.0
var _tt_stops := 0

func _traffic_test(dt: float) -> void:
	if not spawned:
		return
	car.input_brake = 1.0
	_tt += dt
	for c in traffic.cars:
		var id := (c["body"] as Node).get_instance_id()
		_tt_dist[id] = _tt_dist.get(id, 0.0) + float(c["speed"]) * dt
		_tt_maxv = maxf(_tt_maxv, c["speed"])
	if int(_tt / 20.0) != int((_tt - dt) / 20.0):
		for i in mini(4, traffic.cars.size()):
			var cp: Vector3 = traffic.cars[i]["pos"]
			print("[traffic] car %d at %s, %.0f m from player, %.0f km/h, height above road %.2f" % [i, cp, cp.distance_to(car.global_position), float(traffic.cars[i]["speed"]) * 3.6, cp.y - world.ground_height(cp)])
	if _tt > 60.0:
		var moving := 0
		for k in _tt_dist:
			if _tt_dist[k] > 50.0:
				moving += 1
		var ok: bool = traffic.cars.size() >= 10 and moving >= 8
		print("[test] traffic: %d cars, %d drove > 50 m, max speed %.0f km/h -> %s" % [traffic.cars.size(), moving, _tt_maxv * 3.6, "PASS" if ok else "FAIL"])
		get_tree().quit(0 if ok else 1)
