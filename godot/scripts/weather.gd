# Weather: clear, clouds, overcast, rain, fog, snow. Drives the sky's cloud cover, light and fog, rain or snow
# falling round the camera, wet or snowy ground (shader globals), and the tyres' grip.
extends Node3D

const KINDS := ["Ясно", "Облачно", "Пасмурно", "Дождь", "Ливень", "Туман", "Снег"]

var kind := 0
var env: Environment
var sun: DirectionalLight3D
var stream: EarthStream
var sky_mat: ShaderMaterial
var rain: GPUParticles3D
var snow: GPUParticles3D
var wet := 0.0          # follows the rain slowly (roads dry after it stops)
var snow_cover := 0.0
var cam: Camera3D
var mats  # EarthMaterials: hard surfaces turn glossy when wet
var _last_wet := -1.0
var night := 0.0       # 0 day .. 1 night (from main)
var base_sun := 1.6     # the sun's energy for the time of day (set by main), before clouds

func setup(e: Environment, s: DirectionalLight3D, st: EarthStream, sky: ShaderMaterial, c: Camera3D) -> void:
	env = e
	sun = s
	stream = st
	sky_mat = sky
	cam = c
	rain = _particles(9000, Vector3(0.018, 0.6, 0.018), Color(0.8, 0.84, 0.92, 0.5), 10.5, 0.2)
	snow = _particles(7000, Vector3(0.05, 0.05, 0.05), Color(1, 1, 1, 0.9), 1.3, 1.2)
	add_child(rain)
	add_child(snow)

func _particles(n: int, size: Vector3, col: Color, speed: float, sway: float) -> GPUParticles3D:
	var p := GPUParticles3D.new()
	p.amount = n
	p.lifetime = 3.0
	p.preprocess = 3.0
	p.visibility_aabb = AABB(Vector3(-40, -40, -40), Vector3(80, 80, 80))
	p.local_coords = false
	var m := ParticleProcessMaterial.new()
	m.emission_shape = ParticleProcessMaterial.EMISSION_SHAPE_BOX
	m.emission_box_extents = Vector3(30, 1, 30)
	m.direction = Vector3(0, -1, 0)
	m.spread = 3.0
	m.initial_velocity_min = speed * 0.9
	m.initial_velocity_max = speed * 1.1
	m.gravity = Vector3(0.6, -1.0, 0.3) * sway
	m.turbulence_enabled = sway > 0.5
	m.turbulence_noise_strength = 0.6
	p.process_material = m
	var mesh := QuadMesh.new()
	mesh.size = Vector2(size.x, size.y)
	var mat := StandardMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	mat.albedo_color = col
	mat.billboard_mode = BaseMaterial3D.BILLBOARD_FIXED_Y if size.y > size.x * 3 else BaseMaterial3D.BILLBOARD_ENABLED
	mat.billboard_keep_scale = true
	mesh.material = mat
	p.draw_pass_1 = mesh
	p.emitting = false
	return p

func set_kind(k: int) -> void:
	kind = clampi(k, 0, KINDS.size() - 1)

# sky, light and fog for the weather; day is 0..1 (how much sun there is)
func _process(dt: float) -> void:
	if env == null:
		return
	var cloud: float = [0.25, 0.55, 0.95, 0.9, 1.0, 0.85, 0.95][kind]
	var raining: bool = kind == 3 or kind == 4
	var snowing: bool = kind == 6
	if sky_mat:
		sky_mat.set_shader_parameter("cloud_cover", cloud)
	# overcast: the sun goes soft and dim, the fill light takes over
	var dull: float = clampf((cloud - 0.6) / 0.4, 0.0, 1.0)
	sun.light_energy = base_sun * lerpf(1.0, 0.3, dull)
	sun.shadow_opacity = lerpf(1.0, 0.45, dull)
	env.fog_density = [0.0006, 0.0007, 0.0012, 0.0025, 0.0045, 0.012, 0.004][kind]
	env.fog_light_color = Color(0.7, 0.78, 0.9).lerp(Color(0.62, 0.64, 0.66), dull).lerp(Color(0.02, 0.025, 0.04), night)
	# ground wetness lags the rain: wet in a minute, dry in a few
	var wt := 1.0 if raining else 0.0
	wet = move_toward(wet, wt, dt * (0.03 if raining else 0.006))
	var st := 1.0 if snowing else 0.0
	snow_cover = move_toward(snow_cover, st, dt * (0.02 if snowing else 0.003))
	RenderingServer.global_shader_parameter_set("wetness", wet)
	RenderingServer.global_shader_parameter_set("snow_cover", snow_cover)
	if stream:
		stream.set_weather_grip(wet, snow_cover)
	if mats and absf(wet - _last_wet) > 0.01:
		_last_wet = wet
		mats.set_wet(wet)
	rain.emitting = raining
	rain.amount_ratio = 1.0 if kind == 4 else 0.45
	snow.emitting = snowing
	if cam:
		var p := cam.global_position + Vector3(0, 14, 0)
		rain.global_position = p
		snow.global_position = p
