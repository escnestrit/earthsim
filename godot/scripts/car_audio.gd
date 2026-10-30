# Procedural car sounds: engine (firing harmonics + intake noise, following rpm and throttle), tyre squeal
# from wheel slip, wind at speed, and a thump on impacts. No recorded samples needed.
extends Node
class_name CarAudio

const RATE := 22050.0

var player: AudioStreamPlayer
var playback: AudioStreamGeneratorPlayback
var car: Node3D
var phase := 0.0
var phase2 := 0.0
var rpm_s := 800.0
var load_s := 0.0
var squeal := 0.0
var squeal_phase := 0.0
var wind := 0.0
var thump := 0.0
var last_damage := 0.0
var noise_lp := 0.0
var noise_lp2 := 0.0
var cylinders := 4.0
var last_speed := 0.0

func _ready() -> void:
	player = AudioStreamPlayer.new()
	var gen := AudioStreamGenerator.new()
	gen.mix_rate = RATE
	gen.buffer_length = 0.12
	player.stream = gen
	player.volume_db = -6.0
	add_child(player)
	player.play()
	playback = player.get_stream_playback()

func _process(dt: float) -> void:
	if car == null or playback == null:
		return
	var rpm: float = car.rpm()
	var speed: float = car.speed_kmh()
	# throttle is not exposed by every car type: infer load from acceleration and revs
	var accel := (speed - last_speed) / maxf(dt, 1e-3)
	last_speed = speed
	var load_t := clampf(accel / 12.0 + (0.3 if Input.is_key_pressed(KEY_W) or Input.is_key_pressed(KEY_UP) else 0.0), 0.0, 1.0)
	load_s = lerpf(load_s, load_t, clampf(dt * 6.0, 0.0, 1.0))
	rpm_s = lerpf(rpm_s, maxf(rpm, 600.0), clampf(dt * 12.0, 0.0, 1.0))
	wind = clampf((speed - 30.0) / 120.0, 0.0, 1.0)
	var dmg: float = car.damage()
	if dmg > last_damage + 0.02:
		thump = clampf((dmg - last_damage) * 4.0, 0.3, 1.0)
	last_damage = dmg
	# hard braking or sliding: squeal (a rough stand-in until all cars report wheel slip)
	var sq_t := 0.0
	if speed > 15.0 and (Input.is_key_pressed(KEY_SPACE) or accel < -7.0):
		sq_t = 0.6
	squeal = lerpf(squeal, sq_t, clampf(dt * 8.0, 0.0, 1.0))
	_fill()

func _fill() -> void:
	var n := playback.get_frames_available()
	if n <= 0:
		return
	var f := rpm_s / 60.0 * cylinders / 2.0   # firing frequency
	var vol := 0.18 + 0.32 * load_s
	var buf := PackedVector2Array()
	buf.resize(n)
	for i in n:
		phase = fmod(phase + f / RATE, 1.0)
		phase2 = fmod(phase2 + f * 0.5 / RATE, 1.0)
		# pulse-like firing wave with a sub-harmonic (uneven cylinders), soft-clipped
		var p := phase * TAU
		var e := sin(p) * 0.6 + sin(2.0 * p) * 0.25 * (0.5 + load_s) + sin(3.0 * p) * 0.12 + sin(phase2 * TAU) * 0.3
		var nz := randf() * 2.0 - 1.0
		noise_lp += (nz - noise_lp) * 0.25
		noise_lp2 += (nz - noise_lp2) * 0.05
		e += noise_lp * (0.15 + 0.35 * load_s)
		e = tanh(e * (1.2 + load_s)) * vol
		# tyres: a squeal around 700-900 Hz
		squeal_phase = fmod(squeal_phase + (780.0 + noise_lp2 * 120.0) / RATE, 1.0)
		var s := sin(squeal_phase * TAU) * squeal * 0.2
		# wind: low-passed noise
		var w := noise_lp2 * wind * 0.6
		# impact: decaying low thump
		var t := 0.0
		if thump > 0.001:
			t = sin(float(i) * 0.02) * thump * 0.8 + nz * thump * 0.3
			thump *= 0.9993
		var v := clampf(e + s + w + t, -1.0, 1.0)
		buf[i] = Vector2(v, v)
	playback.push_buffer(buf)
