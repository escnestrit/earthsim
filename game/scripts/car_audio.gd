## Procedural car sound: engine (firing frequency harmonics + intake noise, follows rpm and load),
## tyre squeal from slip, wind from speed. Synthesised into an AudioStreamGenerator.
extends AudioStreamPlayer

var car
const RATE := 22050.0
var _pb: AudioStreamGeneratorPlayback
var _ph := 0.0
var _ph2 := 0.0
var _sq := 0.0
var _rng := RandomNumberGenerator.new()
var _lp := 0.0
var _lp2 := 0.0
var _rpm := 900.0
var _load := 0.0

func _ready() -> void:
	var g := AudioStreamGenerator.new()
	g.mix_rate = RATE
	g.buffer_length = 0.12
	stream = g
	volume_db = -6.0
	play()
	_pb = get_stream_playback()

func _process(_dt: float) -> void:
	if car == null or _pb == null:
		return
	var n := _pb.get_frames_available()
	if n <= 0:
		return
	var target_rpm: float = car.rpm
	var target_load: float = car.input_throttle
	var slip := 0.0
	for w in car.wheels:
		if w.contact:
			slip = maxf(slip, maxf(absf(w.slip_ratio) - 0.12, absf(w.slip_angle) - 0.12))
	var spd: float = absf(car.speed)
	var squeal_amp := clampf(slip * 3.0, 0.0, 0.5) * clampf(spd / 4.0, 0.0, 1.0)
	var wind := clampf(spd / 40.0, 0.0, 1.0)
	var buf := PackedVector2Array()
	buf.resize(n)
	for i in n:
		# glide rpm/load per sample for smooth pitch changes
		_rpm += (target_rpm - _rpm) * 0.002
		_load += (target_load - _load) * 0.001
		var f := _rpm / 60.0 * 2.0  # 4 cylinders: 2 firings per revolution
		_ph = fmod(_ph + f / RATE, 1.0)
		_ph2 = fmod(_ph2 + f * 0.5 / RATE, 1.0)
		var t := TAU * _ph
		var eng := 0.55 * sin(t) + 0.3 * sin(2.0 * t + 0.4) + 0.18 * sin(3.0 * t + 1.1) + 0.12 * sin(TAU * _ph2)
		# combustion roughness, stronger under load
		var nz := _rng.randf_range(-1.0, 1.0)
		_lp += (nz - _lp) * 0.08
		eng += _lp * (0.25 + 0.6 * _load)
		eng *= 0.16 + 0.24 * _load + 0.05 * (_rpm / 7000.0)
		# tyre squeal: band-limited noise around ~900 Hz
		_sq = fmod(_sq + (880.0 + 60.0 * sin(t * 0.01)) / RATE, 1.0)
		var sq := sin(TAU * _sq) * (0.6 + 0.4 * nz) * squeal_amp * 0.35
		# wind: low-passed noise
		_lp2 += (_rng.randf_range(-1.0, 1.0) - _lp2) * 0.02
		var wn := _lp2 * wind * 0.5
		var s := clampf(eng + sq + wn, -1.0, 1.0)
		buf[i] = Vector2(s, s)
	_pb.push_buffer(buf)
