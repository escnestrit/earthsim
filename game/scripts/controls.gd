## Input devices: keyboard, gamepad and steering wheels. Axes are configured in controls.cfg next to
## the exe (created on first run). F4 shows every axis of every connected device live.
extends RefCounted

const TEMPLATE := """# BeamNG.Earth: руль / геймпад
# Нажмите F4 в игре: увидите все оси устройств и их значения. Покрутите руль, нажмите педали
# и впишите номера осей ниже. invert = 1 переворачивает ось.
# pedal_range: full — педаль выдаёт -1..1 (так у большинства рулей), half — 0..1 (курки геймпада).
[device]
index = 0
[steer]
axis = 0
invert = 0
deadzone = 0.02
[throttle]
axis = 5
invert = 0
pedal_range = "half"
[brake]
axis = 4
invert = 0
pedal_range = "half"
[buttons]
handbrake = 0
shift_up = 5
shift_down = 4
reset = 3
"""

var cfg := ConfigFile.new()
var path := ""
var _prev := {}

func load_config() -> void:
	path = OS.get_executable_path().get_base_dir().path_join("controls.cfg")
	if OS.has_feature("editor") or not DirAccess.dir_exists_absolute(path.get_base_dir()):
		path = "user://controls.cfg"
	if not FileAccess.file_exists(path):
		var f := FileAccess.open(path, FileAccess.WRITE)
		if f:
			f.store_string(TEMPLATE)
			f.close()
	if cfg.load(path) != OK:
		cfg.parse(TEMPLATE)

func _pedal(dev: int, sec: String) -> float:
	var ax: int = cfg.get_value(sec, "axis", -1)
	if ax < 0:
		return 0.0
	var v := Input.get_joy_axis(dev, ax)
	if int(cfg.get_value(sec, "invert", 0)) == 1:
		v = -v
	if String(cfg.get_value(sec, "pedal_range", "half")) == "full":
		v = (v + 1.0) * 0.5
	return clampf(v, 0.0, 1.0)

## Returns {steer, throttle, brake, handbrake, active, pressed:[names]} for the configured device.
func read() -> Dictionary:
	var out := {"steer": 0.0, "throttle": 0.0, "brake": 0.0, "handbrake": 0.0, "active": false, "pressed": []}
	var pads := Input.get_connected_joypads()
	var dev: int = cfg.get_value("device", "index", 0)
	if not pads.has(dev):
		if pads.is_empty():
			return out
		dev = pads[0]
	var sa: int = cfg.get_value("steer", "axis", 0)
	var s := Input.get_joy_axis(dev, sa)
	if int(cfg.get_value("steer", "invert", 0)) == 1:
		s = -s
	var dz: float = cfg.get_value("steer", "deadzone", 0.02)
	if absf(s) < dz:
		s = 0.0
	out["steer"] = s
	out["throttle"] = _pedal(dev, "throttle")
	out["brake"] = _pedal(dev, "brake")
	out["active"] = absf(s) > dz or out["throttle"] > 0.05 or out["brake"] > 0.05
	for name in ["handbrake", "shift_up", "shift_down", "reset"]:
		var b: int = cfg.get_value("buttons", name, -1)
		var now := b >= 0 and Input.is_joy_button_pressed(dev, b)
		if name == "handbrake":
			out["handbrake"] = 1.0 if now else 0.0
		elif now and not _prev.get(name, false):
			out["pressed"].append(name)
		_prev[name] = now
	return out

func monitor_text() -> String:
	var t := "Устройства ввода (F4 — закрыть). Файл настроек: %s\n" % ProjectSettings.globalize_path(path)
	var pads := Input.get_connected_joypads()
	if pads.is_empty():
		return t + "Руль / геймпад не подключен."
	for d in pads:
		t += "\n[%d] %s\n  оси: " % [d, Input.get_joy_name(d)]
		for a in 10:
			t += "%d:%+.2f  " % [a, Input.get_joy_axis(d, a)]
		t += "\n  кнопки: "
		for b in 24:
			if Input.is_joy_button_pressed(d, b):
				t += "%d " % b
	return t
