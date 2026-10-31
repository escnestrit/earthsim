## Chase / bonnet / orbit camera for the car.
extends Camera3D

var target: Node3D
var mode := 0              # 0 chase, 1 bonnet, 2 orbit (mouse)
var _yaw := 0.0
var _pitch := -0.25
var _dist := 6.5
var _pos := Vector3.ZERO
var _look := Vector3.ZERO

func _ready() -> void:
	fov = 70.0
	near = 0.1
	far = 4000.0

func cycle() -> void:
	mode = (mode + 1) % 3

func _unhandled_input(e: InputEvent) -> void:
	if mode == 2 and e is InputEventMouseMotion and Input.is_mouse_button_pressed(MOUSE_BUTTON_RIGHT):
		_yaw -= e.relative.x * 0.005
		_pitch = clampf(_pitch - e.relative.y * 0.005, -1.4, 0.3)
	if e is InputEventMouseButton and e.pressed:
		if e.button_index == MOUSE_BUTTON_WHEEL_UP:
			_dist = maxf(3.0, _dist * 0.9)
		elif e.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			_dist = minf(60.0, _dist * 1.1)

func _process(dt: float) -> void:
	if target == null:
		return
	var xf := target.global_transform
	var fwd := -xf.basis.z
	fwd.y = 0.0
	if fwd.length() < 0.1:
		fwd = Vector3.FORWARD
	fwd = fwd.normalized()
	if mode == 1:
		global_transform = Transform3D(xf.basis, xf * Vector3(0, 0.95, -0.2))
		return
	var want_pos: Vector3
	var want_look: Vector3
	if mode == 0:
		want_pos = xf.origin - fwd * _dist + Vector3(0, 2.0 + _dist * 0.12, 0)
		want_look = xf.origin + fwd * 4.0 + Vector3(0, 0.9, 0)
	else:
		var dir := Vector3(sin(_yaw) * cos(_pitch), -sin(_pitch), cos(_yaw) * cos(_pitch))
		want_pos = xf.origin + dir * _dist
		want_look = xf.origin + Vector3(0, 0.8, 0)
	var k := 1.0 - exp(-dt * 8.0)
	if _pos == Vector3.ZERO:
		_pos = want_pos
		_look = want_look
	_pos = _pos.lerp(want_pos, k)
	_look = _look.lerp(want_look, 1.0 - exp(-dt * 14.0))
	global_position = _pos
	if _pos.distance_to(_look) > 0.01:
		look_at(_look, Vector3.UP)
