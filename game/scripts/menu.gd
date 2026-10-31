## In-game menu (Tab or Esc): teleport to landmarks of Minsk, time of day, rain, view distance, quit.
extends CanvasLayer

signal teleport(lat: float, lon: float)
signal time_changed(hour: float)
signal rain_changed(on: bool)
signal radius_changed(r: float)

const PLACES := [
	["Площадь Победы", 53.9083, 27.5751],
	["Площадь Независимости", 53.8962, 27.5476],
	["Октябрьская площадь", 53.9019, 27.5612],
	["Немига", 53.9056, 27.5536],
	["Верхний город, пл. Свободы", 53.9034, 27.5560],
	["Троицкое предместье", 53.9087, 27.5550],
	["Парк Горького", 53.9035, 27.5717],
	["Площадь Якуба Коласа", 53.9150, 27.5832],
	["Комаровский рынок", 53.9250, 27.5698],
	["Железнодорожный вокзал", 53.8909, 27.5508],
]

var panel: PanelContainer
var time_slider: HSlider
var radius_slider: HSlider
var rain_box: CheckBox

func _ready() -> void:
	layer = 10
	panel = PanelContainer.new()
	panel.set_anchors_preset(Control.PRESET_CENTER)
	panel.custom_minimum_size = Vector2(520, 0)
	var sb := StyleBoxFlat.new()
	sb.bg_color = Color(0.08, 0.09, 0.11, 0.92)
	sb.set_corner_radius_all(10)
	sb.set_content_margin_all(18)
	panel.add_theme_stylebox_override("panel", sb)
	add_child(panel)
	var v := VBoxContainer.new()
	v.add_theme_constant_override("separation", 8)
	panel.add_child(v)
	var title := Label.new()
	title.text = "BeamNG.Earth — Минск"
	title.add_theme_font_size_override("font_size", 24)
	v.add_child(title)
	var tp := Label.new()
	tp.text = "Телепорт"
	tp.add_theme_font_size_override("font_size", 16)
	v.add_child(tp)
	var grid := GridContainer.new()
	grid.columns = 2
	grid.add_theme_constant_override("h_separation", 8)
	grid.add_theme_constant_override("v_separation", 6)
	v.add_child(grid)
	for p in PLACES:
		var b := Button.new()
		b.text = p[0]
		b.custom_minimum_size = Vector2(240, 34)
		b.pressed.connect(func():
			teleport.emit(p[1], p[2])
			hide_menu())
		grid.add_child(b)
	v.add_child(HSeparator.new())
	time_slider = _slider(v, "Время суток", 0.0, 24.0, 0.25, func(x): time_changed.emit(x))
	radius_slider = _slider(v, "Дальность прорисовки, м", 500.0, 2000.0, 50.0, func(x): radius_changed.emit(x))
	rain_box = CheckBox.new()
	rain_box.text = "Дождь (мокрая дорога)"
	rain_box.toggled.connect(func(on): rain_changed.emit(on))
	v.add_child(rain_box)
	v.add_child(HSeparator.new())
	var h := HBoxContainer.new()
	v.add_child(h)
	var close := Button.new()
	close.text = "Продолжить (Tab)"
	close.pressed.connect(hide_menu)
	h.add_child(close)
	var quit := Button.new()
	quit.text = "Выход"
	quit.pressed.connect(func(): get_tree().quit())
	h.add_child(quit)
	visible = false

func _slider(parent: Container, label: String, a: float, b: float, step: float, cb: Callable) -> HSlider:
	var l := Label.new()
	parent.add_child(l)
	var s := HSlider.new()
	s.min_value = a
	s.max_value = b
	s.step = step
	s.custom_minimum_size = Vector2(480, 24)
	s.value_changed.connect(func(x):
		l.text = "%s: %s" % [label, ("%02d:%02d" % [int(x), int(fmod(x, 1.0) * 60)]) if b == 24.0 else str(int(x))]
		cb.call(x))
	parent.add_child(s)
	l.text = label
	return s

func show_menu(hour: float, radius: float, wet: bool) -> void:
	time_slider.set_value_no_signal(hour)
	radius_slider.set_value_no_signal(radius)
	rain_box.set_pressed_no_signal(wet)
	visible = true
	Input.mouse_mode = Input.MOUSE_MODE_VISIBLE

func hide_menu() -> void:
	visible = false
