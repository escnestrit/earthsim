## The whole user interface: HUD (speedometer, location, streaming status, toasts), pause menu
## with pages (drive, world map, environment, settings, controls) and the loading screen.
extends CanvasLayer

signal teleport(lat: float, lon: float)
signal time_changed(hour: float)
signal rain_changed(on: bool)
signal radius_changed(r: float)
signal assist_changed(name: String, on: bool)
signal respawn
signal quit_game

const MapView := preload("res://scripts/map_view.gd")
const ACCENT := Color(0.33, 0.66, 1.0)
const PANEL_BG := Color(0.07, 0.08, 0.1, 0.9)

# name (ru), name (en), lat, lon
const CITIES := [
	["Минск", "Minsk", 53.9024, 27.5618], ["Брест", "Brest", 52.0976, 23.7341], ["Гродно", "Grodno", 53.6778, 23.8295],
	["Гомель", "Gomel", 52.4345, 30.9754], ["Витебск", "Vitebsk", 55.1904, 30.2049], ["Могилёв", "Mogilev", 53.8945, 30.3307],
	["Москва", "Moscow", 55.7558, 37.6173], ["Санкт-Петербург", "Saint Petersburg", 59.9343, 30.3351], ["Киев", "Kyiv", 50.4501, 30.5234],
	["Вильнюс", "Vilnius", 54.6872, 25.2797], ["Рига", "Riga", 56.9496, 24.1052], ["Таллин", "Tallinn", 59.437, 24.7536],
	["Варшава", "Warsaw", 52.2297, 21.0122], ["Краков", "Krakow", 50.0647, 19.945], ["Прага", "Prague", 50.0755, 14.4378],
	["Вена", "Vienna", 48.2082, 16.3738], ["Будапешт", "Budapest", 47.4979, 19.0402], ["Берлин", "Berlin", 52.52, 13.405],
	["Мюнхен", "Munich", 48.1351, 11.582], ["Гамбург", "Hamburg", 53.5511, 9.9937], ["Амстердам", "Amsterdam", 52.3676, 4.9041],
	["Брюссель", "Brussels", 50.8503, 4.3517], ["Париж", "Paris", 48.8566, 2.3522], ["Лион", "Lyon", 45.764, 4.8357],
	["Лондон", "London", 51.5074, -0.1278], ["Эдинбург", "Edinburgh", 55.9533, -3.1883], ["Дублин", "Dublin", 53.3498, -6.2603],
	["Мадрид", "Madrid", 40.4168, -3.7038], ["Барселона", "Barcelona", 41.3874, 2.1686], ["Лиссабон", "Lisbon", 38.7223, -9.1393],
	["Рим", "Rome", 41.9028, 12.4964], ["Милан", "Milan", 45.4642, 9.19], ["Венеция", "Venice", 45.4408, 12.3155],
	["Афины", "Athens", 37.9838, 23.7275], ["Стамбул", "Istanbul", 41.0082, 28.9784], ["Стокгольм", "Stockholm", 59.3293, 18.0686],
	["Осло", "Oslo", 59.9139, 10.7522], ["Хельсинки", "Helsinki", 60.1699, 24.9384], ["Копенгаген", "Copenhagen", 55.6761, 12.5683],
	["Цюрих", "Zurich", 47.3769, 8.5417], ["Монако", "Monaco", 43.7384, 7.4246], ["Нюрбургринг", "Nurburgring", 50.3356, 6.9475],
	["Тбилиси", "Tbilisi", 41.7151, 44.8271], ["Ереван", "Yerevan", 40.1792, 44.4991], ["Баку", "Baku", 40.4093, 49.8671],
	["Алматы", "Almaty", 43.222, 76.8512], ["Ташкент", "Tashkent", 41.2995, 69.2401], ["Новосибирск", "Novosibirsk", 55.0084, 82.9357],
	["Владивосток", "Vladivostok", 43.1155, 131.8855], ["Казань", "Kazan", 55.7961, 49.1064], ["Сочи", "Sochi", 43.6028, 39.7342],
	["Нью-Йорк", "New York", 40.758, -73.9855], ["Лос-Анджелес", "Los Angeles", 34.0522, -118.2437], ["Сан-Франциско", "San Francisco", 37.7749, -122.4194],
	["Чикаго", "Chicago", 41.8781, -87.6298], ["Лас-Вегас", "Las Vegas", 36.1699, -115.1398], ["Майами", "Miami", 25.7617, -80.1918],
	["Детройт", "Detroit", 42.3314, -83.0458], ["Сиэтл", "Seattle", 47.6062, -122.3321], ["Торонто", "Toronto", 43.6532, -79.3832],
	["Ванкувер", "Vancouver", 49.2827, -123.1207], ["Мехико", "Mexico City", 19.4326, -99.1332], ["Гавана", "Havana", 23.1136, -82.3666],
	["Рио-де-Жанейро", "Rio de Janeiro", -22.9068, -43.1729], ["Сан-Паулу", "Sao Paulo", -23.5505, -46.6333], ["Буэнос-Айрес", "Buenos Aires", -34.6037, -58.3816],
	["Лима", "Lima", -12.0464, -77.0428], ["Богота", "Bogota", 4.711, -74.0721], ["Сантьяго", "Santiago", -33.4489, -70.6693],
	["Каир", "Cairo", 30.0444, 31.2357], ["Марракеш", "Marrakesh", 31.6295, -7.9811], ["Найроби", "Nairobi", -1.2921, 36.8219],
	["Лагос", "Lagos", 6.5244, 3.3792], ["Кейптаун", "Cape Town", -33.9249, 18.4241], ["Йоханнесбург", "Johannesburg", -26.2041, 28.0473],
	["Дубай", "Dubai", 25.2048, 55.2708], ["Эр-Рияд", "Riyadh", 24.7136, 46.6753], ["Тегеран", "Tehran", 35.6892, 51.389],
	["Тель-Авив", "Tel Aviv", 32.0853, 34.7818], ["Мумбаи", "Mumbai", 19.076, 72.8777], ["Дели", "Delhi", 28.6139, 77.209],
	["Бангкок", "Bangkok", 13.7563, 100.5018], ["Сингапур", "Singapore", 1.3521, 103.8198], ["Джакарта", "Jakarta", -6.2088, 106.8456],
	["Ханой", "Hanoi", 21.0278, 105.8342], ["Гонконг", "Hong Kong", 22.3193, 114.1694], ["Шанхай", "Shanghai", 31.2304, 121.4737],
	["Пекин", "Beijing", 39.9042, 116.4074], ["Сеул", "Seoul", 37.5665, 126.978], ["Токио", "Tokyo", 35.6762, 139.6503],
	["Осака", "Osaka", 34.6937, 135.5023], ["Сидней", "Sydney", -33.8688, 151.2093], ["Мельбурн", "Melbourne", -37.8136, 144.9631],
	["Окленд", "Auckland", -36.8485, 174.7633], ["Рейкьявик", "Reykjavik", 64.1466, -21.9426], ["Анкоридж", "Anchorage", 61.2181, -149.9003],
]

var root: Control
# HUD
var hud: Control
var speedo: Control
var loc_label: Label
var loc_sub: Label
var status_box: PanelContainer
var status_label: Label
var toast_box: PanelContainer
var toast_label: Label
var _toast_t := 0.0
var hint: Label
var _hint_t := 12.0
var fps_label: Label
var hud_data := {"speed": 0.0, "gear": "N", "rpm": 0.0, "max_rpm": 7000.0, "auto": true, "abs": true, "tcs": true}
# menu
var menu: Control
var pages := {}
var page_buttons := {}
var current_page := ""
var map: Control
var map_info: Label
var go_button: Button
var search: LineEdit
var results: ItemList
var _found := []
var _pick := Vector2.INF
var time_slider: HSlider
var radius_slider: HSlider
var rain_check: CheckButton
var abs_check: CheckButton
var tcs_check: CheckButton
var auto_check: CheckButton
var place_title: Label
var place_sub: Label
# loading
var loading: Control
var load_title: Label
var load_text: Label
var load_bar: ProgressBar

var fetcher: Node

func _init() -> void:
	layer = 5

func setup(f: Node) -> void:
	fetcher = f
	root = Control.new()
	root.set_anchors_preset(Control.PRESET_FULL_RECT)
	root.mouse_filter = Control.MOUSE_FILTER_IGNORE
	root.theme = _make_theme()
	add_child(root)
	_build_hud()
	_build_menu()
	_build_loading()

# ------------------------------------------------------------------ theme

func _box(bg: Color, radius := 12, border := Color(1, 1, 1, 0.07), pad := 14) -> StyleBoxFlat:
	var s := StyleBoxFlat.new()
	s.bg_color = bg
	s.set_corner_radius_all(radius)
	s.border_color = border
	s.set_border_width_all(1 if border.a > 0.0 else 0)
	s.set_content_margin_all(pad)
	s.anti_aliasing = true
	return s

func _make_theme() -> Theme:
	var t := Theme.new()
	t.default_font_size = 16
	t.set_stylebox("panel", "PanelContainer", _box(PANEL_BG, 14))
	t.set_stylebox("panel", "Panel", _box(PANEL_BG, 14))
	var bn := _box(Color(1, 1, 1, 0.05), 9, Color(1, 1, 1, 0.06), 10)
	var bh := _box(Color(1, 1, 1, 0.12), 9, Color(1, 1, 1, 0.12), 10)
	var bp := _box(Color(ACCENT, 0.85), 9, Color(ACCENT, 1.0), 10)
	var bd := _box(Color(1, 1, 1, 0.02), 9, Color(1, 1, 1, 0.03), 10)
	t.set_stylebox("normal", "Button", bn)
	t.set_stylebox("hover", "Button", bh)
	t.set_stylebox("pressed", "Button", bp)
	t.set_stylebox("hover_pressed", "Button", bp)
	t.set_stylebox("disabled", "Button", bd)
	t.set_stylebox("focus", "Button", StyleBoxEmpty.new())
	t.set_color("font_color", "Button", Color(0.9, 0.92, 0.95))
	t.set_color("font_hover_color", "Button", Color.WHITE)
	t.set_color("font_pressed_color", "Button", Color.WHITE)
	t.set_color("font_color", "Label", Color(0.9, 0.92, 0.95))
	t.set_stylebox("normal", "LineEdit", _box(Color(1, 1, 1, 0.06), 9, Color(1, 1, 1, 0.1), 10))
	t.set_stylebox("focus", "LineEdit", _box(Color(1, 1, 1, 0.06), 9, Color(ACCENT, 0.8), 10))
	t.set_stylebox("panel", "ItemList", _box(Color(1, 1, 1, 0.03), 9, Color(1, 1, 1, 0.06), 6))
	t.set_stylebox("selected", "ItemList", _box(Color(ACCENT, 0.5), 6, Color(0, 0, 0, 0), 4))
	t.set_stylebox("selected_focus", "ItemList", _box(Color(ACCENT, 0.6), 6, Color(0, 0, 0, 0), 4))
	t.set_stylebox("hovered", "ItemList", _box(Color(1, 1, 1, 0.08), 6, Color(0, 0, 0, 0), 4))
	t.set_stylebox("focus", "ItemList", StyleBoxEmpty.new())
	t.set_constant("v_separation", "ItemList", 6)
	var sl := StyleBoxFlat.new()
	sl.bg_color = Color(1, 1, 1, 0.12)
	sl.set_corner_radius_all(3)
	sl.content_margin_top = 3
	sl.content_margin_bottom = 3
	t.set_stylebox("slider", "HSlider", sl)
	var ga := sl.duplicate()
	ga.bg_color = ACCENT
	t.set_stylebox("grabber_area", "HSlider", ga)
	t.set_stylebox("grabber_area_highlight", "HSlider", ga)
	var pb := _box(Color(1, 1, 1, 0.1), 6, Color(0, 0, 0, 0), 0)
	var pf := _box(ACCENT, 6, Color(0, 0, 0, 0), 0)
	t.set_stylebox("background", "ProgressBar", pb)
	t.set_stylebox("fill", "ProgressBar", pf)
	return t

func _label(text: String, size := 16, col := Color(0.9, 0.92, 0.95)) -> Label:
	var l := Label.new()
	l.text = text
	l.add_theme_font_size_override("font_size", size)
	l.add_theme_color_override("font_color", col)
	return l

# ------------------------------------------------------------------ HUD

func _build_hud() -> void:
	hud = Control.new()
	hud.set_anchors_preset(Control.PRESET_FULL_RECT)
	hud.mouse_filter = Control.MOUSE_FILTER_IGNORE
	root.add_child(hud)
	# location pill (top left)
	var lp := PanelContainer.new()
	lp.add_theme_stylebox_override("panel", _box(Color(0.06, 0.07, 0.09, 0.62), 12, Color(1, 1, 1, 0.05), 12))
	lp.position = Vector2(18, 16)
	lp.mouse_filter = Control.MOUSE_FILTER_IGNORE
	hud.add_child(lp)
	var lv := VBoxContainer.new()
	lv.add_theme_constant_override("separation", 0)
	lp.add_child(lv)
	loc_label = _label("", 19, Color.WHITE)
	loc_sub = _label("", 13, Color(0.75, 0.79, 0.85))
	lv.add_child(loc_label)
	lv.add_child(loc_sub)
	# streaming status (top centre)
	status_box = PanelContainer.new()
	status_box.add_theme_stylebox_override("panel", _box(Color(0.06, 0.07, 0.09, 0.7), 18, Color(ACCENT, 0.4), 10))
	status_box.set_anchors_preset(Control.PRESET_CENTER_TOP)
	status_box.position.y = 16
	status_box.mouse_filter = Control.MOUSE_FILTER_IGNORE
	status_box.visible = false
	hud.add_child(status_box)
	status_label = _label("", 14)
	status_box.add_child(status_label)
	# toast
	toast_box = PanelContainer.new()
	toast_box.add_theme_stylebox_override("panel", _box(Color(0.06, 0.07, 0.09, 0.8), 12, Color(1, 1, 1, 0.08), 12))
	toast_box.set_anchors_preset(Control.PRESET_CENTER_BOTTOM)
	toast_box.position.y = -90
	toast_box.mouse_filter = Control.MOUSE_FILTER_IGNORE
	toast_box.visible = false
	hud.add_child(toast_box)
	toast_label = _label("", 16, Color.WHITE)
	toast_box.add_child(toast_label)
	# speedometer
	speedo = Control.new()
	speedo.set_anchors_preset(Control.PRESET_BOTTOM_RIGHT)
	speedo.custom_minimum_size = Vector2(230, 230)
	speedo.size = Vector2(230, 230)
	speedo.position = Vector2(-250, -250)
	speedo.mouse_filter = Control.MOUSE_FILTER_IGNORE
	speedo.draw.connect(_draw_speedo.bind(speedo))
	hud.add_child(speedo)
	# hint
	hint = _label("Esc — меню   ·   M — карта мира   ·   F1 — управление", 14, Color(1, 1, 1, 0.75))
	hint.set_anchors_preset(Control.PRESET_BOTTOM_LEFT)
	hint.position = Vector2(20, -38)
	hint.add_theme_color_override("font_outline_color", Color(0, 0, 0, 0.6))
	hint.add_theme_constant_override("outline_size", 4)
	hud.add_child(hint)
	fps_label = _label("", 12, Color(1, 1, 1, 0.55))
	fps_label.set_anchors_preset(Control.PRESET_TOP_RIGHT)
	fps_label.position = Vector2(-90, 10)
	fps_label.visible = false
	hud.add_child(fps_label)

func _draw_speedo(c: Control) -> void:
	var ctr := c.size * 0.5
	var r := 100.0
	var font := c.get_theme_default_font()
	var a0 := deg_to_rad(135.0)
	var a1 := deg_to_rad(405.0)
	var sp: float = hud_data["speed"]
	var vmax := 240.0 if sp < 200.0 else 320.0
	c.draw_circle(ctr, r + 12, Color(0.05, 0.06, 0.08, 0.62))
	c.draw_arc(ctr, r, a0, a1, 64, Color(1, 1, 1, 0.14), 8.0, true)
	var t := clampf(sp / vmax, 0.0, 1.0)
	if t > 0.002:
		c.draw_arc(ctr, r, a0, lerpf(a0, a1, t), 64, ACCENT, 8.0, true)
	# rpm: inner thin arc, red near the limit
	var rt := clampf(float(hud_data["rpm"]) / float(hud_data["max_rpm"]), 0.0, 1.0)
	c.draw_arc(ctr, r - 16, a0, a1, 64, Color(1, 1, 1, 0.08), 3.0, true)
	if rt > 0.002:
		c.draw_arc(ctr, r - 16, a0, lerpf(a0, a1, rt), 64, Color(1.0, 0.35, 0.3) if rt > 0.88 else Color(1, 1, 1, 0.75), 3.0, true)
	for i in range(0, int(vmax) + 1, 20):
		var a := lerpf(a0, a1, i / vmax)
		var d := Vector2(cos(a), sin(a))
		c.draw_line(ctr + d * (r + 6), ctr + d * (r + (12 if i % 40 == 0 else 9)), Color(1, 1, 1, 0.5), 2.0)
	var s := str(int(round(sp)))
	c.draw_string(font, ctr + Vector2(-70, 14), s, HORIZONTAL_ALIGNMENT_CENTER, 140, 52, Color.WHITE)
	c.draw_string(font, ctr + Vector2(-70, 38), "км/ч", HORIZONTAL_ALIGNMENT_CENTER, 140, 14, Color(1, 1, 1, 0.6))
	c.draw_string(font, ctr + Vector2(-70, 78), String(hud_data["gear"]), HORIZONTAL_ALIGNMENT_CENTER, 140, 30, ACCENT)
	var flags := ("A" if hud_data["auto"] else "M") + ("   ABS" if hud_data["abs"] else "") + ("   TCS" if hud_data["tcs"] else "")
	c.draw_string(font, ctr + Vector2(-70, -40), flags, HORIZONTAL_ALIGNMENT_CENTER, 140, 12, Color(1, 1, 1, 0.5))

func update_hud(d: Dictionary, dt: float) -> void:
	for k in d:
		hud_data[k] = d[k]
	speedo.visible = d.get("driving", true)
	speedo.queue_redraw()
	loc_label.text = hud_data.get("road", "")
	loc_sub.text = hud_data.get("place", "")
	fps_label.text = "%d FPS" % Engine.get_frames_per_second()
	if _hint_t > 0.0:
		_hint_t -= dt
		hint.modulate.a = clampf(_hint_t / 2.0, 0.0, 1.0)
		hint.visible = _hint_t > 0.0
	if _toast_t > 0.0:
		_toast_t -= dt
		toast_box.modulate.a = clampf(_toast_t / 0.5, 0.0, 1.0)
		toast_box.visible = _toast_t > 0.0
	toast_box.position.x = (root.size.x - toast_box.size.x) * 0.5
	status_box.position.x = (root.size.x - status_box.size.x) * 0.5

func set_status(text: String) -> void:
	status_box.visible = text != ""
	status_label.text = text

func toast(text: String, secs := 3.0) -> void:
	toast_label.text = text
	toast_box.reset_size()
	toast_box.visible = true
	_toast_t = secs

func show_hint() -> void:
	_hint_t = 8.0
	hint.visible = true

# ------------------------------------------------------------------ menu

func _build_menu() -> void:
	menu = Control.new()
	menu.set_anchors_preset(Control.PRESET_FULL_RECT)
	menu.visible = false
	root.add_child(menu)
	var dim := ColorRect.new()
	dim.color = Color(0.02, 0.03, 0.05, 0.55)
	dim.set_anchors_preset(Control.PRESET_FULL_RECT)
	menu.add_child(dim)
	var h := HBoxContainer.new()
	h.set_anchors_preset(Control.PRESET_FULL_RECT)
	h.offset_left = 24
	h.offset_top = 24
	h.offset_right = -24
	h.offset_bottom = -24
	h.add_theme_constant_override("separation", 18)
	menu.add_child(h)
	# sidebar
	var side := PanelContainer.new()
	side.custom_minimum_size = Vector2(270, 0)
	h.add_child(side)
	var sv := VBoxContainer.new()
	sv.add_theme_constant_override("separation", 8)
	side.add_child(sv)
	sv.add_child(_label("BeamNG.Earth", 26, Color.WHITE))
	place_title = _label("", 15, Color(0.85, 0.88, 0.92))
	place_title.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	sv.add_child(place_title)
	place_sub = _label("", 13, Color(0.65, 0.7, 0.76))
	place_sub.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	sv.add_child(place_sub)
	var sp := Control.new()
	sp.custom_minimum_size = Vector2(0, 10)
	sv.add_child(sp)
	for item in [["drive", "Продолжить"], ["map", "Карта мира"], ["env", "Время и погода"], ["settings", "Настройки"], ["help", "Управление"]]:
		var b := Button.new()
		b.text = item[1]
		b.alignment = HORIZONTAL_ALIGNMENT_LEFT
		b.toggle_mode = item[0] != "drive"
		b.custom_minimum_size = Vector2(0, 42)
		b.add_theme_font_size_override("font_size", 17)
		var id: String = item[0]
		b.pressed.connect(func():
			if id == "drive":
				hide_menu()
			else:
				open_page(id))
		sv.add_child(b)
		page_buttons[id] = b
	var fill := Control.new()
	fill.size_flags_vertical = Control.SIZE_EXPAND_FILL
	sv.add_child(fill)
	var rs := Button.new()
	rs.text = "Вернуть машину на дорогу"
	rs.custom_minimum_size = Vector2(0, 38)
	rs.pressed.connect(func():
		respawn.emit()
		hide_menu())
	sv.add_child(rs)
	var q := Button.new()
	q.text = "Выйти из игры"
	q.custom_minimum_size = Vector2(0, 38)
	q.pressed.connect(func(): quit_game.emit())
	sv.add_child(q)
	# content
	var content := PanelContainer.new()
	content.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	h.add_child(content)
	var stack := Control.new()
	content.add_child(stack)
	pages["map"] = _page_map()
	pages["env"] = _page_env()
	pages["settings"] = _page_settings()
	pages["help"] = _page_help()
	for k in pages:
		var pg: Control = pages[k]
		pg.set_anchors_preset(Control.PRESET_FULL_RECT)
		pg.visible = false
		stack.add_child(pg)

func _title(v: Container, text: String) -> void:
	v.add_child(_label(text, 22, Color.WHITE))

func _page_map() -> Control:
	var v := VBoxContainer.new()
	v.add_theme_constant_override("separation", 10)
	var top := HBoxContainer.new()
	top.add_theme_constant_override("separation", 10)
	v.add_child(top)
	var tl := _label("Карта мира", 22, Color.WHITE)
	tl.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	top.add_child(tl)
	search = LineEdit.new()
	search.placeholder_text = "Город или координаты: 48.8584, 2.2945"
	search.custom_minimum_size = Vector2(380, 0)
	search.text_changed.connect(_on_search)
	search.text_submitted.connect(func(_t): _on_search_submit())
	top.add_child(search)
	var body := HBoxContainer.new()
	body.size_flags_vertical = Control.SIZE_EXPAND_FILL
	body.add_theme_constant_override("separation", 10)
	v.add_child(body)
	var frame := Control.new()
	frame.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	frame.size_flags_vertical = Control.SIZE_EXPAND_FILL
	frame.clip_contents = true
	body.add_child(frame)
	map = MapView.new()
	map.fetcher = fetcher
	map.picked.connect(_on_pick)
	map.set_anchors_preset(Control.PRESET_FULL_RECT)
	frame.add_child(map)
	# zoom buttons over the map
	var zb := VBoxContainer.new()
	zb.anchor_left = 1.0
	zb.anchor_right = 1.0
	zb.offset_left = -56
	zb.offset_top = 12
	for z in [["+", 1.0], ["−", -1.0]]:
		var b := Button.new()
		b.text = z[0]
		b.custom_minimum_size = Vector2(40, 40)
		b.add_theme_font_size_override("font_size", 22)
		var dz: float = z[1]
		b.pressed.connect(func(): map.zoom_by(dz))
		zb.add_child(b)
	frame.add_child(zb)
	var attr := _label("Рельеф: Mapzen / AWS Terrain Tiles · Дороги и здания: © OpenStreetMap, Overture Maps", 11, Color(1, 1, 1, 0.7))
	attr.anchor_left = 1.0
	attr.anchor_right = 1.0
	attr.anchor_top = 1.0
	attr.anchor_bottom = 1.0
	attr.grow_horizontal = Control.GROW_DIRECTION_BEGIN
	attr.grow_vertical = Control.GROW_DIRECTION_BEGIN
	attr.offset_right = -10
	attr.offset_bottom = -6
	attr.add_theme_color_override("font_outline_color", Color(0, 0, 0, 0.7))
	attr.add_theme_constant_override("outline_size", 3)
	frame.add_child(attr)
	# right column: results + selection
	var col := VBoxContainer.new()
	col.custom_minimum_size = Vector2(290, 0)
	col.add_theme_constant_override("separation", 10)
	body.add_child(col)
	results = ItemList.new()
	results.size_flags_vertical = Control.SIZE_EXPAND_FILL
	results.item_selected.connect(_on_result)
	col.add_child(results)
	map_info = _label("Кликни по карте или выбери город", 14, Color(0.8, 0.84, 0.9))
	map_info.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	col.add_child(map_info)
	go_button = Button.new()
	go_button.text = "Поехать сюда"
	go_button.custom_minimum_size = Vector2(0, 48)
	go_button.add_theme_font_size_override("font_size", 18)
	go_button.disabled = true
	go_button.pressed.connect(func():
		if _pick != Vector2.INF:
			teleport.emit(_pick.x, _pick.y)
			hide_menu())
	col.add_child(go_button)
	var car_b := Button.new()
	car_b.text = "Показать машину"
	car_b.pressed.connect(func():
		if map.car_ll != Vector2.INF:
			map.look_at_ll(map.car_ll.x, map.car_ll.y, maxf(map.zoom, 13.0)))
	col.add_child(car_b)
	_on_search("")
	return v

func _on_search(text: String) -> void:
	results.clear()
	_found = []
	var q := text.strip_edges().to_lower()
	for c in CITIES:
		if q == "" or String(c[0]).to_lower().contains(q) or String(c[1]).to_lower().contains(q):
			_found.append(c)
			results.add_item("%s  ·  %s" % [c[0], c[1]] if c[0] != c[1] else c[0])

func _parse_coords(t: String) -> Vector2:
	var re := RegEx.create_from_string("(-?\\d+(?:[.,]\\d+)?)[\\s,;]+(-?\\d+(?:[.,]\\d+)?)")
	var m := re.search(t.replace(", ", " "))
	if m == null:
		return Vector2.INF
	var la := float(m.get_string(1).replace(",", "."))
	var lo := float(m.get_string(2).replace(",", "."))
	if absf(la) > 85.0 or absf(lo) > 180.0:
		return Vector2.INF
	return Vector2(la, lo)

func _on_search_submit() -> void:
	var ll := _parse_coords(search.text)
	if ll != Vector2.INF:
		map.look_at_ll(ll.x, ll.y, 14.0)
		map.target_ll = ll
		_on_pick(ll.x, ll.y)
	elif not _found.is_empty():
		results.select(0)
		_on_result(0)

func _on_result(i: int) -> void:
	var c: Array = _found[i]
	map.look_at_ll(c[2], c[3], 13.5)
	map.target_ll = Vector2(c[2], c[3])
	_on_pick(c[2], c[3])

func _on_pick(lat: float, lon: float) -> void:
	_pick = Vector2(lat, lon)
	go_button.disabled = false
	map_info.text = "Точка: %.5f, %.5f\nКарта района загрузится из открытых данных (нужен интернет; 10–60 с)." % [lat, lon]

func _row(v: Container, text: String, ctrl: Control) -> void:
	var h := HBoxContainer.new()
	var l := _label(text, 16)
	l.custom_minimum_size = Vector2(260, 0)
	h.add_child(l)
	ctrl.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	h.add_child(ctrl)
	v.add_child(h)

func _page_env() -> Control:
	var v := VBoxContainer.new()
	v.add_theme_constant_override("separation", 16)
	_title(v, "Время и погода")
	var tl := _label("", 15, Color(0.75, 0.8, 0.86))
	time_slider = HSlider.new()
	time_slider.min_value = 0
	time_slider.max_value = 24
	time_slider.step = 0.25
	time_slider.value_changed.connect(func(x):
		tl.text = "%02d:%02d" % [int(x), int(fmod(x, 1.0) * 60)]
		time_changed.emit(x))
	_row(v, "Время суток", time_slider)
	v.add_child(tl)
	rain_check = CheckButton.new()
	rain_check.text = "Дождь, мокрый асфальт"
	rain_check.toggled.connect(func(on): rain_changed.emit(on))
	v.add_child(rain_check)
	var quick := HBoxContainer.new()
	quick.add_theme_constant_override("separation", 8)
	for q in [["Утро", 7.5], ["День", 13.0], ["Закат", 19.3], ["Ночь", 23.0]]:
		var b := Button.new()
		b.text = q[0]
		b.custom_minimum_size = Vector2(110, 40)
		var hv: float = q[1]
		b.pressed.connect(func(): time_slider.value = hv)
		quick.add_child(b)
	v.add_child(quick)
	return v

func _page_settings() -> Control:
	var v := VBoxContainer.new()
	v.add_theme_constant_override("separation", 16)
	_title(v, "Настройки")
	var rl := _label("", 15, Color(0.75, 0.8, 0.86))
	radius_slider = HSlider.new()
	radius_slider.min_value = 500
	radius_slider.max_value = 2000
	radius_slider.step = 50
	radius_slider.value_changed.connect(func(x):
		rl.text = "%d м (дальше — рельеф упрощённой геометрией)" % int(x)
		radius_changed.emit(x))
	_row(v, "Дальность детальной прорисовки", radius_slider)
	v.add_child(rl)
	auto_check = CheckButton.new()
	auto_check.text = "Автоматическая коробка передач"
	auto_check.toggled.connect(func(on): assist_changed.emit("auto", on))
	v.add_child(auto_check)
	abs_check = CheckButton.new()
	abs_check.text = "ABS"
	abs_check.toggled.connect(func(on): assist_changed.emit("abs", on))
	v.add_child(abs_check)
	tcs_check = CheckButton.new()
	tcs_check.text = "Антипробуксовочная система"
	tcs_check.toggled.connect(func(on): assist_changed.emit("tcs", on))
	v.add_child(tcs_check)
	var fps := CheckButton.new()
	fps.text = "Показывать FPS"
	fps.toggled.connect(func(on): fps_label.visible = on)
	v.add_child(fps)
	v.add_child(_label("Руль и геймпад: оси настраиваются в controls.cfg рядом с игрой; F4 — монитор осей.", 14, Color(0.7, 0.74, 0.8)))
	v.add_child(_label("Кеш карт: " + (fetcher.cache_dir if fetcher else ""), 13, Color(0.6, 0.64, 0.7)))
	return v

func _page_help() -> Control:
	var v := VBoxContainer.new()
	v.add_theme_constant_override("separation", 6)
	_title(v, "Управление")
	var rows := [
		["W / ↑", "газ"], ["S / ↓", "тормоз; на месте — задний ход"], ["A D / ← →", "руль"], ["Пробел", "ручной тормоз"],
		["Q / E", "передача вниз / вверх (ручная)"], ["G", "автомат / ручная коробка"], ["B / T", "ABS / антипробуксовка"],
		["C", "камера"], ["R", "вернуть на дорогу"], ["M", "карта мира"], ["N", "+1 час"], ["F2", "дождь"],
		["F3", "FPS"], ["F4", "монитор руля / геймпада"], ["F12", "скриншот"], ["Esc", "меню"],
	]
	var g := GridContainer.new()
	g.columns = 2
	g.add_theme_constant_override("h_separation", 24)
	g.add_theme_constant_override("v_separation", 8)
	for r in rows:
		var k := _label(r[0], 16, ACCENT)
		k.custom_minimum_size = Vector2(140, 0)
		g.add_child(k)
		g.add_child(_label(r[1], 16))
	v.add_child(g)
	return v

func open_page(id: String) -> void:
	current_page = id
	for k in pages:
		pages[k].visible = k == id
	for k in page_buttons:
		if page_buttons[k].toggle_mode:
			page_buttons[k].set_pressed_no_signal(k == id)

func show_menu(page: String, state: Dictionary) -> void:
	time_slider.set_value_no_signal(state.get("hour", 13.0))
	radius_slider.set_value_no_signal(state.get("radius", 1000.0))
	rain_check.set_pressed_no_signal(state.get("wet", false))
	abs_check.set_pressed_no_signal(state.get("abs", true))
	tcs_check.set_pressed_no_signal(state.get("tcs", true))
	auto_check.set_pressed_no_signal(state.get("auto", true))
	place_title.text = state.get("place", "")
	place_sub.text = state.get("place_sub", "")
	var cl: Vector2 = state.get("car_ll", Vector2.INF)
	map.car_ll = cl
	map.car_heading = state.get("heading", 0.0)
	map.region_ll = state.get("region_ll", [])
	if page == "map" and cl != Vector2.INF and (current_page != "map" or not menu.visible):
		map.look_at_ll(cl.x, cl.y, 13.0)
	menu.visible = true
	hud.visible = false
	open_page(page)
	Input.mouse_mode = Input.MOUSE_MODE_VISIBLE

func hide_menu() -> void:
	menu.visible = false
	hud.visible = true

func menu_open() -> bool:
	return menu.visible

# ------------------------------------------------------------------ loading screen

func _build_loading() -> void:
	loading = Control.new()
	loading.set_anchors_preset(Control.PRESET_FULL_RECT)
	loading.visible = false
	root.add_child(loading)
	var bg := ColorRect.new()
	bg.color = Color(0.04, 0.05, 0.07, 0.94)
	bg.set_anchors_preset(Control.PRESET_FULL_RECT)
	loading.add_child(bg)
	var c := CenterContainer.new()
	c.set_anchors_preset(Control.PRESET_FULL_RECT)
	loading.add_child(c)
	var v := VBoxContainer.new()
	v.custom_minimum_size = Vector2(560, 0)
	v.add_theme_constant_override("separation", 14)
	c.add_child(v)
	v.add_child(_label("BeamNG.Earth", 34, Color.WHITE))
	load_title = _label("", 20, Color(0.85, 0.88, 0.92))
	load_title.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	v.add_child(load_title)
	load_bar = ProgressBar.new()
	load_bar.custom_minimum_size = Vector2(0, 10)
	load_bar.show_percentage = false
	load_bar.max_value = 1.0
	v.add_child(load_bar)
	load_text = _label("", 15, Color(0.7, 0.74, 0.8))
	load_text.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	v.add_child(load_text)
	v.add_child(_label("Данные: © OpenStreetMap contributors, Overture Maps Foundation; рельеф — Mapzen / AWS Terrain Tiles.", 12, Color(0.55, 0.58, 0.63)))

func show_loading(title: String, text: String, frac: float) -> void:
	loading.visible = true
	load_title.text = title
	load_text.text = text
	load_bar.value = frac if frac >= 0.0 else 0.0

func hide_loading() -> void:
	loading.visible = false
