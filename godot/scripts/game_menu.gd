# In-game menu (Esc): teleport anywhere (presets, coordinates, place search), pick the car, settings.
# Settings are kept in user://settings.cfg; a teleport restarts the scene at the new origin.
extends CanvasLayer
class_name GameMenu

const PLACES := [
	["Минск — проспект Независимости", 53.9024, 27.5618],
	["Минск — Немига", 53.9045, 27.5537],
	["Минск — Площадь Победы", 53.9087, 27.5750],
	["Москва — Тверская", 55.7625, 37.6080],
	["Москва — Садовое кольцо", 55.7570, 37.6340],
	["Санкт-Петербург — Невский проспект", 59.9343, 30.3351],
	["Киев — Крещатик", 50.4470, 30.5220],
	["Варшава — центр", 52.2297, 21.0122],
	["Берлин — Александерплац", 52.5219, 13.4132],
	["Париж — Елисейские поля", 48.8698, 2.3075],
	["Лондон — Пикадилли", 51.5100, -0.1340],
	["Нью-Йорк — Таймс-сквер", 40.7580, -73.9855],
	["Лос-Анджелес — Голливуд", 34.1016, -118.3267],
	["Токио — Синдзюку", 35.6938, 139.7034],
]

var main: Node
var root: PanelContainer
var tabs: TabContainer
var car_list: ItemList
var cfg_list: ItemList
var cfg_names := PackedStringArray()
var place_list: ItemList
var coord_edit: LineEdit
var search_edit: LineEdit
var search_list: ItemList
var search_results := []
var info: Label
var http: HTTPRequest
var cfg := ConfigFile.new()
var controls := {}

static func load_settings() -> ConfigFile:
	var c := ConfigFile.new()
	c.load("user://settings.cfg")
	return c

func _init(m: Node) -> void:
	main = m
	layer = 10
	process_mode = Node.PROCESS_MODE_ALWAYS
	cfg = load_settings()

func _ready() -> void:
	http = HTTPRequest.new()
	http.process_mode = Node.PROCESS_MODE_ALWAYS
	add_child(http)
	http.request_completed.connect(_on_search_done)
	root = PanelContainer.new()
	# centred, up to 760 x 600, shrinking with small windows
	root.anchor_left = 0.5
	root.anchor_top = 0.5
	root.anchor_right = 0.5
	root.anchor_bottom = 0.5
	_fit()
	get_viewport().size_changed.connect(_fit)
	var sb := StyleBoxFlat.new()
	sb.bg_color = Color(0.08, 0.09, 0.11, 0.93)
	sb.corner_radius_top_left = 10
	sb.corner_radius_top_right = 10
	sb.corner_radius_bottom_left = 10
	sb.corner_radius_bottom_right = 10
	sb.content_margin_left = 14
	sb.content_margin_right = 14
	sb.content_margin_top = 12
	sb.content_margin_bottom = 12
	root.add_theme_stylebox_override("panel", sb)
	add_child(root)
	var vb := VBoxContainer.new()
	root.add_child(vb)
	var title := Label.new()
	title.text = "BeamNG.Earth"
	title.add_theme_font_size_override("font_size", 24)
	vb.add_child(title)
	tabs = TabContainer.new()
	tabs.size_flags_vertical = Control.SIZE_EXPAND_FILL
	vb.add_child(tabs)
	_build_teleport()
	_build_cars()
	_build_settings()
	_build_controls()
	var bottom := HBoxContainer.new()
	vb.add_child(bottom)
	info = Label.new()
	info.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	info.autowrap_mode = TextServer.AUTOWRAP_WORD
	bottom.add_child(info)
	var close := Button.new()
	close.text = "Продолжить (Esc)"
	close.pressed.connect(toggle)
	bottom.add_child(close)
	var quit := Button.new()
	quit.text = "Выход"
	quit.pressed.connect(func(): main.get_tree().quit())
	bottom.add_child(quit)
	visible = false

func _fit() -> void:
	var vs := get_viewport().get_visible_rect().size
	var w := minf(760.0, vs.x - 24.0)
	var h := minf(600.0, vs.y - 24.0)
	root.offset_left = -w / 2
	root.offset_right = w / 2
	root.offset_top = -h / 2
	root.offset_bottom = h / 2
	var small := vs.y < 500
	root.theme = Theme.new()
	root.theme.default_font_size = 12 if small else 16

func _input(e: InputEvent) -> void:
	if e is InputEventKey and e.pressed and not e.echo and e.keycode == KEY_ESCAPE:
		toggle()
		get_viewport().set_input_as_handled()

func toggle() -> void:
	visible = not visible
	main.get_tree().paused = visible and not main.args.has("shot")
	if visible:
		_refresh_cars()
		var ll: Vector2 = main.world.stream.latlon(main.car.global_position)
		if main.args.has("menu"):
			tabs.current_tab = int(main.args["menu"])
		info.text = "Сейчас: %.5f, %.5f" % [ll.x, ll.y]

# ---------------- teleport
func _build_teleport() -> void:
	var vb := VBoxContainer.new()
	vb.name = "Телепорт"
	tabs.add_child(vb)
	var l := Label.new()
	l.text = "Места:"
	vb.add_child(l)
	place_list = ItemList.new()
	place_list.custom_minimum_size = Vector2(0, 150)
	for p in PLACES:
		place_list.add_item(p[0])
	place_list.item_activated.connect(func(i): _teleport(PLACES[i][1], PLACES[i][2]))
	vb.add_child(place_list)
	var h := HBoxContainer.new()
	vb.add_child(h)
	coord_edit = LineEdit.new()
	coord_edit.placeholder_text = "Координаты: 53.9024, 27.5618"
	coord_edit.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	coord_edit.text_submitted.connect(func(_t): _go_coords())
	h.add_child(coord_edit)
	var go := Button.new()
	go.text = "Перенестись"
	go.pressed.connect(_go_coords)
	h.add_child(go)
	var h2 := HBoxContainer.new()
	vb.add_child(h2)
	search_edit = LineEdit.new()
	search_edit.placeholder_text = "Поиск места: город, улица, адрес"
	search_edit.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	search_edit.text_submitted.connect(func(_t): _search())
	h2.add_child(search_edit)
	var sbt := Button.new()
	sbt.text = "Найти"
	sbt.pressed.connect(_search)
	h2.add_child(sbt)
	search_list = ItemList.new()
	search_list.custom_minimum_size = Vector2(0, 90)
	search_list.item_activated.connect(func(i): _teleport(search_results[i][1], search_results[i][2]))
	vb.add_child(search_list)
	var hint := Label.new()
	hint.text = "Двойной щелчок по месту — перенос. Поиск: OpenStreetMap Nominatim."
	hint.add_theme_font_size_override("font_size", 12)
	vb.add_child(hint)

func _go_coords() -> void:
	var t := coord_edit.text.replace(";", ",").replace(" ", ",")
	var nums := []
	for s in t.split(",", false):
		if s.is_valid_float():
			nums.append(float(s))
	if nums.size() >= 2 and absf(nums[0]) <= 85 and absf(nums[1]) <= 180:
		_teleport(nums[0], nums[1])
	else:
		info.text = "Не понял координаты. Пример: 53.9024, 27.5618"

func _search() -> void:
	var q := search_edit.text.strip_edges()
	if q == "":
		return
	var url := "https://nominatim.openstreetmap.org/search?format=json&limit=8&q=" + q.uri_encode()
	var hdr := PackedStringArray(["User-Agent: BeamNG.Earth/0.4 (earthsim game; place search)", "Accept-Language: ru,en"])
	http.cancel_request()
	if http.request(url, hdr) != OK:
		info.text = "Поиск недоступен"
	else:
		info.text = "Ищу…"

func _on_search_done(result: int, code: int, _h: PackedStringArray, body: PackedByteArray) -> void:
	search_list.clear()
	search_results.clear()
	if result != HTTPRequest.RESULT_SUCCESS or code != 200:
		info.text = "Поиск не удался (нет сети?)"
		return
	var arr = JSON.parse_string(body.get_string_from_utf8())
	if not (arr is Array) or (arr as Array).is_empty():
		info.text = "Ничего не найдено"
		return
	for r in arr:
		var name: String = r.get("display_name", "?")
		search_results.append([name, float(r.get("lat", "0")), float(r.get("lon", "0"))])
		search_list.add_item(name.left(90))
	info.text = "Найдено: %d" % search_results.size()

func _teleport(lat: float, lon: float) -> void:
	Engine.set_meta("earthsim_teleport", Vector2(lat, lon))
	Engine.set_meta("earthsim_vehicle", main.current_vehicle_path())
	main.get_tree().paused = false
	main.get_tree().reload_current_scene()

# ---------------- cars
func _build_cars() -> void:
	var vb := VBoxContainer.new()
	vb.name = "Машина"
	tabs.add_child(vb)
	var hs := HSplitContainer.new()
	hs.size_flags_vertical = Control.SIZE_EXPAND_FILL
	vb.add_child(hs)
	car_list = ItemList.new()
	car_list.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	car_list.item_activated.connect(_pick_car)
	car_list.item_selected.connect(_show_configs)
	hs.add_child(car_list)
	# the selected vehicle's configurations (BeamNG .pc files), the default first
	cfg_list = ItemList.new()
	cfg_list.custom_minimum_size = Vector2(220, 0)
	cfg_list.item_activated.connect(func(_i):
		var s := car_list.get_selected_items()
		if s.size() > 0:
			_pick_car(s[0]))
	hs.add_child(cfg_list)
	var h := HBoxContainer.new()
	vb.add_child(h)
	var b := Button.new()
	b.text = "Сесть в машину"
	b.pressed.connect(func():
		var s := car_list.get_selected_items()
		if s.size() > 0:
			_pick_car(s[0]))
	h.add_child(b)
	var r := Button.new()
	r.text = "Поставить на колёса (R)"
	r.pressed.connect(func():
		main.car.reset()
		toggle())
	h.add_child(r)
	var hint := Label.new()
	hint.text = "Машины BeamNG.drive находятся сами: установка в Steam, ваши моды BeamNG (zip, repo, unpacked) и папка mods рядом с игрой. Справа — комплектации (.pc)."
	hint.autowrap_mode = TextServer.AUTOWRAP_WORD
	hint.add_theme_font_size_override("font_size", 12)
	vb.add_child(hint)

func _refresh_cars() -> void:
	car_list.clear()
	car_list.add_item("Встроенная машина (своя физика)")
	for v in main.vehicles:
		var t: String = v.get("type", "")
		car_list.add_item(String(v["name"]) + ("  [мод]" if v.get("mod", false) else "") + ("" if t == "" else "  · " + t))
	car_list.select(main.vehicle_i + 1)

func _show_configs(i: int) -> void:
	cfg_list.clear()
	cfg_names = PackedStringArray()
	if i <= 0 or i - 1 >= main.vehicles.size():
		return
	cfg_names = EarthJCar.configs(main.vehicles[i - 1]["path"])
	for k in cfg_names.size():
		cfg_list.add_item(cfg_names[k] + ("  (по умолчанию)" if k == 0 else ""))
	if cfg_names.size() > 0:
		cfg_list.select(0)

func _pick_car(i: int) -> void:
	var cfg := ""
	var cs := cfg_list.get_selected_items()
	if i > 0 and cs.size() > 0 and cs[0] < cfg_names.size():
		cfg = cfg_names[cs[0]]
	main.select_vehicle(i - 1, cfg)
	toggle()

# ---------------- settings
func _add_slider(parent: Control, key: String, label: String, lo: float, hi: float, step: float, def: float, fmt: String) -> void:
	var h := HBoxContainer.new()
	parent.add_child(h)
	var l := Label.new()
	l.text = label
	l.custom_minimum_size = Vector2(230, 0)
	h.add_child(l)
	var s := HSlider.new()
	s.min_value = lo
	s.max_value = hi
	s.step = step
	s.value = cfg.get_value("game", key, def)
	s.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	h.add_child(s)
	var v := Label.new()
	v.custom_minimum_size = Vector2(70, 0)
	v.text = fmt % s.value
	h.add_child(v)
	s.value_changed.connect(func(x):
		v.text = fmt % x
		cfg.set_value("game", key, x)
		cfg.save("user://settings.cfg")
		main.apply_setting(key, x))
	controls[key] = s

func _add_check(parent: Control, key: String, label: String, def: bool) -> void:
	var c := CheckBox.new()
	c.text = label
	c.button_pressed = cfg.get_value("game", key, def)
	c.toggled.connect(func(x):
		cfg.set_value("game", key, x)
		cfg.save("user://settings.cfg")
		main.apply_setting(key, x))
	parent.add_child(c)
	controls[key] = c

func _build_settings() -> void:
	var sc := ScrollContainer.new()
	sc.name = "Настройки"
	tabs.add_child(sc)
	var vb := VBoxContainer.new()
	vb.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	sc.add_child(vb)
	_add_slider(vb, "hour", "Время суток", 0, 23.75, 0.25, main.hour, "%.2f ч")
	_add_slider(vb, "traffic", "Машин в потоке", 0, 120, 1, 40, "%d")
	_add_slider(vb, "people", "Пешеходов", 0, 200, 1, 60, "%d")
	_add_slider(vb, "fov", "Угол обзора", 50, 100, 1, 70, "%d°")
	_add_slider(vb, "volume", "Громкость", 0, 1, 0.05, 0.8, "%.2f")
	_add_slider(vb, "detail", "Детальная зона (после переноса)", 300, 1200, 50, 700, "%d м")
	_add_slider(vb, "far", "Дальность рельефа (после переноса)", 1000, 5000, 100, 2400, "%d м")
	_add_check(vb, "gi", "Глобальное освещение (SDFGI)", true)
	_add_check(vb, "ssr", "Отражения (SSR)", true)
	_add_check(vb, "shadows", "Тени", true)
	_add_check(vb, "fog", "Дымка вдали", true)
	_add_check(vb, "hud", "Показывать HUD", true)

func _build_controls() -> void:
	var l := Label.new()
	l.name = "Управление"
	l.autowrap_mode = TextServer.AUTOWRAP_WORD
	l.text = """W/S или ↑/↓ — газ / тормоз и задний ход
A/D или ←/→ — руль
Space — ручник
R — поставить на колёса
C — камера: сзади / из салона / свободная (I J K L, Shift — быстрее)
Правая кнопка мыши — повернуть камеру, колесо — дальше/ближе
V — следующая машина
L — фары, T — время суток +1.5 ч, G — GI вкл/выкл
Esc — это меню"""
	tabs.add_child(l)
