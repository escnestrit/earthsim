## World map: shaded relief of the whole planet (Terrarium elevation tiles, shaded in C++),
## roads from Overture tiles when zoomed in, the car and the loaded region. Drag to pan, wheel to
## zoom, click to pick a destination.
extends Control

signal picked(lat: float, lon: float)

var fetcher: Node
var center := Vector2(0.5, 0.5)      # Web Mercator, 0..1
var zoom := 3.0
var car_ll := Vector2.INF            # lat, lon of the car
var car_heading := 0.0               # radians, 0 = north, clockwise
var region_ll := []                  # [lat_min, lon_min, lat_max, lon_max] of the loaded region
var target_ll := Vector2.INF
var _tiles := {}                     # "z/x/y" -> ImageTexture (or null while loading)
var _roads := {}                     # "x/y" (z14) -> Array of [cls, PackedVector2Array]
var _pending := 0
var _drag := false
var _drag_moved := 0.0
var _font: Font

const ROAD_COL := [Color(0.35, 0.33, 0.4), Color(0.95, 0.95, 0.95), Color(1, 1, 1), Color(1.0, 0.86, 0.45), Color(0.98, 0.62, 0.35)]
const ROAD_W := [1.0, 1.0, 1.6, 2.6, 3.4]

func _ready() -> void:
	clip_contents = true
	mouse_filter = Control.MOUSE_FILTER_STOP
	_font = get_theme_default_font()

static func merc(lat: float, lon: float) -> Vector2:
	var x := (lon + 180.0) / 360.0
	var y := (1.0 - asinh(tan(deg_to_rad(clampf(lat, -85.0, 85.0)))) / PI) / 2.0
	return Vector2(x, y)

static func unmerc(m: Vector2) -> Vector2:
	var lon := m.x * 360.0 - 180.0
	var lat := rad_to_deg(atan(sinh(PI * (1.0 - 2.0 * m.y))))
	return Vector2(lat, lon)

func world_px() -> float:
	return 256.0 * pow(2.0, zoom)

func to_screen(m: Vector2) -> Vector2:
	return (m - center) * world_px() + size * 0.5

func to_merc(p: Vector2) -> Vector2:
	return center + (p - size * 0.5) / world_px()

func look_at_ll(lat: float, lon: float, z: float) -> void:
	center = merc(lat, lon)
	zoom = z
	queue_redraw()

func _gui_input(e: InputEvent) -> void:
	if e is InputEventMouseButton:
		var mb := e as InputEventMouseButton
		if mb.button_index == MOUSE_BUTTON_WHEEL_UP or mb.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			if mb.pressed:
				var before := to_merc(mb.position)
				zoom = clampf(zoom + (0.35 if mb.button_index == MOUSE_BUTTON_WHEEL_UP else -0.35), 1.0, 17.5)
				center += before - to_merc(mb.position)
				queue_redraw()
			accept_event()
		elif mb.button_index == MOUSE_BUTTON_LEFT:
			if mb.pressed:
				_drag = true
				_drag_moved = 0.0
			else:
				_drag = false
				if _drag_moved < 6.0:
					var ll := unmerc(to_merc(mb.position))
					target_ll = ll
					picked.emit(ll.x, ll.y)
					queue_redraw()
			accept_event()
	elif e is InputEventMouseMotion and _drag:
		var mm := e as InputEventMouseMotion
		_drag_moved += mm.relative.length()
		center -= mm.relative / world_px()
		center.x = fposmod(center.x, 1.0)
		center.y = clampf(center.y, 0.0, 1.0)
		queue_redraw()
		accept_event()

func zoom_by(d: float) -> void:
	zoom = clampf(zoom + d, 1.0, 17.5)
	queue_redraw()

func _tile(z: int, x: int, y: int) -> ImageTexture:
	var key := "%d/%d/%d" % [z, x, y]
	if _tiles.has(key):
		return _tiles[key]
	if _pending > 10:
		return null
	if _tiles.size() > 700:
		_tiles.clear()
	_tiles[key] = null
	_pending += 1
	_load_tile(z, x, y, key)
	return null

func _load_tile(z: int, x: int, y: int, key: String) -> void:
	var png: PackedByteArray = await fetcher.dem_tile(z, x, y)
	_pending -= 1
	var rgb: PackedByteArray = EarthWorld.map_shade(png, z, y) if not png.is_empty() else PackedByteArray()
	if rgb.is_empty():
		return  # stays null: the parent tile is drawn instead
	var img := Image.create_from_data(256, 256, false, Image.FORMAT_RGB8, rgb)
	img.generate_mipmaps()
	_tiles[key] = ImageTexture.create_from_image(img)
	queue_redraw()

func _road_tile(x: int, y: int) -> Array:
	var key := "%d/%d" % [x, y]
	if _roads.has(key):
		return _roads[key]
	if _pending > 10:
		return []
	_roads[key] = []
	_pending += 1
	_load_roads(x, y, key)
	return []

func _load_roads(x: int, y: int, key: String) -> void:
	var d: PackedByteArray = await fetcher.roads_tile(x, y)
	_pending -= 1
	_roads[key] = EarthWorld.map_tile_roads(d)
	queue_redraw()

func _draw() -> void:
	draw_rect(Rect2(Vector2.ZERO, size), Color(0.16, 0.27, 0.4))
	var tz := clampi(int(floor(zoom + 0.25)), 0, 13)
	var n := 1 << tz
	var tpx := world_px() / n
	var m0 := to_merc(Vector2.ZERO)
	var m1 := to_merc(size)
	var x0 := int(floor(m0.x * n))
	var x1 := int(floor(m1.x * n))
	var y0 := maxi(0, int(floor(m0.y * n)))
	var y1 := mini(n - 1, int(floor(m1.y * n)))
	for ty in range(y0, y1 + 1):
		for txr in range(x0, x1 + 1):
			var tx := posmod(txr, n)
			var r := Rect2(to_screen(Vector2(float(txr) / n, float(ty) / n)), Vector2(tpx, tpx))
			var tex := _tile(tz, tx, ty)
			if tex:
				draw_texture_rect(tex, r, false)
				continue
			# parent tiles while loading
			for up in range(1, tz + 1):
				var pz := tz - up
				var key := "%d/%d/%d" % [pz, tx >> up, ty >> up]
				var pt: ImageTexture = _tiles.get(key)
				if pt:
					var sub := 256.0 / (1 << up)
					var src := Rect2((tx - ((tx >> up) << up)) * sub, (ty - ((ty >> up) << up)) * sub, sub, sub)
					draw_texture_rect_region(pt, r, src)
					break
	# roads when zoomed in
	if zoom >= 12.5:
		var n14 := 1 << 14
		var a := Vector2i(int(floor(m0.x * n14)), int(floor(m0.y * n14)))
		var b := Vector2i(int(floor(m1.x * n14)), int(floor(m1.y * n14)))
		if (b.x - a.x + 1) * (b.y - a.y + 1) <= 90:
			var k := clampf((zoom - 12.5) / 3.0, 0.4, 1.4)
			for cls_pass in [0, 1, 2, 3, 4]:
				for x in range(a.x, b.x + 1):
					for y in range(a.y, b.y + 1):
						var lines := _road_tile(x, y)
						var org := to_screen(Vector2(float(x) / n14, float(y) / n14))
						var sc := world_px() / n14 / 256.0
						for e in lines:
							if e[0] != cls_pass:
								continue
							var pl: PackedVector2Array = e[1]
							var tr := PackedVector2Array()
							tr.resize(pl.size())
							for i in pl.size():
								tr[i] = org + pl[i] * sc
							draw_polyline(tr, ROAD_COL[e[0]], ROAD_W[e[0]] * k * (1.5 if zoom > 15 else 1.0), true)
	# loaded region
	if region_ll.size() == 4:
		var p0 := to_screen(merc(region_ll[2], region_ll[1]))
		var p1 := to_screen(merc(region_ll[0], region_ll[3]))
		draw_rect(Rect2(p0, p1 - p0), Color(0.4, 0.75, 1.0, 0.9), false, 2.0)
	# destination
	if target_ll != Vector2.INF:
		var t := to_screen(merc(target_ll.x, target_ll.y))
		draw_circle(t + Vector2(0, -18), 9.0, Color(0.95, 0.3, 0.25))
		draw_colored_polygon(PackedVector2Array([t, t + Vector2(-7, -14), t + Vector2(7, -14)]), Color(0.95, 0.3, 0.25))
		draw_circle(t + Vector2(0, -18), 3.5, Color.WHITE)
	# car
	if car_ll != Vector2.INF:
		var c := to_screen(merc(car_ll.x, car_ll.y))
		var f := Vector2(sin(car_heading), -cos(car_heading))
		var s := Vector2(-f.y, f.x)
		draw_circle(c, 13.0, Color(0, 0, 0, 0.25))
		draw_colored_polygon(PackedVector2Array([c + f * 12.0, c - f * 8.0 + s * 8.0, c - f * 4.0, c - f * 8.0 - s * 8.0]), Color(0.3, 0.7, 1.0))
		draw_polyline(PackedVector2Array([c + f * 12.0, c - f * 8.0 + s * 8.0, c - f * 4.0, c - f * 8.0 - s * 8.0, c + f * 12.0]), Color.WHITE, 2.0, true)
	# scale bar
	var lat_c: float = unmerc(center).x
	var mpp := 40075016.0 * cos(deg_to_rad(lat_c)) / world_px()
	var nice := [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000, 1000000, 2000000]
	var lm := 1.0
	for v in nice:
		if v / mpp < 140.0:
			lm = v
	var bl := lm / mpp
	var o := Vector2(20, size.y - 28)
	draw_line(o, o + Vector2(bl, 0), Color.WHITE, 3.0)
	draw_line(o, o + Vector2(0, -6), Color.WHITE, 2.0)
	draw_line(o + Vector2(bl, 0), o + Vector2(bl, -6), Color.WHITE, 2.0)
	var txt := ("%d км" % int(lm / 1000)) if lm >= 1000 else ("%d м" % int(lm))
	draw_string(_font, o + Vector2(bl + 8, 5), txt, HORIZONTAL_ALIGNMENT_LEFT, -1, 14, Color.WHITE)
