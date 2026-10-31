## Downloads map data for any place on Earth while the game runs, into a disk cache:
## Overture Maps vector tiles (PMTiles archives read with HTTP range requests) and Terrarium
## elevation tiles. Writes a region description the core loads like a bundled region.
## Open data only: (c) OpenStreetMap contributors, Overture Maps Foundation, Mapzen/AWS terrain tiles.
extends Node

signal progress(done: int, total: int, text: String)

const OVERTURE := "https://overturemaps-extras-us-west-2.s3.us-west-2.amazonaws.com"
const TERRARIUM := "https://s3.amazonaws.com/elevation-tiles-prod/terrarium"
const THEMES := ["transportation", "buildings", "base"]
const WORKERS := 6

var overture_url := OVERTURE
var terrarium_url := TERRARIUM
var cache_dir := ""            # absolute path of the cache root
var release := ""
var last_error := ""
var _headers := {}             # theme -> header dictionary
var _dirs := {}                # "theme:offset" -> parsed directory
var _pool: Array[HTTPRequest] = []
var _free: Array[HTTPRequest] = []

func _ready() -> void:
	if cache_dir == "":
		cache_dir = ProjectSettings.globalize_path("user://cache")
	DirAccess.make_dir_recursive_absolute(cache_dir)
	for i in WORKERS + 2:
		var h := HTTPRequest.new()
		h.use_threads = true
		h.body_size_limit = -1
		h.timeout = 60.0
		add_child(h)
		_pool.append(h)
		_free.append(h)

func _exit_tree() -> void:
	for h in _pool:
		h.cancel_request()

# ------------------------------------------------------------------ HTTP

func _http(url: String, from := -1, length := 0) -> Dictionary:
	while _free.is_empty():
		await get_tree().process_frame
	var h: HTTPRequest = _free.pop_back()
	var headers := PackedStringArray(["User-Agent: earthsim/1.0 (open-data driving sim)"])
	if from >= 0:
		headers.append("Range: bytes=%d-%d" % [from, from + length - 1])
	var res := {"ok": false, "code": 0, "body": PackedByteArray()}
	for attempt in 3:
		if h.request(url, headers) != OK:
			await get_tree().create_timer(0.5).timeout
			continue
		var r: Array = await h.request_completed
		res["code"] = r[1]
		if r[0] == HTTPRequest.RESULT_SUCCESS and (r[1] == 200 or r[1] == 206):
			res["ok"] = true
			res["body"] = r[3]
			break
		if r[1] == 404 or r[1] == 403:
			break
		await get_tree().create_timer(1.0 + attempt).timeout
	_free.append(h)
	return res

# ------------------------------------------------------------------ PMTiles v3

static func tile_id(z: int, x: int, y: int) -> int:
	var acc := ((1 << (z * 2)) - 1) / 3
	var a := z - 1
	while a >= 0:
		var s := 1 << a
		var rx := s & x
		var ry := s & y
		acc += ((3 * rx) ^ ry) << a
		if ry == 0:
			if rx != 0:
				x = s - 1 - x
				y = s - 1 - y
			var t := x
			x = y
			y = t
		a -= 1
	return acc

static func _u64(b: PackedByteArray, o: int) -> int:
	return b.decode_u64(o)

static func _varints(b: PackedByteArray) -> PackedInt64Array:
	var out := PackedInt64Array()
	var i := 0
	var n := b.size()
	while i < n:
		var v := 0
		var shift := 0
		while true:
			var c := b[i]
			i += 1
			v |= (c & 0x7f) << shift
			if c < 0x80:
				break
			shift += 7
		out.append(v)
	return out

static func _decompress(b: PackedByteArray, mode: int) -> PackedByteArray:
	# 1 none, 2 gzip
	if mode == 2 or (b.size() > 2 and b[0] == 0x1f and b[1] == 0x8b):
		return b.decompress_dynamic(-1, FileAccess.COMPRESSION_GZIP)
	return b

static func _parse_dir(raw: PackedByteArray) -> Dictionary:
	var v := _varints(raw)
	var n := v[0]
	var ids := PackedInt64Array()
	ids.resize(n)
	var runs := PackedInt64Array()
	runs.resize(n)
	var lens := PackedInt64Array()
	lens.resize(n)
	var offs := PackedInt64Array()
	offs.resize(n)
	var k := 1
	var last := 0
	for i in n:
		last += v[k + i]
		ids[i] = last
	k += n
	for i in n:
		runs[i] = v[k + i]
	k += n
	for i in n:
		lens[i] = v[k + i]
	k += n
	for i in n:
		var o := v[k + i]
		if o == 0 and i > 0:
			offs[i] = offs[i - 1] + lens[i - 1]
		else:
			offs[i] = o - 1
	return {"ids": ids, "runs": runs, "lens": lens, "offs": offs}

func _archive(theme: String) -> String:
	return "%s/tiles/%s/%s.pmtiles" % [overture_url, release, theme]

func _header(theme: String) -> Dictionary:
	if _headers.has(theme):
		return _headers[theme]
	var r: Dictionary = await _http(_archive(theme), 0, 16384)
	if not r["ok"]:
		return {}
	var b: PackedByteArray = r["body"]
	if b.size() < 127 or b.slice(0, 7).get_string_from_ascii() != "PMTiles":
		return {}
	var h := {"root_off": _u64(b, 8), "root_len": _u64(b, 16), "leaf_off": _u64(b, 40), "data_off": _u64(b, 56),
		"icomp": b[97], "tcomp": b[98], "minz": b[100], "maxz": b[101]}
	var root := b.slice(h["root_off"], h["root_off"] + h["root_len"])
	if root.size() < h["root_len"]:
		var rr: Dictionary = await _http(_archive(theme), h["root_off"], h["root_len"])
		root = rr["body"]
	_dirs[theme + ":root"] = _parse_dir(_decompress(root, h["icomp"]))
	_headers[theme] = h
	return h

# returns the (compressed) tile bytes or an empty array when the tile does not exist
func _tile(theme: String, z: int, x: int, y: int) -> PackedByteArray:
	var h: Dictionary = await _header(theme)
	if h.is_empty():
		return PackedByteArray()
	var id := tile_id(z, x, y)
	var key := theme + ":root"
	for depth in 4:
		var d: Dictionary = _dirs[key]
		var ids: PackedInt64Array = d["ids"]
		# last entry with ids[i] <= id
		var lo := 0
		var hi := ids.size() - 1
		var idx := -1
		while lo <= hi:
			var mid := (lo + hi) >> 1
			if ids[mid] <= id:
				idx = mid
				lo = mid + 1
			else:
				hi = mid - 1
		if idx < 0:
			return PackedByteArray()
		var run: int = d["runs"][idx]
		var off: int = d["offs"][idx]
		var ln: int = d["lens"][idx]
		if run == 0:
			key = "%s:%d" % [theme, off]
			if not _dirs.has(key):
				var r: Dictionary = await _http(_archive(theme), h["leaf_off"] + off, ln)
				if not r["ok"]:
					return PackedByteArray()
				_dirs[key] = _parse_dir(_decompress(r["body"], h["icomp"]))
			continue
		if id >= ids[idx] + run:
			return PackedByteArray()
		var t: Dictionary = await _http(_archive(theme), h["data_off"] + off, ln)
		return _decompress(t["body"], h["tcomp"]) if t["ok"] else PackedByteArray()
	return PackedByteArray()

# ------------------------------------------------------------------ release

func _find_release() -> bool:
	if release != "":
		return true
	var r: Dictionary = await _http(overture_url + "/?list-type=2&prefix=tiles/&delimiter=/")
	var found := []
	if r["ok"]:
		var re := RegEx.create_from_string("<Prefix>tiles/(\\d[^/]+)/</Prefix>")
		for m in re.search_all((r["body"] as PackedByteArray).get_string_from_utf8()):
			found.append(m.get_string(1))
	var rel_file := cache_dir.path_join("release.txt")
	if found.is_empty():
		# offline or listing refused: the last release that worked
		if FileAccess.file_exists(rel_file):
			release = FileAccess.get_file_as_string(rel_file).strip_edges()
		return release != ""
	found.sort()
	release = found[-1]
	var f := FileAccess.open(rel_file, FileAccess.WRITE)
	if f:
		f.store_string(release)
	return true

# ------------------------------------------------------------------ regions

static func tile_xy(lat: float, lon: float, z: int) -> Vector2i:
	var n := float(1 << z)
	var x := int(floor((lon + 180.0) / 360.0 * n))
	var y := int(floor((1.0 - asinh(tan(deg_to_rad(lat))) / PI) / 2.0 * n))
	return Vector2i(clampi(x, 0, (1 << z) - 1), clampi(y, 0, (1 << z) - 1))

func _save(path: String, data: PackedByteArray) -> void:
	DirAccess.make_dir_recursive_absolute(path.get_base_dir())
	var f := FileAccess.open(path, FileAccess.WRITE)
	if f:
		f.store_buffer(data)

# Make sure all data for a square region around lat/lon is in the cache. Returns the path of the
# region description ("" on failure, see last_error).
func prepare_region(lat: float, lon: float, radius_m: float) -> String:
	last_error = ""
	progress.emit(0, 1, "Поиск свежей версии карты…")
	if not await _find_release():
		last_error = "нет доступа к серверу карт (Overture Maps)"
		return ""
	var dlat := radius_m / 111320.0
	var dlon := radius_m / (111320.0 * cos(deg_to_rad(lat)))
	var a := tile_xy(lat + dlat, lon - dlon, 14)
	var b := tile_xy(lat - dlat, lon + dlon, 14)
	var tiles14 := []
	for x in range(a.x, b.x + 1):
		for y in range(a.y, b.y + 1):
			tiles14.append([x, y])
	var da := tile_xy(lat + dlat, lon - dlon, 13)
	var db := tile_xy(lat - dlat, lon + dlon, 13)
	var dem := []
	for x in range(da.x - 1, db.x + 2):
		for y in range(da.y - 1, db.y + 2):
			dem.append([x, y])
	# jobs: [kind, theme, z, x, y, path]
	var jobs := []
	for theme in THEMES:
		var h: Dictionary = await _header(theme)
		if h.is_empty():
			last_error = "не удалось открыть архив %s (версия %s)" % [theme, release]
			return ""
		var z := mini(14, int(h["maxz"]))
		var seen := {}
		for t in tiles14:
			var k := Vector2i(t[0] >> (14 - z), t[1] >> (14 - z))
			if seen.has(k):
				continue
			seen[k] = true
			var p := cache_dir.path_join("overture/%s/%s/%d/%d/%d.mvt" % [release, theme, z, k.x, k.y])
			var miss := p + ".none"
			if not FileAccess.file_exists(p) and not FileAccess.file_exists(miss):
				jobs.append(["mvt", theme, z, k.x, k.y, p])
	for t in dem:
		var p := cache_dir.path_join("dem/13/%d/%d.png" % [t[0], t[1]])
		if not FileAccess.file_exists(p):
			jobs.append(["dem", "", 13, t[0], t[1], p])
	var total := jobs.size()
	var state := {"done": 0, "failed": 0, "active": 0}
	progress.emit(0, total, "Загрузка карты: 0 / %d" % total)
	var worker := func() -> void:
		while not jobs.is_empty():
			var j: Array = jobs.pop_back()
			var data := PackedByteArray()
			if j[0] == "mvt":
				data = await _tile(j[1], j[2], j[3], j[4])
				if data.is_empty():
					_save(j[5] + ".none", PackedByteArray([0]))  # sea / empty: remember, do not ask again
				else:
					_save(j[5], data)
			else:
				var r: Dictionary = await _http("%s/13/%d/%d.png" % [terrarium_url, j[3], j[4]])
				if r["ok"]:
					_save(j[5], r["body"])
				else:
					state["failed"] += 1
			state["done"] += 1
			progress.emit(state["done"], total, "Загрузка карты: %d / %d" % [state["done"], total])
		state["active"] -= 1
	for i in mini(WORKERS, maxi(1, total)):
		state["active"] += 1
		worker.call()
	while state["active"] > 0:
		await get_tree().process_frame
	# the core expects every listed tile of the transportation theme; sea tiles are written empty
	for t in tiles14:
		var p := cache_dir.path_join("overture/%s/transportation/14/%d/%d.mvt" % [release, t[0], t[1]])
		if not FileAccess.file_exists(p):
			_save(p, PackedByteArray())
	var spec := {"lat": lat, "lon": lon, "radius_m": radius_m, "release": release, "tiles14": tiles14, "dem13": dem}
	var sp := cache_dir.path_join("regions/%.4f_%.4f.json" % [lat, lon])
	DirAccess.make_dir_recursive_absolute(sp.get_base_dir())
	var f := FileAccess.open(sp, FileAccess.WRITE)
	f.store_string(JSON.stringify(spec))
	f.close()
	progress.emit(total, total, "Карта загружена")
	return sp

# Elevation tile for the world map (any zoom), cached.
func dem_tile(z: int, x: int, y: int) -> PackedByteArray:
	var p := cache_dir.path_join("dem/%d/%d/%d.png" % [z, x, y])
	if FileAccess.file_exists(p):
		return FileAccess.get_file_as_bytes(p)
	var r: Dictionary = await _http("%s/%d/%d/%d.png" % [terrarium_url, z, x, y])
	if r["ok"]:
		_save(p, r["body"])
		return r["body"]
	return PackedByteArray()

# Roads tile for the world map at z >= 13 (decoded transportation vector tile), cached on disk.
func roads_tile(x: int, y: int) -> PackedByteArray:
	if not await _find_release():
		return PackedByteArray()
	var h: Dictionary = await _header("transportation")
	if h.is_empty():
		return PackedByteArray()
	var p := cache_dir.path_join("overture/%s/transportation/14/%d/%d.mvt" % [release, x, y])
	if FileAccess.file_exists(p):
		return FileAccess.get_file_as_bytes(p)
	var d: PackedByteArray = await _tile("transportation", 14, x, y)
	if not d.is_empty():
		_save(p, d)
	return d
