## Procedural sedan body: lofted cross-sections for the lower body and the glasshouse,
## wheels with tyres and rims, head/tail lights. Built once, in the car's local frame
## (forward -Z, origin at wheel-centre height).
extends RefCounted

var body_color := Color(0.62, 0.05, 0.06)

# lower body sections: z (front -> rear), half width, bottom y, top y
const BODY := [
	[-2.32, 0.80, -0.12, 0.30],
	[-2.26, 0.86, -0.17, 0.42],
	[-2.05, 0.89, -0.19, 0.50],
	[-1.60, 0.90, -0.19, 0.55],
	[-1.00, 0.90, -0.19, 0.60],
	[-0.80, 0.90, -0.19, 0.62],
	[0.00, 0.91, -0.19, 0.64],
	[1.00, 0.91, -0.19, 0.66],
	[1.70, 0.90, -0.19, 0.70],
	[2.05, 0.88, -0.17, 0.70],
	[2.28, 0.84, -0.12, 0.62],
	[2.36, 0.78, -0.05, 0.45],
]
# glasshouse: z, half width at belt, half width at roof, belt y, roof y
const CABIN := [
	[-0.95, 0.86, 0.70, 0.60, 0.62],
	[-0.55, 0.86, 0.70, 0.62, 1.08],
	[-0.25, 0.85, 0.69, 0.63, 1.12],
	[0.60, 0.85, 0.68, 0.65, 1.12],
	[0.95, 0.85, 0.67, 0.66, 1.06],
	[1.55, 0.86, 0.68, 0.69, 0.71],
]

func _ring_body(sec: Array) -> Array:
	var z: float = sec[0]
	var hw: float = sec[1]
	var y0: float = sec[2]
	var y1: float = sec[3]
	# rounded rectangle, 10 points, clockwise seen from the front (-Z looking +Z)
	var r := 0.12
	return [
		Vector3(-hw + r, y0, z), Vector3(-hw, y0 + r, z), Vector3(-hw, y1 - r * 1.4, z), Vector3(-hw + r * 0.6, y1, z),
		Vector3(hw - r * 0.6, y1, z), Vector3(hw, y1 - r * 1.4, z), Vector3(hw, y0 + r, z), Vector3(hw - r, y0, z),
	]

func _ring_cabin(sec: Array) -> Array:
	var z: float = sec[0]
	var wb: float = sec[1]
	var wr: float = sec[2]
	var yb: float = sec[3]
	var yr: float = sec[4]
	return [Vector3(-wb, yb, z), Vector3(-wr, yr - 0.05, z), Vector3(-wr + 0.08, yr, z), Vector3(wr - 0.08, yr, z), Vector3(wr, yr - 0.05, z), Vector3(wb, yb, z)]

static func _loft(st: SurfaceTool, rings: Array, closed: bool) -> void:
	for i in rings.size() - 1:
		var a: Array = rings[i]
		var b: Array = rings[i + 1]
		var n := a.size()
		var segs := n if closed else n - 1
		for k in segs:
			var k1 := (k + 1) % n
			var p := [a[k], a[k1], b[k1], b[k]]
			var nrm: Vector3 = (p[1] - p[0]).cross(p[3] - p[0]).normalized()
			if nrm.length() < 0.5:
				nrm = (p[2] - p[1]).cross(p[3] - p[1]).normalized()
			for idx in [0, 1, 2, 0, 2, 3]:
				st.set_normal(nrm)
				st.add_vertex(p[idx])

static func _cap(st: SurfaceTool, ring: Array, flip: bool) -> void:
	var c := Vector3.ZERO
	for p in ring:
		c += p
	c /= ring.size()
	for k in ring.size():
		var a: Vector3 = ring[k]
		var b: Vector3 = ring[(k + 1) % ring.size()]
		var n := (a - c).cross(b - c).normalized() * (-1.0 if flip else 1.0)
		var tri := [c, b, a] if flip else [c, a, b]
		for q in tri:
			st.set_normal(n)
			st.add_vertex(q)

func build_body() -> ArrayMesh:
	var paint := StandardMaterial3D.new()
	paint.albedo_color = body_color
	paint.metallic = 0.55
	paint.roughness = 0.28
	paint.clearcoat_enabled = true
	paint.clearcoat = 0.8
	var glass := StandardMaterial3D.new()
	glass.albedo_color = Color(0.05, 0.07, 0.09)
	glass.metallic = 0.2
	glass.roughness = 0.05
	var trim := StandardMaterial3D.new()
	trim.albedo_color = Color(0.04, 0.04, 0.045)
	trim.roughness = 0.6
	var mesh := ArrayMesh.new()
	# body
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	var rings := []
	for s in BODY:
		rings.append(_ring_body(s))
	_loft(st, rings, true)
	_cap(st, rings[0], true)
	_cap(st, rings[rings.size() - 1], false)
	st.commit(mesh)
	mesh.surface_set_material(0, paint)
	# glasshouse
	var gl := SurfaceTool.new()
	gl.begin(Mesh.PRIMITIVE_TRIANGLES)
	var cr := []
	for s in CABIN:
		cr.append(_ring_cabin(s))
	_loft(gl, cr, false)
	gl.commit(mesh)
	mesh.surface_set_material(1, glass)
	# roof panel (paint) over the glass, pillars implied by the glass colour
	var rf := SurfaceTool.new()
	rf.begin(Mesh.PRIMITIVE_TRIANGLES)
	var roof := []
	for i in range(1, 5):
		var s: Array = CABIN[i]
		var hw: float = s[2] - 0.06
		var y: float = s[4] + 0.012
		roof.append([Vector3(-hw, y, s[0] + (0.08 if i == 1 else 0.0)), Vector3(hw, y, s[0] + (0.08 if i == 1 else 0.0))])
	for i in roof.size() - 1:
		var p := [roof[i][0], roof[i][1], roof[i + 1][1], roof[i + 1][0]]
		for idx in [0, 2, 1, 0, 3, 2]:
			rf.set_normal(Vector3.UP)
			rf.add_vertex(p[idx])
	rf.commit(mesh)
	mesh.surface_set_material(2, paint)
	# lights and grille
	var li := SurfaceTool.new()
	li.begin(Mesh.PRIMITIVE_TRIANGLES)
	for sx in [-1.0, 1.0]:
		_quad(li, Vector3(sx * 0.62, 0.36, -2.275), 0.34, 0.11, Vector3(0, 0, -1))
	li.commit(mesh)
	var head := StandardMaterial3D.new()
	head.albedo_color = Color(0.9, 0.92, 0.95)
	head.emission_enabled = true
	head.emission = Color(1.0, 0.95, 0.85)
	head.emission_energy_multiplier = 0.4
	mesh.surface_set_material(3, head)
	var tl := SurfaceTool.new()
	tl.begin(Mesh.PRIMITIVE_TRIANGLES)
	for sx in [-1.0, 1.0]:
		_quad(tl, Vector3(sx * 0.6, 0.55, 2.335), 0.36, 0.1, Vector3(0, 0, 1))
	tl.commit(mesh)
	var tail := StandardMaterial3D.new()
	tail.albedo_color = Color(0.6, 0.02, 0.02)
	tail.emission_enabled = true
	tail.emission = Color(1.0, 0.05, 0.03)
	tail.emission_energy_multiplier = 0.6
	mesh.surface_set_material(4, tail)
	var gr := SurfaceTool.new()
	gr.begin(Mesh.PRIMITIVE_TRIANGLES)
	_quad(gr, Vector3(0, 0.2, -2.29), 0.75, 0.1, Vector3(0, 0, -1))
	_quad(gr, Vector3(0, -0.05, 2.37), 0.8, 0.1, Vector3(0, 0, 1))
	gr.commit(mesh)
	mesh.surface_set_material(5, trim)
	return mesh

static func _quad(st: SurfaceTool, c: Vector3, hw: float, hh: float, n: Vector3) -> void:
	var r := Vector3(1, 0, 0) if n.z < 0 else Vector3(-1, 0, 0)
	var p := [c - r * hw - Vector3(0, hh, 0), c + r * hw - Vector3(0, hh, 0), c + r * hw + Vector3(0, hh, 0), c - r * hw + Vector3(0, hh, 0)]
	for idx in [0, 2, 1, 0, 3, 2]:
		st.set_normal(n)
		st.add_vertex(p[idx])

func build_wheel(radius: float, width: float) -> ArrayMesh:
	var mesh := ArrayMesh.new()
	var tyre := SurfaceTool.new()
	tyre.begin(Mesh.PRIMITIVE_TRIANGLES)
	var seg := 24
	var hw := width * 0.5
	for i in seg:
		var a0 := TAU * i / seg
		var a1 := TAU * (i + 1) / seg
		var d0 := Vector3(0, cos(a0), sin(a0))
		var d1 := Vector3(0, cos(a1), sin(a1))
		# tread
		var p := [d0 * radius + Vector3(-hw, 0, 0), d1 * radius + Vector3(-hw, 0, 0), d1 * radius + Vector3(hw, 0, 0), d0 * radius + Vector3(hw, 0, 0)]
		var nn := [d0, d1, d1, d0]
		for idx in [0, 1, 2, 0, 2, 3]:
			tyre.set_normal(nn[idx])
			tyre.add_vertex(p[idx])
		# sidewalls (outer +X, inner -X) between rim radius and tyre radius
		for side in [-1.0, 1.0]:
			var ri := radius * 0.64
			var q := [d0 * ri + Vector3(side * hw * 0.9, 0, 0), d1 * ri + Vector3(side * hw * 0.9, 0, 0), d1 * radius + Vector3(side * hw, 0, 0), d0 * radius + Vector3(side * hw, 0, 0)]
			var order := [0, 2, 1, 0, 3, 2] if side > 0 else [0, 1, 2, 0, 2, 3]
			for idx in order:
				tyre.set_normal(Vector3(side, 0, 0))
				tyre.add_vertex(q[idx])
	tyre.commit(mesh)
	var rubber := StandardMaterial3D.new()
	rubber.albedo_color = Color(0.05, 0.05, 0.05)
	rubber.roughness = 0.9
	mesh.surface_set_material(0, rubber)
	var rim := SurfaceTool.new()
	rim.begin(Mesh.PRIMITIVE_TRIANGLES)
	var spokes := 5
	for side in [-1.0, 1.0]:
		var x: float = side * hw * 0.85
		for i in seg:
			var a0 := TAU * i / seg
			var a1 := TAU * (i + 1) / seg
			var ri := radius * 0.64
			var spoke: bool = absf(sin(a0 * spokes * 0.5)) > 0.55
			var rin := ri * (0.18 if spoke else 0.85)
			var p := [Vector3(x, cos(a0) * rin, sin(a0) * rin), Vector3(x, cos(a1) * rin, sin(a1) * rin), Vector3(x, cos(a1) * ri, sin(a1) * ri), Vector3(x, cos(a0) * ri, sin(a0) * ri)]
			var order := [0, 2, 1, 0, 3, 2] if side > 0 else [0, 1, 2, 0, 2, 3]
			for idx in order:
				rim.set_normal(Vector3(side, 0, 0))
				rim.add_vertex(p[idx])
	rim.commit(mesh)
	var alloy := StandardMaterial3D.new()
	alloy.albedo_color = Color(0.72, 0.73, 0.75)
	alloy.metallic = 0.9
	alloy.roughness = 0.3
	mesh.surface_set_material(1, alloy)
	return mesh
