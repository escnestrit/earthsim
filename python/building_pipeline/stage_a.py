"""Stage A: instant representation from footprint + best-known height. Flat roof, no facade claims."""
from earth_core import LocalFrame
from .passport import BuildingPassport, Sourced, Origin, Status

ALGO = "stageA-0.1"
LEVEL_H = 3.0   # assumption, to be calibrated against real data

def estimate_height(tags):
    if "height" in tags:
        try: return Sourced(float(str(tags["height"]).split()[0]), Origin.TAG)
        except ValueError: pass
    lv = tags.get("building:levels")
    if lv:
        try: return Sourced(float(lv) * LEVEL_H, Origin.ESTIMATED)   # levels*3m is an estimate
        except ValueError: pass
    n = 2 if tags.get("building") in ("house", "garage", "shed") else 5
    return Sourced(n * LEVEL_H, Origin.DEFAULT)

def signed_area(ring):
    return 0.5 * sum(x1 * y2 - x2 * y1 for (x1, y1), (x2, y2) in zip(ring, ring[1:] + ring[:1]))

def triangulate(ring):
    """Ear clipping for a CCW simple polygon -> index triples into the ORIGINAL ring, or None if it fails.
    Duplicate and collinear vertices (common in OSM) are skipped for the roof only; walls use the full ring."""
    def cross(o, a, b): return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    def inside(p, a, b, c): return cross(a, b, p) >= 0 and cross(b, c, p) >= 0 and cross(c, a, p) >= 0
    idx = []
    for i, p in enumerate(ring):                       # drop consecutive duplicates
        if idx and abs(p[0]-ring[idx[-1]][0]) + abs(p[1]-ring[idx[-1]][1]) < 1e-6: continue
        idx.append(i)
    while len(idx) > 1 and abs(ring[idx[0]][0]-ring[idx[-1]][0]) + abs(ring[idx[0]][1]-ring[idx[-1]][1]) < 1e-6: idx.pop()
    changed = True
    while changed and len(idx) >= 3:                   # drop collinear vertices and zero-width spikes
        changed = False
        for k in range(len(idx)):
            if abs(cross(ring[idx[k-1]], ring[idx[k]], ring[idx[(k+1) % len(idx)]])) < 1e-6:
                idx.pop(k); changed = True; break
    if len(idx) < 3: return None
    out = []
    while len(idx) > 3:
        for k in range(len(idx)):
            i0, i1, i2 = idx[k-1], idx[k], idx[(k+1) % len(idx)]
            a, b, c = ring[i0], ring[i1], ring[i2]
            if cross(a, b, c) <= 0: continue
            if any(inside(ring[j], a, b, c) for j in idx if j not in (i0, i1, i2)): continue
            out.append((i0, i1, i2)); idx.pop(k); break
        else:
            return None
    out.append(tuple(idx))
    return out

def build(bid, latlon_ring, tags, frame, base_u=0.0):
    ring = [frame.to_local(la, lo)[:2] for la, lo in latlon_ring]
    if ring[0] == ring[-1]: ring = ring[:-1]
    if signed_area(ring) < 0: ring.reverse()
    h = estimate_height(tags)
    tris = triangulate(ring)
    n = len(ring)
    verts = [(x, y, base_u) for x, y in ring] + [(x, y, base_u + h.value) for x, y in ring]
    idxs = []
    for i in range(n):
        j = (i + 1) % n
        idxs += [(i, j, n + j), (i, n + j, n + i)]
    if tris is not None:
        idxs += [(n + a, n + b, n + c) for a, b, c in tris]
    p = BuildingPassport(id=bid, footprint=list(latlon_ring), height_m=h, category=tags.get("building", "yes"),
                         status=Status.PROCEDURAL_APPROXIMATION if tris is not None else Status.LOW_DETAIL_FALLBACK,
                         geometry_confidence="footprint_only", algo_version=ALGO)
    return p, verts, idxs
