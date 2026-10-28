"""SYNTHETIC city in Overpass 'out geom' format (NOT real data). Exercises the same pipeline as real OSM."""
import random, json, sys, math, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from earth_core import LocalFrame

def make(lat0=53.9045, lon0=27.5615, seed=7):
    rnd = random.Random(seed)
    fr = LocalFrame(lat0, lon0)
    els, nid = [], [1000]
    def ll(x, y):
        la, lo, _ = fr.from_local(x, y, 0)
        return {"lat": la, "lon": lo}
    def way(pts, tags, closed=False):
        nid[0] += 1
        p2 = pts + ([pts[0]] if closed else [])
        els.append({"type": "way", "id": nid[0], "tags": tags, "geometry": [ll(*p) for p in p2],
                    "nodes": [hash((round(p[0]), round(p[1]))) & 0xffffff for p in p2]})
    S, N = 130.0, 5
    ext = N * S
    for k in range(-N, N + 1):
        cls = "primary" if k == 0 else ("secondary" if k % 3 == 0 else "residential")
        tags = {"highway": cls, "name": f"Demo {k}"}
        if k == 0:
            tags["lanes"] = "4"
        step = 10.0
        n = int(2 * ext / step) + 1
        way([(k * S, -ext + i * step) for i in range(n)], tags)
        way([(-ext + i * step, k * S) for i in range(n)], dict(tags, name=f"DemoE {k}"))
    way([(300 * math.cos(a / 20 * 2 * math.pi), 300 * math.sin(a / 20 * 2 * math.pi)) for a in range(0, 21)],
        {"highway": "tertiary"})
    for i in range(-N, N):
        for j in range(-N, N):
            x0, y0 = i * S + 11, j * S + 11
            x1, y1 = (i + 1) * S - 11, (j + 1) * S - 11
            for lx in range(2):
                for ly in range(2):
                    bx0 = x0 + lx * (x1 - x0) / 2 + rnd.uniform(1, 4); bx1 = x0 + (lx + 1) * (x1 - x0) / 2 - rnd.uniform(1, 4)
                    by0 = y0 + ly * (y1 - y0) / 2 + rnd.uniform(1, 4); by1 = y0 + (ly + 1) * (y1 - y0) / 2 - rnd.uniform(1, 4)
                    if rnd.random() < 0.08:
                        continue
                    far = abs(i) + abs(j)
                    lv = rnd.choice([2, 3, 5, 5, 9, 9, 12, 16]) if far < 6 else rnd.choice([2, 2, 3, 5, 9])
                    tags = {"building": "yes", "building:levels": str(lv)}
                    if rnd.random() < 0.3:
                        tags["height"] = str(round(lv * 3.0 + rnd.uniform(-1, 1), 1))
                    if rnd.random() < 0.3:
                        mx, my = (bx0 + bx1) / 2, (by0 + by1) / 2
                        pts = [(bx0, by0), (bx1, by0), (bx1, my), (mx, my), (mx, by1), (bx0, by1)]
                    else:
                        pts = [(bx0, by0), (bx1, by0), (bx1, by1), (bx0, by1)]
                    way(pts, tags, closed=True)
    for k in range(-N, N + 1):
        for t in range(-N * 12, N * 12):
            for (x, y) in ((k * S + 6.5, t * 10.8), (t * 10.8, k * S + 6.5)):
                along = y if x == k * S + 6.5 else x      # coordinate along the street the tree stands on
                if abs((along + S / 2) % S - S / 2) < 9:  # keep carriageways of cross streets clear
                    continue
                if rnd.random() < 0.5:
                    g = ll(x, y)
                    nid[0] += 1
                    els.append({"type": "node", "id": nid[0], "lat": g["lat"], "lon": g["lon"], "tags": {"natural": "tree"}})
    return {"elements": els}, (lat0, lon0)

if __name__ == "__main__":
    d, _ = make()
    json.dump(d, open(sys.argv[1], "w"))
