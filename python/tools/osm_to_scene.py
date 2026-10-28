"""Overpass JSON ('out geom') -> scene.json for the viewer.
Pipeline: lat/lon -> local ENU (earth_core) -> Stage A buildings (building_pipeline).
Not handled yet: multipolygon relations, building:part, DEM (everything is flat), bridges/layers."""
import json, math, sys, pathlib, collections
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from earth_core import LocalFrame
from building_pipeline.stage_a import build

DRIVABLE = {  # class -> default width in m (assumption; overridden by 'lanes' when present)
    "motorway": 14, "trunk": 12, "primary": 11, "secondary": 9, "tertiary": 8,
    "unclassified": 6.5, "residential": 6.5, "living_street": 5, "service": 4,
    "motorway_link": 7, "trunk_link": 7, "primary_link": 7, "secondary_link": 6.5, "tertiary_link": 6,
}

def road_width(tags):
    try:
        return int(tags.get("lanes", "")) * 3.3, "tag"
    except ValueError:
        return float(DRIVABLE[tags["highway"]]), "default"

def convert(overpass, lat0, lon0, radius=None):
    fr = LocalFrame(lat0, lon0)
    roads, blds, trees = [], [], []
    node_use = collections.Counter()
    status = collections.Counter(); hsrc = collections.Counter()
    for el in overpass.get("elements", []):
        t = el.get("tags", {})
        if el["type"] == "node" and t.get("natural") == "tree" and "lat" in el:
            x, y, _ = fr.to_local(el["lat"], el["lon"])
            if radius is None or math.hypot(x, y) <= radius * 1.2:
                trees.append([round(x, 2), round(y, 2)])
            continue
        geom = el.get("geometry")
        if el["type"] != "way" or not geom:
            continue
        if t.get("highway") in DRIVABLE:
            pts = [fr.to_local(g["lat"], g["lon"])[:2] for g in geom]
            if len(pts) < 2:
                continue
            w, wo = road_width(t)
            ids = el.get("nodes") or [None] * len(pts)
            for i in ids:
                if i is not None:
                    node_use[i] += 1
            roads.append({"cls": t["highway"], "w": w, "w_src": wo, "name": t.get("name", ""),
                          "pts": [[round(x, 2), round(y, 2)] for x, y in pts], "nodes": ids})
        elif "building" in t:
            ring = [(g["lat"], g["lon"]) for g in geom]
            if len(ring) < 4 or ring[0] != ring[-1]:
                continue
            cx, cy, _ = fr.to_local(*ring[0])
            if radius is not None and math.hypot(cx, cy) > radius * 1.2:
                continue
            p, v, idx = build(f"osm:way/{el['id']}", ring, t, fr)
            fp = [fr.to_local(*r)[:2] for r in ring[:-1]]
            blds.append({"id": p.id, "status": p.status.value, "h": p.height_m.value, "h_src": p.height_m.origin.value,
                         "fp": [[round(x, 2), round(y, 2)] for x, y in fp],
                         "v": [round(c, 2) for vv in v for c in vv], "i": [k for tri in idx for k in tri]})
            status[p.status.value] += 1; hsrc[p.height_m.origin.value] += 1
    junction_ids = {n for n, c in node_use.items() if c >= 2}
    for r in roads:
        r["junction"] = [bool(n in junction_ids) if n is not None else False for n in r["nodes"]]
        del r["nodes"]
    return {"meta": {"lat0": lat0, "lon0": lon0, "radius": radius, "n_roads": len(roads), "n_buildings": len(blds),
                     "n_trees": len(trees), "status_counts": dict(status), "height_origin_counts": dict(hsrc),
                     "terrain": "FLAT (no DEM yet)"},
            "roads": roads, "buildings": blds, "trees": trees}

if __name__ == "__main__":
    src, out, lat0, lon0 = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
    radius = float(sys.argv[5]) if len(sys.argv) > 5 else None
    sc = convert(json.load(open(src, encoding="utf-8")), lat0, lon0, radius)
    json.dump(sc, open(out, "w"), separators=(",", ":"))
    print(out, sc["meta"])
