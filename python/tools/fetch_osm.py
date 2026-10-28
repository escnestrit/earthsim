"""Download OSM data around a point via Overpass API (ODbL: (c) OpenStreetMap contributors).
NOT tested in the dev sandbox (no direct network). Usage:
  python tools/fetch_osm.py --lat 53.9045 --lon 27.5615 --radius 700 --out scene.json"""
import argparse, json, math, sys, pathlib, urllib.request, urllib.parse
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from osm_to_scene import convert, DRIVABLE

MIRRORS = ["https://overpass-api.de/api/interpreter", "https://overpass.kumi.systems/api/interpreter"]

def query(lat, lon, r):
    dlat = r / 111320.0
    dlon = r / (111320.0 * math.cos(math.radians(lat)))
    bb = f"{lat-dlat:.6f},{lon-dlon:.6f},{lat+dlat:.6f},{lon+dlon:.6f}"
    hw = "|".join(DRIVABLE)
    return (f'[out:json][timeout:90];(way["highway"~"^({hw})$"]({bb});way["building"]({bb});'
            f'node["natural"="tree"]({bb}););out geom;')

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lat", type=float, required=True)
    ap.add_argument("--lon", type=float, required=True)
    ap.add_argument("--radius", type=float, default=700)
    ap.add_argument("--out", default="scene.json")
    ap.add_argument("--raw", help="also save raw Overpass JSON here")
    a = ap.parse_args()
    data = urllib.parse.urlencode({"data": query(a.lat, a.lon, a.radius)}).encode()
    last = None
    for url in MIRRORS:
        try:
            req = urllib.request.Request(url, data=data, headers={"User-Agent": "BeamNG.Earth-prototype/0.1"})
            raw = json.load(urllib.request.urlopen(req, timeout=120))
            break
        except Exception as e:
            last = e
            print("mirror failed:", url, e)
    else:
        sys.exit(f"all Overpass mirrors failed: {last}")
    if a.raw:
        json.dump(raw, open(a.raw, "w"))
    sc = convert(raw, a.lat, a.lon, a.radius)
    sc["meta"]["source"] = "OpenStreetMap via Overpass (ODbL)"
    json.dump(sc, open(a.out, "w"), separators=(",", ":"))
    print(a.out, sc["meta"])

if __name__ == "__main__":
    main()
