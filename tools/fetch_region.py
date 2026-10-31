#!/usr/bin/env python3
"""Download real map data for a region, for offline development and tests (run by the fetch-data workflow,
which has internet access; the development sandbox has none).

Saves, under out/:
  overture/<release>/<theme>/14/<x>/<y>.mvt   raw (decompressed) Overture vector tiles, themes buildings,
                                               transportation, base
  dem/13/<x>/<y>.png                           terrarium elevation tiles
  region.json                                  what was fetched
Data: (c) OpenStreetMap contributors (ODbL), (c) Overture Maps Foundation (CDLA / ODbL per theme),
elevation: Mapzen/AWS Terrain Tiles (various open sources).
"""
import gzip, json, math, os, sys, time, re
import requests
from pmtiles.reader import Reader

BUCKET = "https://overturemaps-extras-us-west-2.s3.us-west-2.amazonaws.com"

def tile_of(lat, lon, z):
    n = 2 ** z
    x = int((lon + 180) / 360 * n)
    y = int((1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * n)
    return x, y

def bounds(z, x, y):
    n = 2 ** z
    lon0, lon1 = x / n * 360 - 180, (x + 1) / n * 360 - 180
    lat0 = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * (y + 1) / n))))
    lat1 = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / n))))
    return lat0, lon0, lat1, lon1

def newest_release():
    r = requests.get(f"{BUCKET}/?list-type=2&prefix=tiles/&delimiter=/", timeout=60)
    rel = sorted(set(re.findall(r"<Prefix>tiles/(\d[^/]+)/</Prefix>", r.text)))
    return rel[-1]

def main():
    lat, lon, radius = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    out = sys.argv[4]
    phase = sys.argv[5] if len(sys.argv) > 5 else 'all'
    os.makedirs(out, exist_ok=True)
    rel = newest_release()
    print("release", rel)
    dlat = radius / 111320
    dlon = radius / (111320 * math.cos(math.radians(lat)))
    x0, y1 = tile_of(lat - dlat, lon - dlon, 14)
    x1, y0 = tile_of(lat + dlat, lon + dlon, 14)
    tiles = [(x, y) for x in range(x0, x1 + 1) for y in range(y0, y1 + 1)]
    print(len(tiles), "z14 tiles")
    sess = requests.Session()
    if phase in ('all', 'map'):
        fetch_map(rel, tiles, out, sess, lat, lon, dlat, dlon)
    json.dump({"lat": lat, "lon": lon, "radius_m": radius, "release": rel, "tiles14": tiles}, open(f"{out}/region.json", "w"))
    print("all done")

def fetch_map(rel, tiles, out, sess, lat, lon, dlat, dlon):
    for theme in ["buildings", "transportation", "base"]:
        url = f"{BUCKET}/tiles/{rel}/{theme}.pmtiles"
        def get_bytes(offset, length):
            for k in range(5):
                try:
                    r = sess.get(url, headers={"Range": f"bytes={offset}-{offset + length - 1}"}, timeout=120)
                    if r.status_code in (200, 206):
                        return r.content
                except Exception as e:
                    print("retry", e)
                time.sleep(2 * (k + 1))
            raise RuntimeError("range request failed")
        rd = Reader(get_bytes)
        z = min(14, rd.header()["max_zoom"])  # the base theme stops below 14: take parent tiles
        for (x, y) in sorted(set((x >> (14 - z), y >> (14 - z)) for (x, y) in tiles)):
            data = rd.get(z, x, y)
            if data is None:
                continue
            if data[:2] == b"\x1f\x8b":
                data = gzip.decompress(data)
            p = f"{out}/overture/{rel}/{theme}/{z}/{x}"
            os.makedirs(p, exist_ok=True)
            open(f"{p}/{y}.mvt", "wb").write(data)
        print(theme, "done at zoom", z)
    # elevation
    xs0, ys1 = tile_of(lat - dlat, lon - dlon, 13)
    xs1, ys0 = tile_of(lat + dlat, lon + dlon, 13)
    for x in range(xs0 - 1, xs1 + 2):
        for y in range(ys0 - 1, ys1 + 2):
            r = sess.get(f"https://s3.amazonaws.com/elevation-tiles-prod/terrarium/13/{x}/{y}.png", timeout=60)
            if r.status_code == 200:
                p = f"{out}/dem/13/{x}"
                os.makedirs(p, exist_ok=True)
                open(f"{p}/{y}.png", "wb").write(r.content)

main()
