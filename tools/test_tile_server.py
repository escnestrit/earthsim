#!/usr/bin/env python3
"""Dev tool: serve a local copy of the map data the way the real servers do, to test the game's
runtime downloader offline. Packs data/overture/<rel>/<theme>/<z>/<x>/<y>.mvt into gzip-tiled
PMTiles archives and serves them with HTTP Range support, plus an S3-style release listing and
the Terrarium elevation tiles.

  python3 test_tile_server.py DATA_DIR PORT    (needs the protomaps 'pmtiles' python package)
URLs: /?list-type=2...  -> listing,  /tiles/<rel>/<theme>.pmtiles,  /terrarium/13/x/y.png
"""
import gzip, http.server, io, os, re, sys
from pmtiles.tile import zxy_to_tileid, TileType, Compression
from pmtiles.writer import Writer

data, port = sys.argv[1], int(sys.argv[2])
rel = sorted(os.listdir(os.path.join(data, "overture")))[-1]
archives = {}
for theme in sorted(os.listdir(os.path.join(data, "overture", rel))):
    tiles = []
    root = os.path.join(data, "overture", rel, theme)
    for z in os.listdir(root):
        for x in os.listdir(os.path.join(root, z)):
            for f in os.listdir(os.path.join(root, z, x)):
                y = int(f.split(".")[0])
                tiles.append((zxy_to_tileid(int(z), int(x), y), int(z), open(os.path.join(root, z, x, f), "rb").read()))
    tiles.sort()
    buf = io.BytesIO()
    w = Writer(buf)
    for tid, z, d in tiles:
        w.write_tile(tid, gzip.compress(d))
    zs = [t[1] for t in tiles]
    w.finalize({"tile_type": TileType.MVT, "tile_compression": Compression.GZIP, "min_zoom": min(zs), "max_zoom": max(zs),
                "min_lon_e7": 0, "min_lat_e7": 0, "max_lon_e7": 0, "max_lat_e7": 0, "center_zoom": max(zs), "center_lon_e7": 0, "center_lat_e7": 0}, {})
    archives[f"/tiles/{rel}/{theme}.pmtiles"] = buf.getvalue()
    print(theme, len(tiles), "tiles", len(archives[f"/tiles/{rel}/{theme}.pmtiles"]), "bytes")

class H(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass
    def do_GET(self):
        p = self.path.split("?")[0]
        if p == "/" and "list-type" in self.path:
            body = f"<ListBucketResult><CommonPrefixes><Prefix>tiles/2025-01-01.0/</Prefix></CommonPrefixes><CommonPrefixes><Prefix>tiles/{rel}/</Prefix></CommonPrefixes></ListBucketResult>".encode()
            return self.send(200, body)
        if p in archives:
            d = archives[p]
            m = re.match(r"bytes=(\d+)-(\d+)", self.headers.get("Range", ""))
            if m:
                a, b = int(m.group(1)), min(int(m.group(2)), len(d) - 1)
                return self.send(206, d[a:b + 1], {"Content-Range": f"bytes {a}-{b}/{len(d)}"})
            return self.send(200, d)
        m = re.match(r"/terrarium/(\d+)/(\d+)/(\d+)\.png", p)
        if m:
            f = os.path.join(data, "dem", m.group(1), m.group(2), m.group(3) + ".png")
            if os.path.exists(f):
                return self.send(200, open(f, "rb").read())
        self.send(404, b"not found")
    def send(self, code, body, hdr={}):
        self.send_response(code)
        self.send_header("Content-Length", str(len(body)))
        for k, v in hdr.items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

http.server.ThreadingHTTPServer(("127.0.0.1", port), H).serve_forever()
