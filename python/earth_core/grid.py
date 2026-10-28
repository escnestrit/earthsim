"""Stable global cell ids: UTM zone + integer cell index at fixed size (default 256 m).
Known limitation: cells never span zones; zone seams need a dedicated strategy (NOT implemented)."""
from dataclasses import dataclass
from .geo import utm_forward, utm_inverse

@dataclass(frozen=True)
class CellId:
    zone: int
    north: bool
    ix: int
    iy: int
    size: int = 256
    def __str__(self):
        return f"u{self.zone}{'N' if self.north else 'S'}/{self.size}/{self.ix}_{self.iy}"

def cell_of(lat, lon, size=256, zone=None):
    z, e, n = utm_forward(lat, lon, zone)
    return CellId(z, lat >= 0, int(e // size), int(n // size), size)

def cell_bounds(c):
    return (c.ix * c.size, c.iy * c.size, (c.ix + 1) * c.size, (c.iy + 1) * c.size)

def cell_center_latlon(c):
    e0, n0, e1, n1 = cell_bounds(c)
    return utm_inverse(c.zone, (e0 + e1) / 2, (n0 + n1) / 2, c.north)

def neighbors(c, ring=1):
    return [CellId(c.zone, c.north, c.ix + dx, c.iy + dy, c.size)
            for dx in range(-ring, ring + 1) for dy in range(-ring, ring + 1) if (dx, dy) != (0, 0)]
