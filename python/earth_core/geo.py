"""WGS84 / ECEF / UTM (Kruger series) / local ENU. Pure stdlib, float64."""
import math

A = 6378137.0
F = 1 / 298.257223563
E2 = F * (2 - F)

def geodetic_to_ecef(lat, lon, h=0.0):
    p, l = math.radians(lat), math.radians(lon)
    n = A / math.sqrt(1 - E2 * math.sin(p) ** 2)
    return ((n + h) * math.cos(p) * math.cos(l),
            (n + h) * math.cos(p) * math.sin(l),
            (n * (1 - E2) + h) * math.sin(p))

def ecef_to_geodetic(x, y, z):
    lon = math.atan2(y, x)
    r = math.hypot(x, y)
    p = math.atan2(z, r * (1 - E2))
    h = 0.0
    for _ in range(10):
        n = A / math.sqrt(1 - E2 * math.sin(p) ** 2)
        h = r / math.cos(p) - n
        p = math.atan2(z, r * (1 - E2 * n / (n + h)))
    return math.degrees(p), math.degrees(lon), h

class LocalFrame:
    """East-North-Up frame at an origin. Floating origin = create a new frame and re-base."""
    def __init__(self, lat, lon, h=0.0):
        self.lat, self.lon, self.h = lat, lon, h
        self._o = geodetic_to_ecef(lat, lon, h)
        p, l = math.radians(lat), math.radians(lon)
        sp, cp, sl, cl = math.sin(p), math.cos(p), math.sin(l), math.cos(l)
        self._e = (-sl, cl, 0.0)
        self._n = (-sp * cl, -sp * sl, cp)
        self._u = (cp * cl, cp * sl, sp)

    def to_local(self, lat, lon, h=0.0):
        d = [a - b for a, b in zip(geodetic_to_ecef(lat, lon, h), self._o)]
        return tuple(sum(d[i] * v[i] for i in range(3)) for v in (self._e, self._n, self._u))

    def from_local(self, e, n, u):
        o = self._o
        x = o[0] + e * self._e[0] + n * self._n[0] + u * self._u[0]
        y = o[1] + e * self._e[1] + n * self._n[1] + u * self._u[1]
        z = o[2] + e * self._e[2] + n * self._n[2] + u * self._u[2]
        return ecef_to_geodetic(x, y, z)

# ---- UTM (Kruger n-series, 3 terms) ----
K0, E0 = 0.9996, 500000.0
_n = F / (2 - F)
_AA = A / (1 + _n) * (1 + _n**2 / 4 + _n**4 / 64)
_al = (_n/2 - 2*_n**2/3 + 5*_n**3/16, 13*_n**2/48 - 3*_n**3/5, 61*_n**3/240)
_be = (_n/2 - 2*_n**2/3 + 37*_n**3/96, _n**2/48 + _n**3/15, 17*_n**3/480)
_de = (2*_n - 2*_n**2/3 - 2*_n**3, 7*_n**2/3 - 8*_n**3/5, 56*_n**3/15)

def utm_zone(lon):
    return int((lon + 180) // 6) % 60 + 1

def utm_forward(lat, lon, zone=None):
    """-> (zone, easting, northing). Southern hemisphere uses false northing 1e7."""
    zone = zone or utm_zone(lon)
    lam = math.radians(lon - ((zone - 1) * 6 - 180 + 3))
    phi = math.radians(lat)
    c = 2 * math.sqrt(_n) / (1 + _n)
    t = math.sinh(math.atanh(math.sin(phi)) - c * math.atanh(c * math.sin(phi)))
    xi_ = math.atan2(t, math.cos(lam))
    eta_ = math.atanh(math.sin(lam) / math.sqrt(1 + t * t))
    xi = xi_ + sum(_al[j-1] * math.sin(2*j*xi_) * math.cosh(2*j*eta_) for j in (1, 2, 3))
    eta = eta_ + sum(_al[j-1] * math.cos(2*j*xi_) * math.sinh(2*j*eta_) for j in (1, 2, 3))
    n0 = 0.0 if lat >= 0 else 1e7
    return zone, E0 + K0 * _AA * eta, n0 + K0 * _AA * xi

def utm_inverse(zone, e, n, north=True):
    xi_ = (n - (0.0 if north else 1e7)) / (K0 * _AA)
    eta_ = (e - E0) / (K0 * _AA)
    xi0 = xi_ - sum(_be[j-1] * math.sin(2*j*xi_) * math.cosh(2*j*eta_) for j in (1, 2, 3))
    eta0 = eta_ - sum(_be[j-1] * math.cos(2*j*xi_) * math.sinh(2*j*eta_) for j in (1, 2, 3))
    chi = math.asin(math.sin(xi0) / math.cosh(eta0))
    phi = chi + sum(_de[j-1] * math.sin(2*j*chi) for j in (1, 2, 3))
    lam = math.atan2(math.sinh(eta0), math.cos(xi0))
    return math.degrees(phi), (zone - 1) * 6 - 180 + 3 + math.degrees(lam)
