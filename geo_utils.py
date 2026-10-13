"""Geodetic helpers: conversion between latitude/longitude and local ENU meters."""

from __future__ import annotations

import math


class LocalTangentPlane:
    """
    Local east/north plane anchored at a reference point (WGS84 ellipsoid).

    Latitude and longitude are decimal degrees. At the reference latitude the
    meridian radius of curvature M and the prime vertical radius N are computed
    from the WGS84 ellipsoid, and the offsets are linearized:

        north = (lat - lat0) * M
        east  = (lon - lon0) * N * cos(lat0)

    Accuracy limits:
        * Within a few kilometers of the reference point the error is at the
          centimeter level (well below GPS noise).
        * The error grows roughly quadratically with distance (the cos(lat)
          factor is frozen at lat0); it reaches meters at tens of kilometers.
        * Not suitable near the poles or across the antimeridian (+-180 deg).
        * Heights are ignored; the plane lies on the ellipsoid surface.
    """

    SEMI_MAJOR_AXIS_M = 6378137.0
    FLATTENING = 1.0 / 298.257223563

    def __init__(self, origin_latitude_deg: float, origin_longitude_deg: float) -> None:
        if not -90.0 < origin_latitude_deg < 90.0:
            raise ValueError("origin latitude must be within (-90, 90) degrees")
        if not -180.0 <= origin_longitude_deg <= 180.0:
            raise ValueError("origin longitude must be within [-180, 180] degrees")
        self._lat0 = origin_latitude_deg
        self._lon0 = origin_longitude_deg

        e2 = self.FLATTENING * (2.0 - self.FLATTENING)  # first eccentricity squared
        sin_lat = math.sin(math.radians(origin_latitude_deg))
        w = math.sqrt(1.0 - e2 * sin_lat ** 2)
        a = self.SEMI_MAJOR_AXIS_M
        meridian_radius = a * (1.0 - e2) / w ** 3
        prime_vertical_radius = a / w
        # Meters per degree of latitude and longitude at the origin
        self._m_per_deg_lat = math.radians(1.0) * meridian_radius
        self._m_per_deg_lon = (math.radians(1.0) * prime_vertical_radius
                               * math.cos(math.radians(origin_latitude_deg)))

    @property
    def origin(self) -> tuple[float, float]:
        """Reference point as (latitude_deg, longitude_deg)."""
        return self._lat0, self._lon0

    def to_local(self, latitude_deg: float, longitude_deg: float) -> tuple[float, float]:
        """Convert latitude/longitude to (east, north) meters from the origin."""
        east = (longitude_deg - self._lon0) * self._m_per_deg_lon
        north = (latitude_deg - self._lat0) * self._m_per_deg_lat
        return east, north

    def to_geodetic(self, east_m: float, north_m: float) -> tuple[float, float]:
        """Convert (east, north) meters from the origin to (latitude_deg, longitude_deg)."""
        latitude = self._lat0 + north_m / self._m_per_deg_lat
        longitude = self._lon0 + east_m / self._m_per_deg_lon
        return latitude, longitude
