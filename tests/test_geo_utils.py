import pytest

from geo_utils import LocalTangentPlane


@pytest.mark.parametrize("lat0,lon0", [(55.76, 37.57), (0.0, 0.0), (-33.9, 151.2)])
def test_round_trip(lat0, lon0):
    plane = LocalTangentPlane(lat0, lon0)
    lat, lon = lat0 + 0.02, lon0 - 0.03
    assert plane.to_geodetic(*plane.to_local(lat, lon)) == pytest.approx((lat, lon), abs=1e-9)
    assert plane.to_local(*plane.to_geodetic(1234.5, -987.6)) == pytest.approx((1234.5, -987.6), abs=1e-6)


def test_origin_maps_to_zero():
    assert LocalTangentPlane(55.76, 37.57).to_local(55.76, 37.57) == (0.0, 0.0)


def test_one_degree_of_latitude_is_about_111_2_km():
    # Meridian degree length varies from 110.57 km (equator) to 111.69 km (poles)
    for lat0 in (0.0, 45.0, 60.0):
        _, north = LocalTangentPlane(lat0, 10.0).to_local(lat0 + 1.0, 10.0)
        assert north == pytest.approx(111_200.0, rel=0.01)


def test_east_is_positive_towards_larger_longitude():
    east, north = LocalTangentPlane(55.0, 37.0).to_local(55.0, 37.001)
    assert east > 0 and north == 0.0
    assert east == pytest.approx(63.9, rel=0.01)  # 111.3 km * cos(55 deg) / 1000


def test_invalid_origin_rejected():
    with pytest.raises(ValueError):
        LocalTangentPlane(90.0, 0.0)
