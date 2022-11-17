import numpy as np
import pytest
from rover.geometry import Point, Line, Arc, point_to_angle


def test_point():
    p1 = Point(1, 2)
    p2 = Point(1, 3)

    assert Point.distance(p1, p2) == 1.0
    p3 = Point(1, 2)
    assert p1 == p3
    assert p1.__repr__() == 'Point(1, 2)'
    assert (p1 == 5) is False


def test_point_to_angle():
    assert round(point_to_angle((1, -1))) == -135
    assert round(point_to_angle((1, 0))) == -90
    assert round(point_to_angle((1, 1))) == -45
    assert round(point_to_angle((0, 1))) == 0
    assert round(point_to_angle((-1, 1))) == 45
    assert round(point_to_angle((-1, 0))) == 90
    assert round(point_to_angle((-1, -1))) == 135
    assert round(point_to_angle((0, -1))) == 180


def test_line_vert():
    p1 = Point(10, 5)
    p2 = Point(5, 6)
    l1 = Line(Point(1, 2), Point(1, 3))
    l2 = Line.from_slope(Point(1, 2), np.inf)
    l3 = Line.from_slope(Point(0, 5), 0)

    assert l1.is_vector is False
    assert l1.__repr__() == 'Line(Point(1, 2), Point(1, 3))'
    assert l1.length() == 1.0
    with pytest.raises(TypeError):
        l1.angle()
    assert l1.midpoint() == Point(1, 2.5)
    assert l1.slope() == np.inf
    assert l1.y_intercept() is np.NaN
    assert l1.perpendicular_bisector() == Line.from_slope(l1.midpoint(), 0)
    assert l1.intersection(l3) == Point(1, 5)
    assert l1.intersection(l2) is None
    with pytest.raises(TypeError):
        l1.intersection(5)

    assert l2.is_vector is False
    assert l2.slope() == np.inf
    assert l2.y_intercept() is np.NaN
    assert l2.__repr__() == 'Line(Point(1, 2), Point(1, 3))'

    assert l3.perpendicular_bisector() == Line.from_slope(l3.midpoint(), np.inf)
    assert l3.intersection(p1) == p1
    assert l3.intersection(p2) is None
    assert l3.intersection(l1) == Point(1, 5)


def test_line_vector():
    v1 = Line.vector(4, 4)
    v2 = Line.vector(1, -1)

    assert v1.is_vector
    with pytest.raises(TypeError):
        v1.length()
    assert v1.angle() == 45.0
    assert v2.angle() == 315.0
    with pytest.raises(TypeError):
        v1.midpoint()
    with pytest.raises(TypeError):
        v1.slope()
    with pytest.raises(TypeError):
        v1.y_intercept()
    with pytest.raises(TypeError):
        v1.perpendicular_bisector()
    with pytest.raises(TypeError):
        v1.intersection(Point(0, 0))


def test_arc():
    a1 = Arc(Point(0, 0), 1, 90)
    a2 = Arc.from_points(Point(-1, 0), Point(1 / (2**0.5), -1 / (2**0.5)), Point(0, 1))

    assert a1 == a2
    assert a1.__repr__() == 'Arc(Point(0, 0), 1, 90)'
    assert a2.tangent(Point(0, 1)) == Line.from_slope(Point(0, 1), np.inf)
    with pytest.raises(ValueError):
        a2.tangent(Point(0, 0))

    a1 = Arc.from_points(Point(0, 0), Point(-1.174, 0.984), Point(-1.940, -0.342))
    # print(a1.radius, a1.center, a1.angle, a1.direction)
