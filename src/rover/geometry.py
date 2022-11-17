import numpy as np
import numpy.typing as npt


class Point:
    arr: npt.NDArray[np.float64]

    def __init__(self, x: float, y: float):
        self.arr = np.array([x, y])

    def __repr__(self):
        return f"Point({self.arr[0]}, {self.arr[1]})"

    def __eq__(self, o: object) -> bool:
        if isinstance(o, Point):
            return bool((self.arr == o.arr).all())
        return False

    @staticmethod
    def distance(p1: 'Point', p2: 'Point') -> float:
        """Linear distance between 2d points

        Args:
            p1 (Point): First point
            p2 (Point): Second point

        Returns:
            float: distance
        """
        return float(np.linalg.norm(p1.arr - p2.arr))


def point_to_angle(point: 'Point') -> float:
    """Gets the angle of a point with reference to the x-axis

    Args:
        point (Point): the point

    Returns:
        int: degrees from x-axis
    """
    angle = round(np.rad2deg(np.arctan2(point[1], point[0])) - 90)
    if angle > 180:
        angle -= 360
    elif angle <= -180:
        angle += 360
    return angle


class Line:
    """Generic class for a line or vector"""
    base: Point
    end: Point
    is_vector: bool = False

    def __init__(self, p1: Point, p2: Point) -> None:
        self.base = p1
        self.end = p2

    @classmethod
    def from_slope(cls, p1: Point, slope: float):
        if slope == np.inf:
            return Line(p1, Point(p1.arr[0], p1.arr[1] + 1))
        return Line(p1, Point(p1.arr[0] + 1, p1.arr[1] + slope))

    @classmethod
    def vector(cls, x: float, y: float):
        length = np.linalg.norm([x, y])
        vector = Line(Point(0, 0), Point(float(x / length), float(y / length)))
        vector.is_vector = True
        return vector

    def __repr__(self) -> str:
        return f"Line({self.base}, {self.end})"

    def __eq__(self, other: 'Line') -> bool:
        return isinstance(other, Line) and self.is_vector == other.is_vector \
            and self.base == other.base and self.end == other.end

    def length(self) -> float:
        if self.is_vector:
            raise TypeError("Line.length() can only be called on a line")
        return Point.distance(self.base, self.end)

    def angle(self) -> float:
        if not self.is_vector:
            raise TypeError("Line.angle() can only be called on a vector")

        angle = np.arctan2(self.end.arr[1], self.end.arr[0])
        if angle < 0:
            angle += 2 * np.pi
        return np.rad2deg(angle)

    def midpoint(self) -> Point:
        if self.is_vector:
            raise TypeError("Line.midpoint() can only be called on a line.")
        return Point((self.base.arr[0] + self.end.arr[0]) / 2,
                     (self.base.arr[1] + self.end.arr[1]) / 2)

    def slope(self) -> float:
        if self.is_vector:
            raise TypeError("Line.slope() can only be called on a line.")
        if self.base.arr[0] == self.end.arr[0]:
            return np.inf
        return (self.end.arr[1] - self.base.arr[1]) \
            / (self.end.arr[0] - self.base.arr[0])

    def y_intercept(self) -> float:
        if self.is_vector:
            raise TypeError("Line.y_intercept() can only be called on a line.")
        if self.slope() == np.inf:
            return np.NaN
        return self.base.arr[1] - self.slope() * self.base.arr[0]

    def perpendicular_bisector(self) -> 'Line':
        if self.is_vector:
            raise TypeError("Line.perpendicular_bisector() can only be called on a line.")
        if self.slope() == 0:
            return Line.from_slope(self.midpoint(), np.inf)
        return Line.from_slope(self.midpoint(), -1 / self.slope())

    def intersection(self, other: 'Line') -> Point:
        if self.is_vector:
            raise TypeError("Line.intersection() can only be called on a line.")
        if isinstance(other, Line):
            if self.slope() == other.slope():
                return None
            if self.slope() == np.inf:
                return Point(self.base.arr[0], other.slope() * self.base.arr[0]
                             + other.y_intercept())
            if other.slope() == np.inf:
                return Point(other.base.arr[0], self.slope() * other.base.arr[0]
                             + self.y_intercept())
            x_val = (other.y_intercept() - self.y_intercept()) \
                / (self.slope() - other.slope())
            y_val = self.slope() * x_val + self.y_intercept()
            return Point(x_val, y_val)
        elif isinstance(other, Point):
            if Line(self.base, other).slope() != self.slope():
                return None
            return other
        else:
            raise TypeError("Line.intersection() can only be called with Line or Point")


class Arc:
    """This arc class only works for forward arcs. If you do a backward arc,
    turn around first before you do it.  """
    center: Point
    radius: float
    angle: float
    direction: str

    def __init__(self, center: Point, radius: float, angle: float, direction: str = 'right'):
        self.center = center
        self.radius = radius
        self.angle = angle
        self.direction = direction

    @classmethod
    def from_points(cls, p1: Point, p2: Point, p3: Point):
        direction = 'left' if p3.arr[0] < p1.arr[0] else 'right'

        # get the lines that intersect with the center
        l1 = Line(p1, p2).perpendicular_bisector()
        l2 = Line(p2, p3).perpendicular_bisector()

        # find the center point and radius
        center = l1.intersection(l2)
        center.arr[0] = round(center.arr[0], 2)
        center.arr[1] = round(center.arr[1], 2)
        radius = Point.distance(p1, center)
        radius = round(radius, 2)

        # law of cosines for the angle
        a = b = radius
        c = Point.distance(p1, p3)
        angle = np.rad2deg(np.arccos((c**2 - a**2 - b**2) / (-2 * a * b)))
        angle = round(angle)

        if p3.arr[1] < p1.arr[1]:
            angle = 360 - angle

        return cls(center, radius, angle, direction)

    def __repr__(self) -> str:
        return f"Arc({self.center}, {self.radius}, {self.angle}, {self.direction})"

    def __eq__(self, other: 'Arc') -> bool:
        return isinstance(other, Arc) and self.center == other.center \
            and self.radius == other.radius and self.angle == other.angle \
            and self.direction == other.direction

    def tangent(self, point: Point) -> Line:
        if Point.distance(self.center, point) != self.radius:
            raise ValueError("Point is not on the arc")
        temp_line = Line(self.center, point)
        return Line.from_slope(point, temp_line.slope())

    def arc_length(self) -> float:
        return self.radius * np.deg2rad(self.angle)
