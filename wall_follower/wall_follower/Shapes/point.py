import math


def rotate_vector(vector, angle):
    radian = math.radians(angle)
    cos = math.cos(radian)
    sin = math.sin(radian)
    x = vector.x * cos + vector.y * sin
    y = vector.x * -sin + vector.y * cos
    return Point(x, y)


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)

    def __mul__(self, other):
        if isinstance(other, (int, float)):
            return Point(self.x * other, self.y * other)
        return Point(self.x * other.x, self.y * other.y)

    def __truediv__(self, other):
        if isinstance(other, (int, float)):
            return Point(self.x / other, self.y / other)
        return Point(self.x / other.x, self.y / other.y)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __str__(self):
        return f'Point<x: {self.x}, y: {self.y}>'

    def __repr__(self):
        return str(self)

    @property
    def normalized(self):
        normalizer = self.dist
        if normalizer == 0:
            return Point(0, 0)
        return Point(self.x / normalizer, self.y / normalizer)

    @property
    def dist(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)

    def to_array(self) -> list:
        return [self.x, self.y]

    def rotate(self, angle):
        new_point = rotate_vector(self, angle)
        self.x = new_point.x
        self.y = new_point.y

    @staticmethod
    def zero():
        return Point(0, 0)

    @staticmethod
    def up():
        return Point(0, 1)

    @staticmethod
    def right():
        return Point(1, 0)