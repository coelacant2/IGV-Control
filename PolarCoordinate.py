import math

class PolarCoordinate:
    radius = 0.0
    angle  = 0.0

    def __init__(self, radius: float, angle: float):
        self.radius = radius
        self.angle  = angle

    @staticmethod
    def fromVector2D(x: float, y: float):
        return PolarCoordinate(pow(pow(x, 2.0) + pow(y, 2.0), 0.5), atan2(y, x) * 180.0 / math.pi)
