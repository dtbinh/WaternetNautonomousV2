
from point2d import Point2D
from vector2d import Vector2D
from enum import Enum
import math


class AngleUnit(Enum):
    DEGREES = 0,
    RADIANS = 1


class Rotation():

    def __init__(self, theta, unit):
        if unit == AngleUnit.RADIANS:
            self.theta_ = theta
        elif unit == AngleUnit.DEGREES:
            self.theta_ = math.radians(theta)

    def rotate_vector(self, vector):
        new_x = round(vector.x() * math.cos(self.theta_) - vector.y() * math.sin(self.theta_), 2)
        new_y = round(vector.x() * math.sin(self.theta_) + vector.y() * math.cos(self.theta_), 2)
        
        point = Point2D(new_x, new_y)
        return Vector2D(point, vector.coordinate_system(), False)

    def rotate_vectors(self, vectors):
        rotated_vectors = []

        for vector in vectors:
            rotated_vectors.append(self.rotate_vector(vector))

        return rotated_vectors

    # def rotate_coordinate_system(self, top_left_coordinate_system, center_coordinate_system):
        