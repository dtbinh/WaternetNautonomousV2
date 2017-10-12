
from point2d import Point2D
from vector2d import Vector2D

from position import Position

from rotation import AngleUnit, Rotation

class CoordinateSystem():
    def __init__(self, x, y, theta, position):
        self.position_ = position

        self.origin_ = Point2D(x, y)
    
        self.theta_ = theta

        self.rotation_ = Rotation(theta, AngleUnit.RADIANS)
    
    def origin(self):
        return self.origin_

    def position(self):
        return self.position_

    def theta(self):
        return self.theta_

    def point_to_vector(self, point):
        if self.position_ == Position.CENTER:
            vector = Vector2D(point, self)
            rotated_vector = self.rotation_.rotate_vector(vector)
            return rotated_vector
        
    def convert_vector(self, vector):
        if self.position_ == Position.TOPLEFT and vector.coordinate_system().position() == Position.CENTER:
            original_point = vector.original_point()
            new_vector = Vector2D(original_point, self, invert_y = True)
            return new_vector

    def points_to_vectors(self, points):
        vectors = []

        for point in points:
            vectors.append(self.point_to_vector(point))

        return vectors

    def __repr__(self):
        return "("+ str(self.origin()) + "," + str(self.position()) + ")" + " theta " + str(self.theta_)