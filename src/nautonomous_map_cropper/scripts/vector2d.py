from point2d import Point2D
from position import Position

import math

class Vector2D():
    def __init__(self, point, coordinate_system, transpose = True, invert_y = False):
        if transpose:
            new_x = point.x() - coordinate_system.origin().x()
            new_y = point.y() - coordinate_system.origin().y()
            theta = coordinate_system.theta()
            self.x_ = math.cos(theta) * new_x + math.sin(theta) * new_y
            self.y_ = math.cos(theta) * new_y - math.sin(theta) * new_x
        else:
            self.x_ = point.x()
            self.y_ = point.y()
            
        self.coordinate_system_ = coordinate_system

        if(invert_y):
            self.y_ = -self.y_
    
    def x(self):
        return self.x_

    def y(self):
        return self.y_

    def scale(self, scalar):
        new_x = self.x_ * scalar
        new_y = self.y_ * scalar

        return Vector2D(Point2D(new_x, new_y), self.coordinate_system(), transpose = False)

    def coordinate_system(self):
        return self.coordinate_system_

    def original_point(self):
        coordinate_system_origin = self.coordinate_system_.origin()
        if(self.coordinate_system_.position() == Position.CENTER):
            return Point2D(self.x() + coordinate_system_origin.x(), self.y() + coordinate_system_origin.y())
        elif(self.coordinate_system_.position() == Position.TOPLEFT):
            return Point2D(self.x() + coordinate_system_origin.x(), (-self.y()) + coordinate_system_origin.y())
    
    
    def __add__(self, other):
        if(self.coordinate_system_ != other):
            raise Exception("Adding two vectors with different coordinate systems")
            return None
        
        addition_point = Point2D(self.x() + other.x(), self.y() + other.y())

        return Vector2D(addition_point, self.coordinate_system())    
    
    def __sub__(self, other):
        if(self.coordinate_system_ != other):
            raise Exception("Subtracting two vectors with different coordinate systems")
            return None
        
        subtraction_point = Point2D(self.x() - other.x(), self.y() - other.y())

        return Vector2D(subtraction_point, self.coordinate_system())    

    def __repr__(self):
        return "("+ str(self.x()) + "," + str(self.y()) + ")" + " CS: " + str(self.coordinate_system()) + " original " + str(self.original_point())