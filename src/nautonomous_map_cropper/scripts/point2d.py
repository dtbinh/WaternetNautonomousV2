import math

from geometry_msgs.msg import Pose2D

class Point2D():
    def __init__(self, x, y):
        self.x_ = x
        self.y_ = y

    # X
    def x(self):
        return self.x_

    # Y
    def y(self):
        return self.y_

        
    # Create a ROS Geometry_msgs Pose2d message from a point object.
    def to_Pose2D(self, theta = 0):
        return Pose2D(self.x_, self.y_, theta)

    # Calculate the euclidian distance from the x and y coordinates.
    def euclidean_distance(self, other):
        return math.sqrt(math.pow(self.x() - other.x(), 2) + math.pow(self.y() - other.y(), 2))

    # Get the length of the point, by interpreting it as a vertex from (0,0) to (x,y)
    def length(self):
        return math.sqrt(math.pow(self.x(), 2) + math.pow(self.y(), 2))

    # Scale the point using a scalar
    def scale(self, scalar):
        return Point2D(self.x() * scalar, self.y() * scalar)

    # Calculate the cross product of the two objects.
    def cross_product(self, other):
        return self.x() * other.x()  + self.y() * other.y()

    def __eq__(self, other):
        return self.x() == other.x() and self.y() == other.y()

    def __add__(self, other):
        return Point2D(self.x() + other.x(), self.y() + other.y())
    
    def __sub__(self, other):
        return Point2D(self.x() - other.x(), self.y() - other.y())

    def __repr__(self):
        return "("+ str(self.x()) + "," + str(self.y()) + ")" 