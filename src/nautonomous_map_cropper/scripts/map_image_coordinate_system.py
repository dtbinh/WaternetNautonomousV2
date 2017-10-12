
from coordinate_system import CoordinateSystem
from position import Position

from rotation import AngleUnit, Rotation

import map_size_optimizer

from point2d import Point2D
from vector2d import Vector2D

from image_box import ImageBox

import math

import numpy as np  
import matplotlib.pyplot as plt  

class MapImageCoordinateSystem():
    def __init__(self, map_data, image):

        self.resolution_ = map_data["resolution"]
        map_utm_left = map_data["map_left"]
        map_utm_top = map_data["map_top"]
        map_utm_bottom = map_data["map_bottom"]
        map_utm_right = map_data["map_right"]

        map_utm_width = map_utm_right - map_utm_left
        map_utm_height = map_utm_top - map_utm_bottom

        map_pixel_width = map_utm_width / self.resolution_
        map_pixel_height = map_utm_height / self.resolution_

        map_utm_center_x = map_utm_left + map_utm_height / 2
        map_utm_center_y = map_utm_bottom + map_utm_width / 2

        self.top_left_utm_coordinate_system_ = CoordinateSystem(map_utm_left, map_utm_top, 0, Position.TOPLEFT)
        self.center_utm_coordinate_system_ = CoordinateSystem(map_utm_center_x, map_utm_center_y, 0, Position.CENTER)

        self.map_image_ = image

    # def crop cropping region
    def crop(self, top_left_vector, width, height):

        # image crop
        x = top_left_vector.x()
        y = top_left_vector.y()

        self.map_image_ = self.map_image_.crop(
			(
				int(x),  
				int(y), 
				int(x + width), 
				int(y + height)
			)
		)

    def rotate(self, theta):
        # rotation = Rotation(theta, AngleUnit.RADIANS)

        self.map_image_ = self.map_image_.rotate(math.degrees(theta), expand=1)

        image_width, image_height = self.map_image_.size
        utm_width = image_width * self.resolution_
        utm_height = image_height * self.resolution_

        # define new top_left coordinate system
        center_origin = self.center_utm_coordinate_system_.origin()
        self.center_utm_coordinate_system_ = CoordinateSystem(center_origin.x(), center_origin.y(), theta, Position.CENTER)

        self.top_left_utm_coordinate_system_ = CoordinateSystem(center_origin.x() + math.sin(theta) * utm_height / 2 - math.cos(theta) * utm_width / 2, center_origin.y() + math.cos(theta) * utm_height / 2 + math.sin(theta) * utm_width / 2, -theta, Position.TOPLEFT)

        self.crop_image_box_ = self.rotated_image_box_

        # rotate all points noted 
        return self.map_image_ 

    # Extract the cropped image positions based on the positions of the nodes in the map.
    def cropped_image_positions(self, route):
        # get map parameters

        points = []

        for pose in route:
            points.append(Point2D(pose.x, pose.y))
            
        theta, a, b = map_size_optimizer.least_squares_theta(points)

        top_left_point, top_right_point, bottom_left_point, bottom_right_point = self.transformed_boundaries(route, a, b)

        top_left_rotated_center_vector = self.center_utm_coordinate_system_.point_to_vector(top_left_point)
        top_right_rotated_center_vector = self.center_utm_coordinate_system_.point_to_vector(top_right_point)
        bottom_left_rotated_center_vector = self.center_utm_coordinate_system_.point_to_vector(bottom_left_point)
        bottom_right_rotated_center_vector = self.center_utm_coordinate_system_.point_to_vector(bottom_right_point)

        self.rotated_image_box_ = ImageBox(top_left_rotated_center_vector, top_right_rotated_center_vector, bottom_left_rotated_center_vector, bottom_right_rotated_center_vector, Position.CENTER)

        top_left_center_vector, top_right_center_vector, bottom_left_center_vector, bottom_right_center_vector = self.boundaries(top_left_rotated_center_vector, top_right_rotated_center_vector, bottom_left_rotated_center_vector, bottom_right_rotated_center_vector)

        self.crop_image_box_ = ImageBox(top_left_center_vector, top_right_center_vector, bottom_left_center_vector, bottom_right_center_vector, Position.CENTER)

        return theta

    def transformed_boundaries(self, route, a, b):

        theta = math.atan2(a, 1)

        min_x = float('inf')
        min_x_vector = None
        max_x = float('-inf')
        max_x_vector = None

        min_y = float('inf')
        min_y_vector = None
        max_y = float('-inf')
        max_y_vector = None

        for pose in route:
            point = Point2D(pose.x, pose.y)
            
            transformed_vector = self.transform_point_to_rotated_vector(point, -theta, b)

            x = transformed_vector.x()
            y = transformed_vector.y()

            if x < min_x:
                min_x = x
                min_x_transformed_vector = transformed_vector

            if y < min_y:
                min_y = y
                min_y_transformed_vector = transformed_vector

            if x > max_x:
                max_x = x
                max_x_transformed_vector = transformed_vector

            if y > max_y:
                max_y = y
                max_y_transformed_vector = transformed_vector
        
        margin = 25

        top_left_transformed_point     = Point2D(min_x_transformed_vector.x() - margin, max_y_transformed_vector.y() + margin)
        top_right_transformed_point    = Point2D(max_x_transformed_vector.x() + margin, max_y_transformed_vector.y() + margin)
        bottom_left_transformed_point  = Point2D(min_x_transformed_vector.x() - margin, min_y_transformed_vector.y() - margin)
        bottom_right_transformed_point = Point2D(max_x_transformed_vector.x() + margin, min_y_transformed_vector.y() - margin)

        top_left_transformed_vector     = Vector2D(top_left_transformed_point,      self.center_utm_coordinate_system_, False)
        top_right_transformed_vector    = Vector2D(top_right_transformed_point,     self.center_utm_coordinate_system_, False)
        bottom_left_transformed_vector  = Vector2D(bottom_left_transformed_point,   self.center_utm_coordinate_system_, False)
        bottom_right_transformed_vector = Vector2D(bottom_right_transformed_point,  self.center_utm_coordinate_system_, False)

        top_left_point      = self.retransform_rotated_vector_to_point(top_left_transformed_vector,     theta, b)
        top_right_point     = self.retransform_rotated_vector_to_point(top_right_transformed_vector,    theta, b)
        bottom_left_point   = self.retransform_rotated_vector_to_point(bottom_left_transformed_vector,  theta, b)
        bottom_right_point  = self.retransform_rotated_vector_to_point(bottom_right_transformed_vector, theta, b)

        return top_left_point, top_right_point, bottom_left_point, bottom_right_point

    def boundaries(self, top_left_vector, top_right_vector, bottom_left_vector, bottom_right_vector):

        vectors = [top_left_vector, top_right_vector, bottom_left_vector, bottom_right_vector]

        min_x = float('inf')
        min_x_vector = None
        max_x = float('-inf')
        max_x_vector = None

        min_y = float('inf')
        min_y_vector = None
        max_y = float('-inf')
        max_y_vector = None

        for vector in vectors:
            x = vector.x()
            y = vector.y()

            if x < min_x:
                min_x = x

            if y < min_y:
                min_y = y

            if x > max_x:
                max_x = x

            if y > max_y:
                max_y = y

        top_left_vector = Vector2D(Point2D(min_x, max_y), top_left_vector.coordinate_system(), False)
        top_right_vector = Vector2D(Point2D(max_x, max_y), top_right_vector.coordinate_system(), False)
        bottom_left_vector = Vector2D(Point2D(min_x, min_y), bottom_left_vector.coordinate_system(), False)
        bottom_right_vector = Vector2D(Point2D(max_x, min_y), bottom_right_vector.coordinate_system(), False)

        return top_left_vector, top_right_vector, bottom_left_vector, bottom_right_vector

    def transform_point_to_rotated_vector(self, point, theta, b):
        rotation = Rotation(theta, AngleUnit.RADIANS)
        point = Point2D(point.x(), point.y() - b)
        vector = Vector2D(point, CoordinateSystem(0, 0, 0, Position.CENTER))
        return rotation.rotate_vector(vector)

    def retransform_rotated_vector_to_point(self, vector, theta, b):
        rotation = Rotation(theta, AngleUnit.RADIANS)
        vector = rotation.rotate_vector(vector)
        point = vector.original_point() 
        return Point2D(vector.x(), vector.y() + b)

    def initial_crop(self):
        
        utm_width = self.crop_image_box_.width()
        utm_height = self.crop_image_box_.height()
        
        top_left = self.top_left_utm_coordinate_system_.convert_vector(self.crop_image_box_.top_left())
        
        pixel_top_left = top_left.scale(1 / self.resolution_)
        pixel_width = utm_width / self.resolution_
        pixel_height = utm_height /  self.resolution_

        self.crop(pixel_top_left, pixel_width, pixel_height)

        top_left_original = top_left.original_point()

        self.top_left_utm_coordinate_system_ = CoordinateSystem(top_left_original.x(), top_left_original.y(), 0, Position.TOPLEFT)
        self.center_utm_coordinate_system_ = CoordinateSystem(top_left_original.x() + utm_width / 2, top_left_original.y() - utm_height / 2, 0, Position.CENTER)

        return self.map_image_

    def final_crop(self):

        top_left = self.top_left_utm_coordinate_system_.convert_vector(self.crop_image_box_.top_left())
        top_right = self.top_left_utm_coordinate_system_.convert_vector(self.crop_image_box_.top_right())
        bottom_left = self.top_left_utm_coordinate_system_.convert_vector(self.crop_image_box_.bottom_left())
        bottom_right = self.top_left_utm_coordinate_system_.convert_vector(self.crop_image_box_.bottom_right())

        rectified_image_box = ImageBox(top_left, top_right, bottom_left, bottom_right, Position.TOPLEFT)

        utm_width = rectified_image_box.width()
        utm_height = rectified_image_box.height()

        pixel_top_left = top_left.scale(1 / self.resolution_)
        pixel_width = utm_width / self.resolution_
        pixel_height = utm_height /  self.resolution_

        self.crop(pixel_top_left, pixel_width, pixel_height)

        return self.map_image_, self.crop_image_box_.bottom_left()
    
