#!/usr/bin/env python
import sys
import os

# Prevent pyc files being generated!
sys.dont_write_bytecode = True

import numpy as np  
import matplotlib.pyplot as plt  

import math

from point2d import Point2D

def least_squares_theta(points):

    sum_x = 0.0
    sum_x_squared = 0.0
    sum_y = 0.0
    sum_x_y = 0.0

    n = len(points)

    x_list = []
    y_list = []

    for point in points:
        x = point.x()
        y = point.y()

        x_list.append(x)
        y_list.append(y)

        sum_x += x
        sum_x_squared += x * x

        sum_y += y
        sum_x_y += x * y

    determinant = (n * sum_x_squared - sum_x * sum_x)

    a = (n * sum_x_y - sum_x * sum_y) / determinant
    b = (sum_y * sum_x_squared - sum_x * sum_x_y) / determinant

    theta = math.atan2(a, 1)
    return theta, a, b

def transform_point(point, theta, width, height):
    x = point.x()
    y = point.y()

    new_x = x
    new_y = y

    if theta < 0.0:
        new_x = x * math.cos(theta) + y * math.sin(theta)
        new_y = (height - x) * math.sin(theta) + y * cos(theta)
    elif theta > 0.0:
        new_x = x * math.cos(theta) + (width - y) * math.sin(theta)
        new_y = x * math.sin(theta) + y * math.cos(theta)

    return Point2D(new_x, new_y)

def transform_points(top_left_point, bottom_right_point, theta, width, height):
    top_left_point = transform_point(top_left_point, theta, width, height)    
    bottom_right_point = transform_point(bottom_right_point, theta, width, height)    
    return top_left_point, bottom_right_point