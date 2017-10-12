import os

import rospy
import rospkg

rospack = rospkg.RosPack()
nautonomous_configuration_path = rospack.get_path('nautonomous_configuration')

from PIL import Image

import numpy as np  
import matplotlib.pyplot as plt  

import yaml

from coordinate_system import CoordinateSystem
from position import Position
from point2d import Point2D
from vector2d import Vector2D

from nautonomous_map_msgs.srv import CropResponse

import map_size_optimizer

import math
from map_image_coordinate_system import MapImageCoordinateSystem

class ImageCropperService:

	# constants
	image_file_extension_ = ".png"
	config_file_extension_ = ".yaml"

	map_folder_ = "/config/navigation/map/"
	file_name_adjustment_ = "_cropped_"

	# map
	map_image_ = None

	def __init__(self):
		self.load_params()

	def load_params(self):
		self.map_name_ = rospy.get_param('~map_image_name_param', 'amsterdam')
		self.negate_image_ = rospy.get_param('~negate_image_param', False)

	# Load the entire image of amsterdam so we can use it to crop it.
	def load_map_image(self):
	
		full_image_string = nautonomous_configuration_path + self.map_folder_ + self.map_name_ + self.image_file_extension_
		self.image_ = Image.open(full_image_string)

		self.map_data_ = self.open_map_config_file()

		self.map_image_coordinate_system_ = MapImageCoordinateSystem(self.map_data_, self.image_)

	# Extract the cropped image positions based on the positions of the nodes in the map.
	def extract_cropped_image_positions(self, nodes):
		# get map parameters
		resolution = self.map_data_["resolution"]
		map_left = self.map_data_["map_left"]
		map_top = self.map_data_["map_top"]
		map_bottom = self.map_data_["map_bottom"]

		# Get the transformed nodes in the full image frame
		nodes_x = []
		nodes_y = []
		for node in nodes:
			nodes_x.append(node.x - map_left)
			nodes_y.append(map_top - node.y)

		# margin for top, left, bottom and right
		margin = 25 # magic 50 meters boundary

		# find rectangle that fits the path
		left_position = min(nodes_x) - margin
		right_position = max(nodes_x) + margin
		bottom_position = max(nodes_y) + margin
		top_position = min(nodes_y) - margin

		return resolution, left_position, right_position, bottom_position, top_position, CoordinateSystem(map_left, map_top, 0, Position.TOPLEFT)

	# Create the service so other nodes can request to crop the map.
	def execute_service(self, request):
		self.load_map_image()

		route = request.route
		name_map = request.name
		rectangular = request.rectangular

		config_name = nautonomous_configuration_path + self.map_folder_ + self.map_name_ + self.file_name_adjustment_ + name_map + self.config_file_extension_
		print config_name
		if(os.path.isfile(config_name)):
			rospy.loginfo("Image Cropper: Config " + config_name + " already exists, returning existing configuration file.")
			return CropResponse(config_name)
    
		if not rectangular:
			# extract the position of the cropped map
			theta = self.map_image_coordinate_system_.cropped_image_positions(route)

			# save the cropper image based on the positions
			initial_cropped_image = self.map_image_coordinate_system_.initial_crop()
			rotated_image = self.map_image_coordinate_system_.rotate(-theta)
			final_cropped_image, bottom_left_point = self.map_image_coordinate_system_.final_crop()
		else:
			# extract the position of the cropped map
			theta = 0.0
			resolution, left_position, right_position, bottom_position, top_position, coordinate_system = self.extract_cropped_image_positions(route)
			# save the cropper image based on the positions
			final_cropped_image, bottom_left_point = self.crop_image(resolution, left_position, right_position, bottom_position, top_position, coordinate_system)

		# Save cropped image
		self.save_cropped_image(name_map, final_cropped_image)

		# save the config file based on the positions
		config_file_name =  self.save_config_file(name_map, bottom_left_point, theta)

		return CropResponse(config_file_name)

	# Save the cropped image based on the cropped positions
	def crop_image(self, resolution, left_position, right_position, bottom_position, top_position, coordinate_system):
		# create cropped image
		cropped_image = self.image_.crop(
			(
				int(left_position / resolution),  
				int(top_position / resolution), 
				int(right_position / resolution), 
				int(bottom_position / resolution)
			)
		)
		print "coordinate_system: " + str(coordinate_system)
		print "left: " + str(left_position)
		print "top: " + str(top_position)
		print "right: " + str(right_position)
		print "bottom: " + str(bottom_position)

		return cropped_image, Vector2D(Point2D(left_position, bottom_position), coordinate_system, transpose = False)

	# Open the config file from the map image
	def open_map_config_file(self):
		with open(nautonomous_configuration_path + self.map_folder_ + self.map_name_ + self.config_file_extension_) as f:
			self.map_data_ = yaml.safe_load(f)
		return self.map_data_

	# Save the cropped image based on the cropped positions
	def save_cropped_image(self, name_map, image):
		print "Saved cropped image " + (nautonomous_configuration_path + self.map_folder_ + self.map_name_ + self.file_name_adjustment_ + name_map + self.image_file_extension_)
		image.save(nautonomous_configuration_path + self.map_folder_ + self.map_name_ + self.file_name_adjustment_ + name_map + self.image_file_extension_)

	# Save config file based on the cropped positions
	def save_config_file(self, name_map, bottom_left_point, theta):
		del self.map_data_["map_left"]
		del self.map_data_["map_right"]
		del self.map_data_["map_bottom"]
		del self.map_data_["map_top"]

		self.map_data_["image"] = self.map_name_ + self.file_name_adjustment_ + name_map + self.image_file_extension_
		self.map_data_["origin"] = [bottom_left_point.original_point().x(), bottom_left_point.original_point().y(), theta]
		
		self.map_data_["negate"] = int(self.negate_image_) # turns boolean into int (false -> 0 and true -> 1)

		# Save the image and config name
		config_name = nautonomous_configuration_path + self.map_folder_ + self.map_name_ + self.file_name_adjustment_ + name_map + self.config_file_extension_
	
		# Open the config file and dump the data
		with open(config_name, "w") as f:
			yaml.dump(self.map_data_, f)

		# return the image and config file name
		return config_name
