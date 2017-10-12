import utm

from waternet_vertex import WaternetVertex
from point import Point
from geometry_msgs.msg import Pose2D

# Convert GPS string to UTM
def convert_GPS_json_to_UTM_point(position_json):
	utm_coordinate = utm.from_latlon(float(position_json["geo_lat"]), float(position_json["geo_lng"]))[:2] # only use the first two elements of the tuple: easting and northing (in that order)
	
	return Point(utm_coordinate[0], utm_coordinate[1])

# Convert GPS string to UTM
def convert_GPS_json_to_UTM_pose2d(position_json):
	utm_coordinate = utm.from_latlon(float(position_json["geo_lat"]), float(position_json["geo_lng"]))[:2] # only use the first two elements of the tuple: easting and northing (in that order)
	
	return Pose2D(utm_coordinate[0], utm_coordinate[1], 0)