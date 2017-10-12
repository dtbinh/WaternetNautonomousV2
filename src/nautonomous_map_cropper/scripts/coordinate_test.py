from point2d import Point2D

from coordinate_system import Position, CoordinateSystem

from rotation import AngleUnit, Rotation
from geometry_msgs.msg import Pose2D

from PIL import Image

from map_image_coordinate_system import MapImageCoordinateSystem

route = [Pose2D(628329,5803523,0), Pose2D(628649, 5803234,0), Pose2D(629094, 5803149,0), Pose2D(629497,5803283,0)]

points = []

for route_pose in route:
    points.append(Point2D(route_pose.x, route_pose.y))

map_left = 622457
map_right = 632457
map_bottom = 5798303
map_top = 5808303

full_image_string = "/home/waternet/waternet/nautonomous/workspaces/default/src/WaternetNautonomous/src/nautonomous_configuration/config/navigation/map/amsterdam.png"

image = Image.open(full_image_string)

map_image_coordinate_system = MapImageCoordinateSystem(map_top, map_left, map_bottom, map_right, image, points)