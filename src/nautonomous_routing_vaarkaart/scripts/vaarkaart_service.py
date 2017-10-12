
import rospy

import astar_route
import graph_adapter
import graph_helper
import vaarkaart_visualizer

from waternet_vertex import WaternetVertex

from nautonomous_routing_msgs.srv import Route, RouteResponse
from geometry_msgs.msg import Pose2D

routing_vaarkaart_service_namespace = "route"

class VaarkaartService:

	def __init__(self, routing_graph):
		self.vaarkaart_graph = routing_graph

		self.debug_visualization = rospy.get_param('~debug', False)
		
		# Setup vaarkaart routing service
		vaarkaart_routing_service_server = rospy.Service(routing_vaarkaart_service_namespace, Route, self.routing_vaarkaart_service)

	# Find the route to the destination.
	def routing_vaarkaart_service(self, request):
		# Create the start coordinate and edge
		start_vertex = WaternetVertex(request.start.x, request.start.y)
		start_edge, start_distance = graph_helper.closest_edge(self.vaarkaart_graph, start_vertex)

		# Create the destination coordinate and edge
		destination_vertex = WaternetVertex(request.destination.x, request.destination.y)
		destination_edge, destination_distance = graph_helper.closest_edge(self.vaarkaart_graph, destination_vertex)

		search_graph = graph_adapter.create_search_graph(self.vaarkaart_graph, start_edge, start_vertex, destination_edge, destination_vertex)
		
		# Temporarily variables to remember the shortest route
		route, route_ids, route_cost = astar_route.astar_search(search_graph, start_vertex, destination_vertex)

		# Create UTM route from astar route
		route_poses = []

		for vertex in route:
			route_poses.append(vertex.to_Pose2D())

		# Add the start and destination to the route. (unnecessary why would you navigate to a position you are already are.)
		# start_pose = Pose2D(request.start.x, request.start.y, 0)
		# if start_pose not in route_poses:
		# 	route_poses.insert(0, start_pose)
		
		destination_pose = Pose2D(request.destination.x, request.destination.y, 0)
		if destination_pose not in route_poses:
			route_poses.append(destination_pose)

		# Visualize kaart route
		if self.debug_visualization:
			vaarkaart_visualizer.visualize_route(search_graph, start_vertex, destination_vertex, route_poses)


		print "Response: " + str(route_poses) + " " + str(route_ids)	
		return RouteResponse(route_poses, route_ids)

