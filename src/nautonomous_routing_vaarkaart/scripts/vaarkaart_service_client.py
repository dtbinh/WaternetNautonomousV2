import rospy
from vaarkaart_service import routing_vaarkaart_service_namespace
import random

from nautonomous_routing_msgs.srv import Route

def vaarkaart_routing_service_client(vaarkaart_graph):

	rospy.wait_for_service(routing_vaarkaart_service_namespace)

	# Setup vaarkaart routing client
	vaarkaart_routing_service_client = rospy.ServiceProxy(routing_vaarkaart_service_namespace, Route)

	
	try:
		# Try to make a request to the vaarkaart routing service with a randomly chosen start and destination position.
		start_vertex = vaarkaart_graph.vertex(random.choice(vaarkaart_graph.edges()).start_vertex_str())
		destination_vertex = vaarkaart_graph.vertex(random.choice(vaarkaart_graph.edges()).destination_vertex_str())

		while(start_vertex == destination_vertex):
			destination_vertex = vaarkaart_graph.vertex(random.choice(vaarkaart_graph.edges()).destination_vertex_str())

		# Execute Service with simulated client
		print "Requesting vaarkaart routing"
		vaarkaart_routing_service_response = vaarkaart_routing_service_client(start_vertex.to_Pose2D(), destination_vertex.to_Pose2D())

		print "Success vaarkaart routing service client response: "
		print vaarkaart_routing_service_response.route

		print "Size route: " + len(vaarkaart_routing_service_response.route)
		print "Size edges: " + len(vaarkaart_routing_service_response.route_ids) 

	except rospy.ServiceException, e:
		print "Service call for vaarkaart routing failed: %s" % e
