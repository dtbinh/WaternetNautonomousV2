#!/usr/bin/env python

import sys
import os
# Prevent pyc files being generated!
sys.dont_write_bytecode = True

sys.path.append(os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + '/../lib/vaarkaart'))
sys.path.append(os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + '/../lib/astar'))
sys.path.append(os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + '/../lib/graph'))

import rospy

import vaarkaart_loader
from vaarkaart_service import VaarkaartService
import vaarkaart_service_client

test_client = False # change this to see a test run.

if __name__ == "__main__":
	rospy.init_node('routing_vaarkaart_node')

	print "Loading vaarkaart"
	vaarkaart_graph = vaarkaart_loader.load_vaarkaart()

	print "Starting vaarkaart service"
	VaarkaartService(vaarkaart_graph)

	test_client = rospy.get_param('~test', False)
	if test_client:
		print "Calling vaarkaart service with client"
		vaarkaart_service_client.vaarkaart_routing_service_client(vaarkaart_graph)
		
	rospy.spin()