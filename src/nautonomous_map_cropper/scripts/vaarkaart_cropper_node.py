#!/usr/bin/env python
#!/usr/bin/env python

import sys
import os
# Prevent pyc files being generated!
sys.dont_write_bytecode = True

sys.path.append(os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + '/../lib/vaarkaart'))
sys.path.append(os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + '/../lib/astar'))
sys.path.append(os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + '/../lib/graph'))

from PIL import Image
import rospy

from image_cropper_service import ImageCropperService
from nautonomous_map_msgs.srv import Crop
import vaarkaart_loader

if __name__ == '__main__':
	rospy.init_node('vaarkaart_cropper_node')

	print "Loading vaarkaart"
	vaarkaart_segments = vaarkaart_loader.load_vaarkaart_segments()

	image_cropper_service = ImageCropperService()
	for key, value in vaarkaart_segments.iteritems():
		print value
		image_cropper_service.execute_service(value)

		
	rospy.spin()
