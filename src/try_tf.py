#!/usr/bin/env python

import rospy
from tf import TransformListener

rospy.init_node('arm_to_pos', anonymous=True)
# Initialize the listener (needs some time to subscribe internally to TF and fill its buffer)
tl = TransformListener()
# Our point would look like this
from geometry_msgs.msg import PointStamped
p = PointStamped()
p.header.stamp =rospy.Time.now())
p.header.frame_id = '/base_link'
p.point.x = 1.0
p.point.y = 0.5
p.point.z = 0.0
# Transform the point from base_footprint to map
map_p = tl.transformPoint('/map', p)
print(map_p)
rospy.spin()