#!/usr/bin/env python

import rospy
from tf import TransformListener

rospy.init_node('arm_to_pos', anonymous=True)
# Initialize the listener (needs some time to subscribe internally to TF and fill its buffer)
tl = TransformListener()
# Our point would look like this
from geometry_msgs.msg import PointStamped
while not rospy.is_shutdown():
    rospy.sleep(1) #must pass a time between the inizialization and the p time for having a tf correctly initialized
    p = PointStamped()
    p.header.stamp =rospy.Time.now()
    rospy.sleep(1) #for security leave it, it reach less error
    p.header.frame_id = '/base_footprint'
    p.point.x = 1.0
    p.point.y = 0.5
    p.point.z = 0.0
    # Transform the point from base_footprint to map
    map_p = tl.transformPoint('map', p)
    print(map_p)