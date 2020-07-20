#! /usr/bin/env python
import math
import rospy
import numpy as np
from costmap_manager import OccupancyGridManager
from nav_msgs.msg import Path, Odometry, OccupancyGrid, MapMetaData
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

# --- Globals ---- 

# Mapping
costmap = OccupancyGrid()	# Costmap, the inflated occupancy grid
mapInfo = MapMetaData()		# Useful information about the map (e.g. resolution, width, height)
occupancyThresh = 50		# Value to decide safe zones for the robot in the occupancy grid
width = 0
height = 0 

def costmapCallback(msg):
	global costmap
	global width
	global height
	costmap = msg
	height = costmap.info.height  
	width = costmap.info.width 
	origin_x = costmap.info.origin.position.x	
	origin_y = costmap.info.origin.position.y
	print(height)
	print(width)
	print(origin_x)
	print(origin_y)
	count = 0
	save_i = 10000
	for i in range (len(costmap.data)):
		if(costmap.data[i] > 0):
			#if(count == 0):
				#save_i = i                 
			count +=1
            
	print(count)
	#print(save_i)
	print(costmap.data[save_i:save_i+1000])
def up_costmapCallback(msg):
	global costmap
	index = 0
	for y in range(msg.y,msg.y+msg.height):
		for x in range(msg.x,msg.x+msg.width):
			costmap.data[getIndex(x,y)] = msg.data[index]
			index+=1
	print(costmap.data[:1000])  

def planner():
# Initialize node
	rospy.init_node('listener_costmap', anonymous=True)
	cMapSub = rospy.Subscriber('move_base/global_costmap/costmap', OccupancyGrid, costmapCallback)
	#cMapSub = rospy.Subscriber('move_base_node/global_costmap/costmap_update', OccupancyGridUpdate, up_costmapCallback)
	r = rospy.spin() # 10 Hz


if __name__ == "__main__":
	try:
		planner()
	except rospy.ROSInterruptException:
		pass