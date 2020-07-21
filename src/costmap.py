#! /usr/bin/env python
import math
import rospy
import numpy as np
from costmap_manager import OccupancyGridManager
from nav_msgs.msg import Path, Odometry, OccupancyGrid, MapMetaData
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf import TransformListener

rospy.init_node('listener_costmap', anonymous=True)
# Subscribe to the nav_msgs/OccupancyGrid topic
ogm = OccupancyGridManager('/move_base/global_costmap/costmap',
                            subscribe_to_updates=False)  # default False

# You can get the cost from world coordinates (in the frame of the OccupancyGrid)
print("You can get the cost from world coordinates (in the frame of the OccupancyGrid)")
print(ogm.get_cost_from_world_x_y(-1.196, 0.935))
print(ogm.get_costmap_x_y(-1.196, 0.935))
# You can find the closest cell with a cost over a value (to find an occupied cell for example)
x,y = ogm.get_costmap_x_y(0, 0)
coordinate = ogm.get_closest_cell_over_cost(x=x, y=y, cost_threshold=98, max_radius=30)
x_ost,y_ost = ogm.get_world_x_y(coordinate[0],coordinate[1])
print(coordinate)
print(x_ost,y_ost,coordinate[2])
area= (ogm.get_area_of_obst(coordinate[0], coordinate[1],98,15,True))
raggio = int (math.sqrt(area/(3.14)))
print(raggio)
center = ogm.get_center_obst(coordinate[0], coordinate[1],98,radius_obst = raggio*2,radius_tollerance = raggio)
print(ogm.get_world_x_y(center[0],center[1]))
print(center)