#! /usr/bin/env python
import math
import rospy
import numpy as np
from costmap_manager import OccupancyGridManager
from nav_msgs.msg import Path, Odometry, OccupancyGrid, MapMetaData
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

rospy.init_node('listener_costmap', anonymous=True)
# Subscribe to the nav_msgs/OccupancyGrid topic
ogm = OccupancyGridManager('/move_base/global_costmap/costmap',subscribe_to_updates=False)  # default False
# You can get the cost from world coordinates (in the frame of the OccupancyGrid)
print("You can get the cost from world coordinates (in the frame of the OccupancyGrid)")
print(ogm.get_cost_from_world_x_y(-1.196, 0.935))
print(ogm.get_costmap_x_y(-10  , 10.276))
# You can find the closest cell with a cost over a value (to find an occupied cell for example)
x,y = ogm.get_costmap_x_y(-4,-3)
print(x,y)
coordinate = ogm.get_closest_cell_over_cost(x=x, y=y, cost_threshold=98, max_radius=50)
x_ost,y_ost = ogm.get_world_x_y(coordinate[0],coordinate[1])
print(coordinate)
print(x_ost,y_ost,coordinate[2])
area= (ogm.get_area_of_obst(coordinate[0], coordinate[1],98,20,True))
raggio = int (math.sqrt(area/(3.14)))
print(raggio)
center = ogm.get_center_obst(coordinate[0], coordinate[1],98,radius_obst = raggio*2,radius_tollerance = raggio)
print(ogm.get_world_x_y(center[0],center[1]))
print(center)

def find_center (x_i = 0 , y_i = 0):
    x,y = ogm.get_costmap_x_y(x_i,y_i)
    coordinate = ogm.get_closest_cell_over_cost(x=x, y=y, cost_threshold=98, max_radius=10)
    if(coordinate[2] == -1):
        return [-10e6,-10e6]
    else:
        x_ost,y_ost = ogm.get_world_x_y(coordinate[0],coordinate[1])
        area= (ogm.get_area_of_obst(coordinate[0], coordinate[1],98,20,True))
        raggio = int (math.sqrt(area/(3.14)))
        center = ogm.get_center_obst(coordinate[0], coordinate[1],98,radius_obst = raggio*2,radius_tollerance = raggio)
        p = ogm.get_world_x_y(center[0],center[1])
        return [p[0],p[1]]

def find_obst(size):
    p = []
    for i in range(-size, size+1):
        for j in range (-size , size+1):
            print(i,j)
            h = find_center(i,j)
            if(h not in p):
                p.append([h[0],h[1]])
    c = len(p)
    i = 0
    while i < c :
        if(p[i][0]>= size+0.5 or p[i][0] <= -size-0.5 or p[i][1] >= size+0.5 or p[i][1] <= -size-0.5):
            p.pop(i)
            c -= 1
            i -= 1
        i += 1
    return p 

p = find_obst(6)
print(p)




