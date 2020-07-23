#! /usr/bin/env python
#from ._tf2 import *
import math
import rospy
import array
from nav_msgs.msg import Path, Odometry, OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sympy import symbols, IndexedBase, Idx
import sympy as sp
import numpy as np
import time 
from sympy import *
from opt_ost import ConvexOpt
from model import DiscretizeandLinearizeGeneric
from std_srvs.srv import Empty, EmptyResponse 
from costmap_manager import OccupancyGridManager

# --- Globals ---- 
# Position
init = PoseStamped()		# Initial position
goal = PoseStamped()		# Goal position
pos = PoseStamped()			# Current position

# Mapping
costmap = OccupancyGrid()	# Costmap, the inflated occupancy grid
mapInfo = MapMetaData()		# Useful information about the map (e.g. resolution, width, height)
occupancyThresh = 50		# Value to decide safe zones for the robot in the occupancy grid

# Planning
gScore = []					# The gScore set

# Utilities (e.g. flags, etc)
haveInitial = 0
haveGoal = 0
#define the position of obstacle 
p = []
#-----define the tf functions ----
def euler_to_quaternion( yaw ,roll = 0, pitch = 0  ):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]
#supporting fonction to find the obstacle center
	
# ---- Subscriber Callbacks ----
# Initial pose msg comes from /initialpose topic, which is of PoseStamped() type
def initCallback(msg):
	global init
	global haveInitial
	init.pose = msg.pose.pose
	haveInitial += 1

# Odometry msgs come from /odom topic, which are of the Odometry() type
def odomCallback(msg):
	global pos
	pos.pose = msg.pose.pose
	
# Goal msg comes from /move_base_simple/goal topic, which is of the PoseStamped() type
def goalCallback(msg):
	global goal
	global haveGoal
	goal = msg
	haveGoal += 1

# Costmap comes from /move_base_node/global_costmap/costmap topic, which is of the OccupancyGrid() type
def costmapCallback(msg):
	global costmap
	costmap = msg
	
# Map meta data comes from /map_metadata topic, which is of the MapMetaData() type
def mapInfoCallback(msg):
	global mapInfo
	mapInfo = msg

def service_callback(msg):
    global p
    global haveGoal
    rospy.loginfo("Waiting for initial and goal poses")
    while haveGoal == 0:
        pass  
    # Set rate
    path = Path()
    path = search()
    path.header.frame_id = "map"
    #set the goal and init to zero
    haveGoal = 0
    # Publish the path continuously
    global pathPub
    print(p)
    pathPub.publish(path)
    return EmptyResponse()

def planner():
    # Initialize node
	rospy.init_node('global_planner', anonymous=True)
	# Create publisher
	global pathPub
	pathPub = rospy.Publisher('/path_proxy', Path, queue_size=1)
	#create service
	my_service = rospy.Service('/call_proxy', Empty , service_callback)
	# Create subscribers
	odomSub = rospy.Subscriber('odom', Odometry, odomCallback)
	initSub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, initCallback)
	goalSub = rospy.Subscriber('move_base_simple/goal', PoseStamped, goalCallback)
	cMapSub = rospy.Subscriber('move_base_node/global_costmap/costmap', OccupancyGrid, costmapCallback)
	infoSub = rospy.Subscriber('map_metadata', MapMetaData, mapInfoCallback)
	ogm = OccupancyGridManager('/move_base/global_costmap/costmap',subscribe_to_updates=False)  # default False
	def find_center():
        # Subscribe to the nav_msgs/OccupancyGrid topic
		x,y = ogm.get_costmap_x_y(0, 0)
		coordinate = ogm.get_closest_cell_over_cost(x=x, y=y, cost_threshold=98, max_radius=30)
		area= (ogm.get_area_of_obst(coordinate[0], coordinate[1],98,15,True))
		raggio = int (math.sqrt(area/(3.14)))
		center = ogm.get_center_obst(coordinate[0], coordinate[1],98,radius_obst = raggio*2,radius_tollerance = raggio)
		p = ogm.get_world_x_y(center[0],center[1])
		p = np.array([p[0],p[1]])
		return p
	global p
	p = find_center()
	# Set rate
	r = rospy.spin() # 10 Hz
		


def search():
    global init
    global goal
	#let's define the variables of the class (u inputs and x states)
    u = IndexedBase('u')
    n_in = symbols('n_in ', integer=True)
    u[n_in]
    #you can change the number of input but not the name
    n_in = Idx('n_in', 2)
    x = IndexedBase('x')
    n_states = symbols('n_states', integer=True)
    x[n_states]
    #You can change the number of states not the name
    n_states = Idx('n_states', 3)
    angle_init = quaternion_to_euler(init.pose.orientation.x,init.pose.orientation.y,init.pose.orientation.z,init.pose.orientation.w)
    # steady state conditions
    x_init = [init.pose.position.x,init.pose.position.y,angle_init[0]]
    u_ss = [1,1]
    # final time
    tf = 10 #(seconds)
    #resolution
    k = 1
    # number of time points
    n = tf * k + 1      #total points
    # time points
    dt = tf/n
    t = np.linspace(0,tf,n)

    #define the ode of the system
    Z = [(.16/2)*(u[0]+u[1])*sp.cos((3.14/180)*x[2]),(.16/2)*(u[0]+u[1])*sp.sin((3.14/180)*x[2]),(.16/.55)*(u[0]-u[1])]
    eq = DiscretizeandLinearizeGeneric(Z,np.zeros(x[n_states].shape[0]),np.ones(u[n_in].shape[0]),n)

    # define inputs over time 
    u1= np.ones(n) * u_ss[0]
    u2= np.ones(n) * u_ss[1]
    uw = np.array( [u1,u2])
    angle_goal = quaternion_to_euler(goal.pose.orientation.x,goal.pose.orientation.y,goal.pose.orientation.z,goal.pose.orientation.w)
    #define the goal  position and the condition initial of the velocity
    x_fin = [goal.pose.position.x,goal.pose.position.y,angle_goal[0]*180/3.14]
    u_in = [0,0]
    x_len = len(x_init)
    uante = [[None] * x_len * n]
    xante = [[None] * x_len * n]
    traj_fin = [[None]*x_len ]
    global p
    #iteration to find the optimum result
    for i in range (18):
        #resolution discrete sistem
        x1,x2,x3 = eq.disc(uw,n,dt,x_init)
        Ad_list,Bd_list,Cd_list = eq.get_list()

        #call the Convex optimization class to resolve the problem 
        cvx = ConvexOpt(n,x_init,x_fin,u_in,Ad_list,Bd_list,Cd_list,xante,uante,p)
        #tell to optimize the power 
        opt_pow = True
        #tell to optimize the rapidity of convergence
        opt_conv = True
        xout,uout = cvx.CVXOPT(opt_pow,opt_conv)
        uante = np.copy(uout)
        xante = np.copy(xout)
        uw = uout
        traj_fin = xout
    #plot the true trajectory calculated take into account the estimated u vector with cvx optimization
    x1,x2,x3 = traj_fin
    path = Path()
    for i in range(0,n):
        position = PoseStamped()
        position.pose.position.x = x1[i]
        position.pose.position.y = x2[i]
        quat = euler_to_quaternion(x3[i]*3.14/180)
        position.pose.orientation.x = quat[0]
        position.pose.orientation.y = quat[1]
        position.pose.orientation.z = quat[2]
        position.pose.orientation.w = quat[3]
        position.header.frame_id = '/map'
        path.poses.append(position)
    return path


if __name__ == "__main__":
	try:
		planner()
	except rospy.ROSInterruptException:
		pass
