# obst_planner
This proxy algo is the planner version with one obstacle avoidance of convex optimization trajectory finder.
This is based only on the costmap, it calculate directly the trajectory by knowing that the map is complitly empty and there is only a circular obstacle that must be avoid.
The shape of the obstacle is important beacuse it must be a convex shape, we can use any shape but the algo will be consider the shape as circular.
The radius is found by the area, it will calculate the number of costmap cell that are up to 98 threshold and then calculate the radius considering that we have a circular shape.
After with an algo it will calculate the center of the circle and then it will pass the center to our convex opt class in order to calculate the trajectory.
For security to the class of cvx we will pass a radius that is 1.5 the radius calculate previusly in order to be sure that the obstacle will be avoid.

#how to use 
run a gazebo simulation with a world similar to my_world.world( a playpen without all the inside obstacle and only with a cone)
```
roslaunch husky_gazebo husky_playpen.world 
```
run the proxy 
```
rosrun obst_planner planner.py
```
run the amcl and move base node in order to start the navigation 
```
roslaunch husky_navigation amcl_demo.launch
```
wait that the costmap of the proxy is initialized and then provide with rviz a initial pose and a goal 
```
rosrun rviz rviz
```
