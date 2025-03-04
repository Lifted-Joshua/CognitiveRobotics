#!/usr/bin/env python3

import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist
from gridviz import GridViz

def make_plan(req):
  ''' 
  Callback function used by the service server to process
  requests from clients. It returns a msg of type PathPlanningPluginResponse
  ''' 
  costmap = req.costmap_ros
  # number of columns in the occupancy grid
  width = req.width
  # number of rows in the occupancy grid
  height = req.height
  start = req.start
  goal = req.goal
  # side of each grid map square in meters
  resolution = 0.05
  # origin of grid map
  origin = [] #hint: find this in your YAML map file

  grid_visualisation = GridViz(costmap, resolution, origin, start, goal, width)

  # time statistics
  start_time = rospy.Time.now()

  # calculate the shortest path

  """
  Your code continues here. Toggle between a_star and dijkstra
  path = a_star(start, goal, width, height, costmap, resolution, origin, grid_visualisation)
  path = dijkstra(start, goal, width, height, costmap, resolution, origin, grid_visualisation)
  """

  if not path:
    rospy.logwarn("No path returned by the path algorithm")
    path = []
  else:
    # additional code here as per your implementation, e.g., displaying your performance metrics
    rospy.loginfo('Path sent to navigation stack')

  resp = PathPlanningPluginResponse()
  resp.plan = path
  return resp

def clean_shutdown():
  cmd_vel.publish(Twist())
  rospy.sleep(1)

if _name_ == '_main_':
  rospy.init_node('path_planning_server', log_level=rospy.INFO, anonymous=False)
  make_plan_service = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)
  cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
  rospy.on_shutdown(clean_shutdown)

  while not rospy.core.is_shutdown():
    rospy.rostime.wallsleep(0.5)
  rospy.Timer(rospy.Duration(2), rospy.signal_shutdown('Shutting down'), oneshot=True)
  
