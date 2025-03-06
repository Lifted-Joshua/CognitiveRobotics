#!/usr/bin/env python3

import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist
from gridviz import GridViz

from algorithms.neighbors import find_neighbors


def manhattan_distance(index, goal_index, width):
    """Heuristic Function for A* algorithm using Manhattan Distance"""
    index_x = index % width
    index_y = index // width
    goal_x = goal_index % width
    goal_y = goal_index // width

    distance = abs(index_x - goal_x) + abs(index_y - goal_y)
    return distance


def a_star(start_index, goal_index, width, height, costmap, resolution, origin, grid_viz):
  '''
    Performs a_star's shortes path algorithm search on a costmap with a given start and goal node
    '''
  # create an open_list
  open_list = []

  # set to hold already processed nodes
  closed_list = set()

  # dict for mapping children to parent
  parents = dict()

  # dict for mapping g costs (travel costs) to nodes
  g_costs = dict()

  # dict for mapping f costs (heuristic + travel) to nodes
  f_costs = dict()

  # set the start's node g_cost and f_cost
  g_costs[start_index] = 0
  f_costs[start_index] = 0

  # add start node to open list
  start_cost = 0 + manhattan_distance(start_index, goal_index, width)
  open_list.append([start_index, start_cost])

  shortest_path = []

  path_found = False
  rospy.loginfo('A Star: Done with initialization')

  # Main loop, executes as long as there are still nodes inside open_list
  while open_list:

    # sort open_list according to the lowest 'g_cost' value (second element of each sublist)
    open_list.sort(key=lambda x: x[1])
    # extract the first element (the one with the lowest 'g_cost' value)
    current_node = open_list.pop(0)[0]

    # Close current_node to prevent from visting it again
    closed_list.add(current_node)

    # Optional: visualize closed nodes
    grid_viz.set_color(current_node, "pale yellow")

    # If current_node is the goal, exit the main loop
    if current_node == goal_index:
      path_found = True
      break

    # Get neighbors of current_node
    neighbors = find_neighbors(current_node, width, height, costmap, resolution)

    # Loop neighbors
    for neighbor_index, step_cost in neighbors:

      # Check if the neighbor has already been visited
      if neighbor_index in closed_list:
        continue

      # calculate g_cost of neighbour considering it is reached through current_node
      g_cost = g_costs[current_node] + step_cost
      h_cost = manhattan_distance(neighbor_index, goal_index, width)
      f_cost = g_cost + h_cost

      # Check if the neighbor is in open_list
      in_open_list = False
      for idx, element in enumerate(open_list):
        if element[0] == neighbor_index:
          in_open_list = True
          break

      # CASE 1: neighbor already in open_list
      if in_open_list:
        if f_cost < f_costs[neighbor_index]:
          # Update the node's g_cost and f_cost
          g_costs[neighbor_index] = g_cost
          f_costs[neighbor_index] = f_cost
          parents[neighbor_index] = current_node
          # Update the node's g_cost inside open_list
          open_list[idx] = [neighbor_index, f_cost]

      # CASE 2: neighbor not in open_list
      else:
        # Set the node's g_cost and f_cost
        g_costs[neighbor_index] = g_cost
        f_costs[neighbor_index] = f_cost
        parents[neighbor_index] = current_node
        # Add neighbor to open_list
        open_list.append([neighbor_index, f_cost])

        # Optional: visualize frontier
        grid_viz.set_color(neighbor_index, 'orange')

  rospy.loginfo('A_Star: Done traversing nodes in open_list')

  if not path_found:
    rospy.logwarn('A_Star: No path found!')
    return shortest_path

  # Reconstruct path by working backwards from target
  if path_found:
    node = goal_index
    shortest_path.append(goal_index)
    while node != start_index:
      shortest_path.append(node)
      # get next node
      node = parents[node]
  # reverse list
  shortest_path = shortest_path[::-1]
  rospy.loginfo('A_Star: Done reconstructing path')

  return shortest_path


def dijkstra(start_index, goal_index, width, height, costmap, resolution, origin, grid_viz):
  ''' 
  Performs Dijkstra's shortes path algorithm search on a costmap with a given start and goal node
  '''
  # create an open_list
  open_list = []

  # set to hold already processed nodes
  closed_list = set()

  # dict for mapping children to parent
  parents = dict()

  # dict for mapping g costs (travel costs) to nodes
  g_costs = dict()

  # set the start's node g_cost
  g_costs[start_index] = 0

  # add start node to open list
  open_list.append([start_index, 0])

  shortest_path = []

  path_found = False
  rospy.loginfo('Dijkstra: Done with initialization')

  # Main loop, executes as long as there are still nodes inside open_list
  while open_list:

    # sort open_list according to the lowest 'g_cost' value (second element of each sublist)
    open_list.sort(key = lambda x: x[1]) 
    # extract the first element (the one with the lowest 'g_cost' value)
    current_node = open_list.pop(0)[0]

    # Close current_node to prevent from visting it again
    closed_list.add(current_node)

    # Optional: visualize closed nodes
    grid_viz.set_color(current_node,"pale yellow")

    # If current_node is the goal, exit the main loop
    if current_node == goal_index:
      path_found = True
      break

    # Get neighbors of current_node
    neighbors = find_neighbors(current_node, width, height, costmap, resolution)

    # Loop neighbors
    for neighbor_index, step_cost in neighbors:

      # Check if the neighbor has already been visited
      if neighbor_index in closed_list:
        continue

      # calculate g_cost of neighbour considering it is reached through current_node
      g_cost = g_costs[current_node] + step_cost

      # Check if the neighbor is in open_list
      in_open_list = False
      for idx, element in enumerate(open_list):
        if element[0] == neighbor_index:
          in_open_list = True
          break

      # CASE 1: neighbor already in open_list
      if in_open_list:
        if g_cost < g_costs[neighbor_index]:
          # Update the node's g_cost inside g_costs
          g_costs[neighbor_index] = g_cost
          parents[neighbor_index] = current_node
          # Update the node's g_cost inside open_list
          open_list[idx] = [neighbor_index, g_cost]

      # CASE 2: neighbor not in open_list
      else:
        # Set the node's g_cost inside g_costs
        g_costs[neighbor_index] = g_cost
        parents[neighbor_index] = current_node
        # Add neighbor to open_list
        open_list.append([neighbor_index, g_cost])

        # Optional: visualize frontier
        grid_viz.set_color(neighbor_index,'orange')

  rospy.loginfo('Dijkstra: Done traversing nodes in open_list')

  if not path_found:
    rospy.logwarn('Dijkstra: No path found!')
    return shortest_path

  # Reconstruct path by working backwards from target
  if path_found:
      node = goal_index
      shortest_path.append(goal_index)
      while node != start_index:
          shortest_path.append(node)
          # get next node
          node = parents[node]
  # reverse list
  shortest_path = shortest_path[::-1]
  rospy.loginfo('Dijkstra: Done reconstructing path')

  return shortest_path


def make_plan(req):
    '''
  Callback function used by the service server to process
  requests from clients. It returns a msg of type PathPlanningPluginResponse
  '''
    # costmap as 1-D array representation
    costmap = req.costmap_ros
    # number of columns in the occupancy grid
    width = req.width
    # number of rows in the occupancy grid
    height = req.height
    start_index = req.start
    goal_index = req.goal
    # side of each grid map square in meters
    resolution = 0.05
    # origin of grid map
    origin = rospy.get_param("/map_origin", [0.0, 0.0, 0.0])  # Load from ROS param

    viz = GridViz(costmap, resolution, origin, start_index, goal_index, width)

    # time statistics
    start_time = rospy.Time.now()

    # calculate the shortest path using Dijkstra
    path = a_star(start_index, goal_index, width, height, costmap, resolution, origin, viz)

    if not path:
        rospy.logwarn("No path returned by algorithm")
        path = []
    else:
        execution_time = rospy.Time.now() - start_time
        print("\n")
        rospy.loginfo('Total execution time: %s seconds', str(execution_time.to_sec()))
        rospy.loginfo('++++++++++++++++++++++++++++++++++++++++++++')
        print("\n")
        rospy.loginfo('Path sent to navigation stack')

    resp = PathPlanningPluginResponse()
    resp.plan = path
    return resp

def clean_shutdown():
    cmd_vel.publish(Twist())
    rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('path_planning_service_server', log_level=rospy.INFO, anonymous=False)
    make_plan_service = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    rospy.on_shutdown(clean_shutdown)

    rospy.spin()  # Keeps the node alive