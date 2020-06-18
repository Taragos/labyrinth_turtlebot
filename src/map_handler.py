#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, Header
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray
from rospy import Time
import math

pub_visualization_marker_ = pub_path_ = None
map_origin_ = position_ = Point()
resolution_ = 0.5  # resolution in m per square -> 0.05 ~ one square = 5cm
height_ = width_ = 0.0
map_data_ = []
markers_ = []
a_star_vis = MarkerArray()
turtlebot_radius_ = 11 # radius of turtlebot = 105mm in cm ~ 11cm
last_wall_ = 0
check_positions_ = []
last_wall_direction = 0

class Node():
    """
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

def setup_wall_distance():
    global check_positions_
    min_dist_in_squares = int(math.ceil(turtlebot_radius_ / (resolution_ * 100))) 
    # Rechts                                                                 # ----------------------------------------------> y
    for i in range(1, min_dist_in_squares):                                  # |                          (-1,0)
        check_positions_.append((0, i))                                      # |                             A
    # Runter                                                                 # |                             |
    for i in range(1, min_dist_in_squares):                                  # |                             |
        check_positions_.append((i, 0))                                      # |              (0, -1) <----- R ----> (0, 1)       
    # Links                                                                  # |                             |
    for i in range(1, min_dist_in_squares):                                  # |                             |
        check_positions_.append((0, -i))                                     # V                             V
    # Hoch                                                                   # x                          (1, 0)
    for i in range(1, min_dist_in_squares):
        check_positions_.append((-i, 0))

# checks if there is a wall somewhere close to the current node, that the turtlebot might bump into
def check_for_wall(current_position, maze):
    global last_wall_direction
    rospy.loginfo(check_positions_)
    run = True
    start_index = last_wall_direction
    while run:
        new_position = check_positions_[last_wall_direction]        
        node_position = (current_position[0] + new_position[0], current_position[1] + new_position[1])
        
        if node_position[0] < 0 or node_position[1] < 0:
            return False

        index = get_index(node_position[0], node_position[1])

        if index < 0 or index >= len(maze):
            return True

            # Make sure walkable terrain
        if maze[index] == 100:
            # add_red_marker(node_position[0], node_position[1], index)
            return True

        last_wall_direction = (last_wall_direction + 1) % len(check_positions_) 
        if start_index is last_wall_direction:
            run = False        
    return False

def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""
    setup_wall_distance()
    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)
        rospy.loginfo("Current Node Pos: %s " % (current_node.position, ))

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            
            if node_position[0] < 0 or node_position[1] < 0:
                continue

            rospy.loginfo("Child Node Pos: %s " % (node_position, ))
            # Make sure within range
            index = get_index(node_position[0], node_position[1])
            rospy.loginfo("Child Index: %s " % str(index))
            if index < 0 or index >= len(maze):
                continue

            # Make sure walkable terrain
            if maze[index] == 100:
                # add_red_marker(node_position[0], node_position[1], index)
                continue

            if check_for_wall(node_position, maze):
                continue
            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)
            # add_green_marker(node_position[0], node_position[1], index)

        rospy.loginfo("Children: %s" % children)
        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)

# callbacks
def clbk_odom(msg):
    global position_

    # position
    position_ = msg.pose.pose.position

def clbk_map(msg):
    global map_origin_, resolution_, height_, width_, map_data_

    # Sets Global Variables
    map_origin_ = Point(
        msg.info.origin.position.x, 
        msg.info.origin.position.y, 
        0)
    resolution_ = msg.info.resolution
    height_ = msg.info.height
    width_ = msg.info.width
    map_data_ = msg.data

    # Get Square closest to TurtleBot Position
    x_i, y_i = get_closest_square(position_.x, position_.y)
    rospy.loginfo("X_I: " + str(x_i) + ", Y_I: " + str(y_i))
    path = astar(map_data_, (x_i, y_i), (0,0))
    rospy.loginfo(path)
    publish_path(path)
    # Get the Square Indices that are closest to the TurtleBot Square and is occupied

    # marker_closest_square(s_x, s_y)
    
def publish_path(data):
    path = MarkerArray()
    counter = 0
    
    for tuple in data:
        x, y = map_indices_to_position(tuple[0],tuple[1])

        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = Time()
        marker.ns = "labyrinth_turtlebot"
        marker.id = counter
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x + (resolution_ / 2)
        marker.pose.position.y = y + (resolution_ / 2)
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = resolution_
        marker.scale.y = resolution_
        marker.scale.z = 0.01
        marker.color.a = 0.5
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        counter = counter + 1
        path.markers.append(marker)
    
    pub_path_.publish(path)

def map_indices_to_position(x_i, y_i):
    """Converts the given map indices to the real world x and y positions 

    
    Parameters:  \n
    x_i (int): X Index of Square\n
    y_i (int): Y Index of Square\n

    Returns: \n
    x (int): X Position in real world\n
    y (int): Y Position in real world
    """
    x = x_i * resolution_ + map_origin_.x        
    y = y_i * resolution_ + map_origin_.y
    return x, y

def get_closest_occupied_square(x_i, y_i):
    global map_origin_, resolution_, height_, width_
    level = 0
    start_x, start_y = x_i, y_i
    # x = start_x * resolution_ + map_origin_.x        
    # y = start_y * resolution_ + map_origin_.y    
    # marker_closest_square(x, y)

    while (start_x + level) < width_ * 2 and (start_y + level) < height_ * 2:
        # 1 Up
        x_i = x_i - 1
        rospy.loginfo("Up Move: X_I: " + str(x_i) + ", Y_I: " + str(y_i))
        if is_occupied(x_i, y_i):
            return x_i, y_i
        # 1 + 2 * level right
        goal =  y_i + (1 + 2 * level)
        while y_i < goal:
            y_i = y_i + 1
            rospy.loginfo("Right Move: X_I: " + str(x_i) + ", Y_I: " + str(y_i))
            if is_occupied(x_i, y_i):
                return x_i, y_i
        # 1 + 2 * level down
        goal =  x_i + (2 + 2 * level)
        while x_i < goal :
            x_i = x_i + 1
            rospy.loginfo("Down Move: X_I: " + str(x_i) + ", Y_I: " + str(y_i))
            if is_occupied(x_i, y_i):
                return x_i, y_i
        # 1 + 2 * level left
        goal =  y_i - (2 + 2 * level)
        while y_i > goal:
            y_i = y_i - 1
            rospy.loginfo("Left Move: X_I: " + str(x_i) + ", Y_I: " + str(y_i))
            if is_occupied(x_i, y_i):
                return x_i, y_i
        # 1 + 2 * level up
        goal = x_i - (2 + 2 * level)
        while x_i > goal:
            x_i = x_i - 1
            rospy.loginfo("Up Move: X_I: " + str(x_i) + ", Y_I: " + str(y_i))
            if is_occupied(x_i, y_i):
                return x_i, y_i
        level = level + 1
    return 1, 1

def is_occupied(x_i, y_i):
    i = get_index(x_i, y_i)
    rospy.loginfo("Index: " + str(i))
    if i > 0 and i < len(map_data_):        # check if calculated index is still in array, square is within map bounds
        return map_data_[i] == 100          # return occupancy
    else:
        return False                        # return False, so it doesn't get used

def get_index(x_i, y_i):
    return int(y_i * width_ + x_i)   # calculate index of square in data array

def get_closest_square(x, y):
    x_delta = x - map_origin_.x
    y_delta = y - map_origin_.y
    x_i = x_delta//resolution_
    y_i = y_delta//resolution_
    return x_i, y_i

def marker_map_origin(x, y):
    global pub_visualization_marker_

    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = Time()
    marker.ns = "labyrinth_turtlebot"
    marker.id = 0
    marker.text = "Square"
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 1
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 2
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    pub_visualization_marker_.publish(marker)

def marker_closest_square(x, y):
    global pub_visualization_marker_

    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = Time()
    marker.ns = "labyrinth_turtlebot"
    marker.id = 0
    marker.text = "Map Origin"
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 1
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 2
    marker.color.a = 0.3
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    pub_visualization_marker_.publish(marker)

def coordinator():
    global pub_visualization_marker_, pub_path_

    rospy.init_node('map_handler')

    pub_visualization_marker_ = rospy.Publisher("/visualization_marker", Marker, queue_size=1)
    pub_path_ = rospy.Publisher("/a_path", MarkerArray, queue_size=10)
    sub_map = rospy.Subscriber('/map', OccupancyGrid, clbk_map)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        coordinator()
    except rospy.ROSInterruptException:
        pass
