#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker
from rospy import Time

pub_visualization_marker_ = None
map_origin_ = Point()
resolution_ = 0.5
height_ = 0.0
width_ = 0.0
position_ = Point()
map_data_ = []

# callbacks
def clbk_odom(msg):
    global position_

    # position
    position_ = msg.pose.pose.position

def clbk_map(msg):
    global map_origin_, resolution_, height_, width_, map_data_
    map_origin_ = Point(
        msg.info.origin.position.x, 
        msg.info.origin.position.y, 
        0)
    resolution_ = msg.info.resolution
    height_ = msg.info.height
    width_ = msg.info.width
    map_data_ = msg.data
    x_i, y_i = get_closest_square(position_.x, position_.y)
    rospy.loginfo("X_I: " + str(x_i) + ", Y_I: " + str(y_i))
    s_x_i, s_y_i = get_closest_occupied_square(x_i, y_i)
    rospy.loginfo("Occupied: X_I: " + str(s_x_i) + ", Y_I: " + str(s_y_i))
    x = s_x_i * resolution_ + map_origin_.x        
    y = s_y_i * resolution_ + map_origin_.y        
    rospy.loginfo("X: " + str(x) + ", Y: " + str(y))
    counter = 0
    for i in map_data_:
        if i is 100:
            counter = counter + 1
    
    rospy.loginfo("Counter: " + str(counter))

    marker_closest_square(x, y)
    

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
            rospy.loginfo("Is Occupied")
            return x_i, y_i
        # 1 + 2 * level right
        goal =  y_i + (1 + 2 * level)
        while y_i < goal:
            y_i = y_i + 1
            rospy.loginfo("Right Move: X_I: " + str(x_i) + ", Y_I: " + str(y_i))
            if is_occupied(x_i, y_i):
                rospy.loginfo("Is Occupied")
                return x_i, y_i
        # 1 + 2 * level down
        goal =  x_i + (2 + 2 * level)
        while x_i < goal :
            x_i = x_i + 1
            rospy.loginfo("Down Move: X_I: " + str(x_i) + ", Y_I: " + str(y_i))
            if is_occupied(x_i, y_i):
                rospy.loginfo("Is Occupied")
                return x_i, y_i
        # 1 + 2 * level left
        goal =  y_i - (2 + 2 * level)
        while y_i > goal:
            y_i = y_i - 1
            rospy.loginfo("Left Move: X_I: " + str(x_i) + ", Y_I: " + str(y_i))
            if is_occupied(x_i, y_i):
                rospy.loginfo("Is Occupied")
                return x_i, y_i
        # 1 + 2 * level up
        goal = x_i - (2 + 2 * level)
        while x_i > goal:
            x_i = x_i - 1
            rospy.loginfo("Up Move: X_I: " + str(x_i) + ", Y_I: " + str(y_i))
            if is_occupied(x_i, y_i):
                rospy.loginfo("Is Occupied")
                return x_i, y_i
        level = level + 1
    return 1, 1

def is_occupied(x_i, y_i):
    i = int(((y_i - 1) * width_) - (width_ - (x_i - 1)))   # calculate index of square in data array
    rospy.loginfo("Index: " + str(i))
    if i > 0 and i < len(map_data_):        # check if calculated index is still in array, square is within map bounds
        return map_data_[i] == 100          # return occupancy
    else:
        return False                        # return False, so it doesn't get used

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
    global pub_visualization_marker_

    rospy.init_node('map_handler')

    pub_visualization_marker_ = rospy.Publisher("/visualization_marker", Marker, queue_size=1)

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
