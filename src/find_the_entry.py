#! /usr/bin/env python
import math

import rospy
from nav_msgs.msg import Odometry
from rospy import Time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
from std_srvs.srv import *
from visualization_msgs.msg import Marker, MarkerArray

# Parameters
hz = 1
entry_found_ = 0
startStop_ = False
dist_left = 0.0
dist_right = 0.0
marker_entries_ = MarkerArray()
position_ = Point()
active_ = False
max_speed = 0.2
inf = 3.6

# Publisher
pub_cmd_vel_ = None
pub_visualization_marker_entries_ = None
pub_entry_ = None

# Sensor regions
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}


def find_the_entry_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res


# callbacks
def clbk_position(msg):
    """
    Callback for the /odom Topic
    Get's the odometry data of the roboter and extracts valuable information
    position_: Current world position of roboter
    """
    global position_

    position_ = msg.pose.pose.position


def clbk_laser(msg):
    """
    Callback for the /scan topic
    Gets Laser Data as Input and calculates the closest distance to an object in a given range
    The ranges are left, front-left, front, front-right, right
    """
    global regions_, inf, dist_left, dist_right
    regions_ = {
        'left': min(min(msg.ranges[72:107]), inf),
        'fleft': min(min(msg.ranges[36:71]), inf),
        'front': min(min(min(msg.ranges[0:35]), min(msg.ranges[324:359])), inf),
        'fright': min(min(msg.ranges[288:323]), inf),
        'right': min(min(msg.ranges[252:287]), inf),
    }

    dist_right = min(min(msg.ranges[16:90]), inf)
    dist_left = min(min(msg.ranges[270:345]), inf)

    take_action()


def clbk_drive(msg):
    """
    Callback for the /start_stop service topic
    Starts/Pauses the roboters activities based on the given Value{True, False}
    """
    global startStop_
    startStop_ = msg.data


def take_action():
    global active_, regions_, entry_found_, position_, pub_entry_
    regions = regions_

    if active_ and entry_found_ == 0:
        if regions['left'] < 1.2 or regions['right'] < 1.2:
            pos_y = int(position_.y.real)

            if 0.49 < dist_left < 1 or 1.49 < dist_left < 2 or 2.49 < dist_left < 3:
                pos_y_left = pos_y + int(math.ceil(dist_left))
            else:
                pos_y_left = pos_y + int(math.floor(dist_left))

            if 0.5 < dist_right < 1 or 1.5 < dist_right < 2 or 2.5 < dist_right < 3:
                pos_y_right = pos_y - int(math.ceil(dist_right))
            else:
                pos_y_right = pos_y - int(math.floor(dist_right))

            entry_markers(int(position_.x.real), pos_y_left)
            entry_markers(int(position_.x.real), pos_y_right)
            entry_found_ = 1
            pub_entry_.publish(True)


def entry_markers(x, y):
    global pub_visualization_marker_entries_

    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = Time()
    marker.ns = "labyrinth_entries"
    marker.text = "Entry Wall"
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 1
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 2
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    marker_entries_.markers.append(marker)

    id = 0
    for marker in marker_entries_.markers:
        marker.id = id
        id += 1

    pub_visualization_marker_entries_.publish(marker_entries_)


def find_entry():
    global max_speed
    msg = Twist()
    msg.linear.x = max_speed
    return msg


def main():
    global hz, active_

    rospy.init_node('find_the_entry')

    init_services()
    init_publisher()
    init_subscribers()

    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():

        msg = Twist()

        if not active_ or not startStop_:
            rate.sleep()
            continue

        msg = find_entry()

        pub_cmd_vel_.publish(msg)

        rate.sleep()


def init_publisher():
    """
    Initializes Publisher:
    pub_cmd_vel_: Publishes to /cmd_vel
    pub_visualization_marker_entries_: Publishes to /visualization_marker_array
    pub_entry_: Publishes to /entry
    """
    global pub_cmd_vel_, pub_visualization_marker_entries_, pub_entry_
    pub_cmd_vel_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    pub_visualization_marker_entries_ = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=2)
    pub_entry_ = rospy.Publisher('/entry', Bool, queue_size=1)


def init_subscribers():
    """
    Initializes Subscribers:
    sub_laser: Subscribes to /scan and executes clbk_laser
    sub_drive: Subscribes to /start_stop and executes clbk_drive
    sub_odom: Subscribes to /odom and executes clbk_position
    """
    sub_scan = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_drive = rospy.Subscriber('/start_stop', Bool, clbk_drive)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_position)


def init_services():
    """
    Initializes service:
    """
    srv = rospy.Service('find_the_entry_switch', SetBool, find_the_entry_switch)


if __name__ == '__main__':
    main()
