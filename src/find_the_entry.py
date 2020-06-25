#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from rospy import Time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
from std_srvs.srv import *
from visualization_msgs.msg import Marker

pub_cmd_vel_ = None
pub_visualization_marker_ = None
pub_entry_ = None

regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
entry_found_ = 0
startStop_ = False
position_ = Point()
active_ = False


def find_the_entry_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res


# callbacks
def clbk_odom(msg):
    global position_

    # position
    position_ = msg.pose.pose.position


def clbk_laser(msg):
    global regions_
    regions_ = {
        'right': min(min(msg.ranges[72:107]), 10),
        'fright': min(min(msg.ranges[36:71]), 10),
        'front': min(min(min(msg.ranges[0:35]), min(msg.ranges[324:359])), 10),
        'fleft': min(min(msg.ranges[288:323]), 10),
        'left': min(min(msg.ranges[252:287]), 10),
    }

    take_action()


def clbk_drive(msg):
    global startStop_
    startStop_ = msg.data


def take_action():
    global active_, regions_, entry_found_, position_, pub_entry_
    regions = regions_

    if active_ and entry_found_ == 0:
        if regions['left'] < 0.2 and regions['right'] < 0.2:
            entry_found_ = 1
            pub_entry_.publish(True)
            marker(int(position_.x.real), int(position_.y.real))


def find_entry():
    msg = Twist()
    msg.linear.x = 0.2
    return msg


def marker(x, y):
    global pub_visualization_marker_

    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = Time()
    marker.ns = "labyrinth_turtlebot"
    marker.id = 0
    marker.text = "Entry"
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


def main():
    global pub_cmd_vel_, pub_visualization_marker_, pub_entry_
    global active_

    rospy.init_node('find_the_entry')

    srv = rospy.Service('find_the_entry_switch', SetBool, find_the_entry_switch)

    pub_cmd_vel_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    pub_visualization_marker_ = rospy.Publisher("/visualization_marker", Marker, queue_size=1)
    pub_entry_ = rospy.Publisher('/entry', Bool, queue_size=1)

    sub_scan = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    sub_drive = rospy.Subscriber('/start_stop', Bool, clbk_drive)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():

        msg = Twist()

        if not active_:
            rate.sleep()
            continue

        if not startStop_:
            msg.linear.x = 0
            pub_cmd_vel_.publish(msg)
            rate.sleep()
            continue

        msg = find_entry()

        pub_cmd_vel_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()