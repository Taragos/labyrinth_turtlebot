#!/usr/bin/env python
import math

import rospy
from geometry_msgs.msg import Twist, Point, PoseWithCovariance
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from rospy import Time
from std_srvs.srv import SetBoolResponse, SetBool
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

srv_client_find_the_entry_ = None
srv_client_wall_follower_ = None
startStop_ = False
regions_ = None
state_desc_ = ['Find the entry', 'wall following', 'Go to point']
state_ = 0
pub_visualization_marker_ = None
orientation_ = Point()
position_ = Point()
roll = pitch = yaw = 0.0
x = y = 0.0
yaw_degree = 0.0


# callbacks
# Gets Laser Data as Input and calculates the closest distance to an object in a given range
# The ranges are left, front-left, front, front-right, right
def clbk_laser(msg):
    global regions_
    regions_ = {
        'left': min(min(msg.ranges[72:107]), 10),
        'flight': min(min(msg.ranges[36:71]), 10),
        'front': min(min(min(msg.ranges[0:35]), min(msg.ranges[324:359])), 10),
        'fright': min(min(msg.ranges[288:323]), 10),
        'right': min(min(msg.ranges[252:287]), 10),
    }
    rospy.loginfo(regions_)


def clbk_drive(msg):
    global startStop_
    startStop_ = msg.data


def change_state(state):
    global state_, state_desc_
    global srv_client_find_the_entry_, srv_client_wall_follower_
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    # rospy.loginfo(log)
    if state_ == 0:
        resp = srv_client_find_the_entry_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_find_the_entry_(False)
        resp = srv_client_wall_follower_(True)


def position(msg):
    global orientation_, position_, roll, pitch, yaw, yaw_degree  # position_marker,
    position_ = msg.pose.pose.position
    orientation_ = msg.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([orientation_.x, orientation_.y, orientation_.z, orientation_.w])
    yaw_degree = (yaw * (180 / math.pi))

def coordinator():
    global srv_client_find_the_entry_, srv_client_wall_follower_, pub_visualization_marker_
    global regions_, state_, startStop_

    rospy.init_node('coordinator')

    pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    rospy.wait_for_service('/find_the_entry_switch')
    rospy.wait_for_service('/wall_follower_switch')
    rospy.wait_for_service('/start_stop')

    sub_drive = rospy.Subscriber('/start_stop', Bool, clbk_drive)
    srv_client_find_the_entry_ = rospy.ServiceProxy('/find_the_entry_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)

    sub_odom = rospy.Subscriber('/odom', Odometry, position)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if regions_ is None:
            continue

        if not startStop_:
            rate.sleep()
            continue
        else:
            change_state(0)

        if state_ == 0:
            if 0.15 < regions_['left'] < 1.2 or 0.15 < regions_['right'] < 1.2:
                change_state(1)

        rate.sleep()



if __name__ == '__main__':
    try:
        coordinator()
    except rospy.ROSInterruptException:
        pass
