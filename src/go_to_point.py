#! /usr/bin/env python

# import ros stuff
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf import transformations
from std_srvs.srv import *

import math

# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 2
# goal
desired_position_ = Point()
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 70  # +/- 2 degree allowed
dist_precision_ = 0.01
active_ = False
startStop_ = False
corner_count_ = 0

# publishers
pub_cmd_ = None


# service callbacks
def go_to_point_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res


def clbk_drive(msg):
    """
    Callback for the /start_stop service topic
    Starts/Pauses the roboters activities based on the given Value{True, False}
    """
    global startStop_
    startStop_ = msg.data


# callbacks
def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def clbk_path_change(req):
    global corner_count_
    if req.data:
        corner_count_ = 0


def change_state(state):
    global state_
    state_ = state
    rospy.loginfo('State changed to [%s]' % state_)


def normalize_angle(angle):
    if math.fabs(angle) > math.pi:
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def fix_yaw(des_pos):
    global yaw_, pub_cmd_, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)

    rospy.loginfo(err_yaw)

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7

    pub_cmd_.publish(twist_msg)

    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        rospy.logerr('Yaw error: [%s]' % err_yaw)
        change_state(1)


def go_straight_ahead(des_pos):
    global yaw_, pub_cmd_, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.1
        twist_msg.angular.z = 0.2 if err_yaw > 0 else -0.2
        pub_cmd_.publish(twist_msg)
    else:
        rospy.logerr('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        rospy.logerr('Yaw error in ahead: [%s]' % err_yaw)
        change_state(0)


def set_destination():
    global desired_position_, corner_count_
    # Get destination coordinates
    if rospy.has_param('/path_corner_points'):
        path_corners = rospy.get_param("/path_corner_points")
        # if final destination reached stop the robot
    if corner_count_ >= len(path_corners):
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        pub_cmd_.publish(twist_msg)
        corner_count_ = 0
    # set new destination
    else:
        desired_position_.x = path_corners[corner_count_]["x"]
        desired_position_.y = path_corners[corner_count_]["y"]
        corner_count_ = corner_count_ + 1
        change_state(0)



def main():
    global active_

    rospy.init_node('go_to_point')

    init_services()
    init_publisher()
    init_subscribers()

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_ or not startStop_:
            rate.sleep()
            continue
        else:
            if state_ == 0:
                fix_yaw(desired_position_)
            elif state_ == 1:
                go_straight_ahead(desired_position_)
            elif state_ == 2:
                set_destination()
            else:
                rospy.logerr('Unknown state!')

        rate.sleep()


def init_publisher():
    """
    Initializes Publisher:
    pub_cmd_: Publishes to /cmd_vel
    """
    global pub_cmd_
    pub_cmd_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


def init_subscribers():
    """
    Initializes Subscribers:
    sub_drive: Subscribes to /start_stop and executes clbk_drive
    sub_odom: Subscribes to /odom and executes clbk_position
    sub_path_change: Subscribes to /path_change and executes clbk_path_change
    """
    sub_drive = rospy.Subscriber('/start_stop', Bool, clbk_drive)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    sub_path_change = rospy.Subscriber('/path_change', Bool, clbk_path_change)


def init_services():
    """
    Initializes service:
    """
    srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)


if __name__ == '__main__':
    main()