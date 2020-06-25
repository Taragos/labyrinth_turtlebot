#! /usr/bin/env python
import math

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
from std_srvs.srv import *
from tf.transformations import euler_from_quaternion

active_ = False
pub_cmd_vel_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
entry_found_ = 0
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn right',
    2: 'turn left',
    3: 'follow the wall',
    4: 'turn right on point',
    5: 'turn left on point',
    # 6: 'drive backwards',
}
startStop_ = False

wall_found = 0
wall_dist = 0.25
max_speed = 0.2

inf = 3.6

orientation_ = Point()
position_ = Point()
roll = pitch = yaw = 0.0
x = y = 0.0
yaw_degree = 0.0
desired_yaw = 0
yaw_dict_ = {
    0: 0,
    1: 1.5,
    2: 3,
    3: -1.5,
}


def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res


# callbacks
def clbk_laser(msg):
    global regions_, inf
    regions_ = {
        'left': min(min(msg.ranges[72:107]), inf),
        'fleft': min(min(msg.ranges[36:71]), inf),
        'front': min(min(min(msg.ranges[0:35]), min(msg.ranges[324:359])), inf),
        'fright': min(min(msg.ranges[288:323]), inf),
        'right': min(min(msg.ranges[252:287]), inf),
    }

    determine_direction_of_obstacle()


def clbk_position(msg):
    """
    Callback for the /odom Topic
    Get's the odometry data of the roboter and extracts valuable information
    position_: Current world position of roboter
    """
    global orientation_, position_, roll, pitch, yaw, yaw_degree  # position_marker,
    position_ = msg.pose.pose.position
    orientation_ = msg.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([orientation_.x, orientation_.y, orientation_.z, orientation_.w])
    yaw_degree = (yaw * (180 / math.pi))


def clbk_drive(msg):
    global startStop_
    startStop_ = msg.data


def change_state(state):
    global state_, state_dict_
    if state is not state_:
        rospy.loginfo('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state


def determine_direction_of_obstacle():
    """
    Checks the Regions and determines in which regions are obstacles  
    Checks against a variable: wall_dist, that set's the maximum distance to the obstacle
    """
    global regions_, entry_found_, wall_dist, wall_found
    regions = regions_
    state_description = ''

    # rospy.loginfo("---------------------------")
    # rospy.loginfo("Deadend: " + str(is_deadend()))
    # rospy.loginfo("Outer Corner: " + str(is_outer_corner()))
    # rospy.loginfo("Inner Corner: " + str(is_inner_corner()))
    # rospy.loginfo("---------------------------")
    #
    # rospy.loginfo("---------------------------")
    # rospy.loginfo("Left: " + str(wall_left()))
    # rospy.loginfo("FLeft: " + str(wall_fleft()))
    # rospy.loginfo("Front: " + str(wall_front()))
    # rospy.loginfo("FRight: " + str(wall_fright()))
    # rospy.loginfo("Right: " + str(wall_right()))
    # rospy.loginfo("---------------------------")

    # rospy.loginfo("---------------------------")
    # rospy.loginfo("Left: " + str(regions_['left']))
    # rospy.loginfo("Left: " + str(regions_['fleft']))
    # rospy.loginfo("Front: " + str(regions_['front']))
    # rospy.loginfo("Right: " + str(regions_['fright']))
    # rospy.loginfo("Right: " + str(regions_['right']))
    # rospy.loginfo("---------------------------")

    if wall_found == 0:
        change_state(0)
    else:
        # 0: 'find the wall', 1: 'turn left', 2: 'turn right', 3: 'follow the wall', 4: 'turn right on point',
        # 5: 'turn left on point',

        if is_inner_corner():
            change_state(5)
        elif wall_front() and wall_fleft() and not wall_fright():
            change_state(4)
        elif wall_fright():
            change_state(1)
        elif not wall_fright() and not wall_right():
            change_state(2)
        elif wall_front():
            change_state(5)
        else:
            rospy.loginfo("Unknown state")


def wall_left():
    return regions_['left'] < wall_dist


def wall_fleft():
    return regions_['fleft'] < wall_dist


def wall_front():
    return regions_['front'] < wall_dist


def wall_fright():
    return regions_['fright'] < wall_dist


def wall_right():
    return regions_['right'] < wall_dist


def is_deadend():
    return wall_fleft() and wall_fright() and wall_front()


def is_inner_corner():
    return wall_fright() and wall_front() #  and not wall_left()


def is_outer_corner():
    return wall_right() and not wall_fright()


def is_wall_front():
    return (wall_fleft() or wall_front() or wall_fright()) and not wall_left() and not wall_right()


def find_wall():
    """
    Generates a Twist Message, that let's the robot turn right
    """
    global regions_, max_speed, wall_found
    regions = regions_
    msg = Twist()
    if regions['front'] < wall_dist or regions['fleft'] < wall_dist or regions['fright'] < wall_dist:
        wall_found = 1
        msg.linear.x = 0
        msg.angular.z = 0
    else:
        msg.linear.x = max_speed
        msg.angular.z = 0.1
    return msg


def turn_left():
    """
    Generates a Twist Message, that let's the robot turn left
    """
    msg = Twist()
    msg.linear.x = max_speed
    msg.angular.z = 0.1
    return msg


def turn_left_on_point():
    """
    Generates a Twist Message, that let's the robot turn left on spot
    """
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = 0.5
    return msg


def turn_right():
    """
    Generates a Twist Message, that let's the robot turn right
    """
    msg = Twist()
    msg.linear.x = max_speed/4
    msg.angular.z = -0.1
    return msg


def turn_right_on_point():
    """
    Generates a Twist Message, that let's the robot turn right on spot
    """
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = -0.5
    return msg


def follow_the_wall():
    """
    Generates a Twist Message, that let's the robot drive forward
    """
    global max_speed

    msg = Twist()
    msg.linear.x = max_speed/4
    msg.angular.z = 0
    return msg


# def drive_backwards():
#     """
#     Generates a Twist Message, that let's the robot drive backwards
#     """
#     global max_speed
#
#     msg = Twist()
#     msg.linear.x = -max_speed/4
#     msg.angular.z = -0.1
#     return msg


def main():
    global pub_cmd_vel_, active_

    rospy.init_node('follow_wall')

    pub_cmd_vel_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub_scan = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    sub_drive = rospy.Subscriber('/start_stop', Bool, clbk_drive)

    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():

        msg = Twist()

        if not active_ or not startStop_:
            rate.sleep()
            continue

        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_right()
        elif state_ == 2:
            msg = turn_left()
        elif state_ == 3:
            msg = follow_the_wall()
        elif state_ == 4:
            msg = turn_right_on_point()
        elif state_ == 5:
            msg = turn_left_on_point()
            pass
        else:
            rospy.logerr('Unknown state!')

        pub_cmd_vel_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()
