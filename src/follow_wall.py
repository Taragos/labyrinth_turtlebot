#! /usr/bin/env python
import math

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import *

hz = 20  # Cycle Frequency
inf = 3.5  # Limit to Laser sensor range in meters, all distances above this value are
#                  considered out of sensor range
wall_dist = 0.17  # Distance desired from the wall
max_speed = 0.1  # Maximum speed of the robot on meters/seconds
active_ = False  # active state
entry_found_ = 0
startStop_ = False
direction = rospy.get_param("/direction")  # 1 for wall on the left side of the robot (-1 for the right side)
wall_found = 0

# Sensor regions
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}

# Robot state machines
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'rotate outer corner',
    2: 'rotate inner corner',
    3: 'follow the wall',
}

# Publisher
pub_cmd_vel_ = None


def wall_follower_switch(req):
    """
    Switch active state of wall follower
    """
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res


# callbacks
def clbk_laser(msg):
    """
    Read sensor messages, and determine distance to each region.
    Manipulates the values measured by the sensor.
    Callback function for the subscription to the published Laser Scan values.
    """
    global regions_, inf
    regions_ = {
        'left': min(min(msg.ranges[72:107]), inf),
        'fleft': min(min(msg.ranges[36:71]), inf),
        'front': min(min(min(msg.ranges[0:35]), min(msg.ranges[324:359])), inf),
        'fright': min(min(msg.ranges[288:323]), inf),
        'right': min(min(msg.ranges[252:287]), inf),
    }

    take_action()


def clbk_drive(msg):
    global startStop_
    startStop_ = msg.data


def change_state(state):
    """
    Update machine state
    """
    global state_, state_dict_
    if state is not state_:
        rospy.loginfo('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state


def take_action():
    """
    Change state for the machine states in accordance with the active and inactive regions of the sensor.
            State 0 No wall found - all regions infinite - Random Wandering
            State 1 Outer corner found - Rotating
            State 2 Inner corner found - Rotating
            State 3 Wall found - Following Wall
    """
    global regions_, entry_found_, direction, wall_dist, wall_found
    regions = regions_
    state_description = ''

    if wall_found == 0:
        change_state(0)
    else:
        if direction == -1:
            if regions['front'] > wall_dist and regions['fright'] > wall_dist:
                state_description = 'case 1 - nothing'
                change_state(1)
            elif regions['front'] < wall_dist < regions['fright']:
                state_description = 'case 2 - front'
                change_state(2)
            elif regions['front'] > wall_dist > regions['fright']:
                state_description = 'case 3 - fright'
                change_state(3)
            elif regions['front'] > wall_dist and regions['fright'] > wall_dist:
                state_description = 'case 4 - fleft'
                change_state(1)
            elif regions['front'] < wall_dist and regions['fright'] < wall_dist:
                state_description = 'case 5 - front and fright'
                change_state(2)
            elif regions['front'] < wall_dist and regions['fleft'] < wall_dist:
                state_description = 'case 6 - front and fleft'
                change_state(2)
            elif regions['front'] < wall_dist and regions['fright'] < wall_dist:
                state_description = 'case 7 - front and fleft and fright'
                change_state(2)
            elif regions['front'] > wall_dist > regions['fright']:
                state_description = 'case 8 - fleft and fright'
                change_state(2)
            else:
                state_description = 'unknown case'
                rospy.loginfo(regions)
        else:
            if regions['front'] > wall_dist and regions['fleft'] > wall_dist:
                state_description = 'case 1 - nothing'
                change_state(1)
            elif regions['front'] < wall_dist < regions['fleft']:
                state_description = 'case 2 - front'
                change_state(2)
            elif regions['front'] > wall_dist and regions['fleft'] > wall_dist:
                state_description = 'case 3 - fright'
                change_state(1)
            elif regions['front'] > wall_dist > regions['fleft']:
                state_description = 'case 4 - fleft'
                change_state(3)
            elif regions['front'] < wall_dist < regions['fleft']:
                state_description = 'case 5 - front and fright'
                change_state(1)
            elif regions['front'] < wall_dist and regions['fleft'] < wall_dist:
                state_description = 'case 6 - front and fleft'
                change_state(2)
            elif regions['front'] < wall_dist and regions['fleft'] < wall_dist:
                state_description = 'case 7 - front and fleft and fright'
                change_state(2)
            elif regions['front'] > wall_dist > regions['fleft']:
                state_description = 'case 8 - fleft and fright'
                change_state(2)
            else:
                state_description = 'unknown case'
                rospy.loginfo(regions)
    rospy.loginfo(state_description)


def find_wall():
    global regions_, max_speed, wall_found
    regions = regions_
    msg = Twist()
    if regions['front'] < wall_dist or regions['fleft'] < wall_dist or regions['fright'] < wall_dist:
        wall_found = 1
        msg.linear.x = 0
        msg.angular.z = 0
    else:
        msg.linear.x = max_speed
        msg.angular.z = direction * 0.3
    return msg


def rotate_inner_corner():
    """
    Rotation movement of the robot.
    Returns:
            Twist(): msg with angular and linear velocities to be published
                    msg.linear.x -> 0m/s
                    msg.angular.z -> -2 or +2 rad/s
    """
    global direction
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = direction * -0.2
    return msg


def rotate_outer_corner():
    """
    Rotation movement of the robot.
    Returns:
            Twist(): msg with angular and linear velocities to be published
                    msg.linear.x -> 0m/s
                    msg.angular.z -> -2 or +2 rad/s
    """
    global direction
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = direction * 0.2
    return msg


def follow_the_wall():
    global max_speed
    msg = Twist()
    msg.linear.x = max_speed
    msg.angular.z = 0
    return msg


def main():
    global pub_cmd_vel_, active_, hz

    rospy.init_node('follow_wall')

    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)

    pub_cmd_vel_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub_scan = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_drive = rospy.Subscriber('/start_stop', Bool, clbk_drive)

    change_state(0)

    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():

        msg = Twist()

        if not active_ or not startStop_:
            rate.sleep()
            continue

        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = rotate_outer_corner()
        elif state_ == 2:
            msg = rotate_inner_corner()
        elif state_ == 3:
            msg = follow_the_wall()
        else:
            rospy.logerr('Unknown state!')

        pub_cmd_vel_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()
