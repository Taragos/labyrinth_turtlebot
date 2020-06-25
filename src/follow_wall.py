#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import *

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
    1: 'turn left',
    2: 'follow the wall',
}
startStop_ = False


def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res


# callbacks
def clbk_laser(msg):
    global regions_
    regions_ = {
        'left': min(min(msg.ranges[72:107]), 10),
        'fleft': min(min(msg.ranges[36:71]), 10),
        'front': min(min(min(msg.ranges[0:35]), min(msg.ranges[324:359])), 10),
        'fright': min(min(msg.ranges[288:323]), 10),
        'right': min(min(msg.ranges[252:287]), 10),
    }

    determine_direction_of_obstacle()


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
    Checks against a variable: d, that set's the maximum distance to the obstacle  
    """
    global regions_, entry_found_
    regions = regions_
    state_description = ''

    d = 0.5

    # Nothin in Front/Front-Left/Front-Right
    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    # Found something in front, but nothing in Front-Left/Front-Right
    elif regions['front'] < d < regions['fleft'] and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    # Found something in Front-Right, but noting in Front/Front-Left
    elif regions['front'] > d > regions['fright'] and regions['fleft'] > d:
        state_description = 'case 3 - fright'
        change_state(2)
    # Found something in Front-Left, but nothing in Front/Front-Right
    elif regions['front'] > d > regions['fleft'] and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    # Found something in Front/Front-Right, but nothing in Front-Left    
    elif regions['front'] < d < regions['fleft'] and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    # Found something in Front/Front-Left, but nothing in Front-Right    
    elif regions['front'] < d < regions['fright'] and regions['fleft'] < d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    # Found something in Front/Front-Right/Front-Left    
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    # Found something in Front-Left/Front-Right, but nothing in Front
    elif regions['front'] > d > regions['fleft'] and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)


def find_wall():
    """
    Generates a Twist Message, that let's the robot drive in a circle
    """
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.3
    return msg


def turn_left():
    """
    Generates a Twist Message, that let's the robot turn left
    """
    msg = Twist()
    msg.angular.z = 0.3
    return msg


def follow_the_wall():
    """
    Generates a Twist Message, that let's the robot drive forward
    """
    global regions_

    msg = Twist()
    msg.linear.x = 0.2
    return msg


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
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')

        pub_cmd_vel_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()