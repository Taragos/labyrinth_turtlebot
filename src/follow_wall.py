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
    2: 'turn right',
    3: 'follow the wall',
    4: 'turn right on point',
    5: 'turn left on point',
}
startStop_ = False

wall_found = 0
wall_dist = 0.2
max_speed = 0.05


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
    Checks against a variable: wall_dist, that set's the maximum distance to the obstacle
    """
    global regions_, entry_found_, wall_dist, wall_found
    regions = regions_
    state_description = ''

    if wall_found == 0:
        change_state(0)
    else:
        # 1 = right, 2 = left, 3 = wall follow, 4 = point rotate
        if regions['fright'] > wall_dist:
            change_state(1)
        elif regions['fright'] < wall_dist and regions['front'] < wall_dist:
            change_state(5)
        elif regions['front'] < wall_dist:
            change_state(4)
        elif regions['fright'] < wall_dist:
            change_state(2)
        elif regions['fright'] > wall_dist > regions['front']:
            change_state(4)
        else:
            change_state(3)
    #
    #     if regions['front'] > wall_dist and regions['fright'] > wall_dist:
    #         state_description = 'case 1 - nothing'
    #         change_state(1)
    #     elif regions['front'] < wall_dist < regions['fright']:
    #         state_description = 'case 2 - front'
    #         change_state(2)
    #     elif regions['front'] > wall_dist > regions['fright']:
    #         state_description = 'case 3 - fright'
    #         change_state(3)
    #     elif regions['front'] > wall_dist and regions['fright'] > wall_dist:
    #         state_description = 'case 4 - fleft'
    #         change_state(1)
    #     elif regions['front'] < wall_dist and regions['fright'] < wall_dist:
    #         state_description = 'case 5 - front and fright'
    #         change_state(2)
    #     elif regions['front'] < wall_dist and regions['fleft'] < wall_dist:
    #         state_description = 'case 6 - front and fleft'
    #         change_state(2)
    #     elif regions['front'] < wall_dist and regions['fright'] < wall_dist:
    #         state_description = 'case 7 - front and fleft and fright'
    #         change_state(2)
    #     elif regions['front'] > wall_dist > regions['fright']:
    #         state_description = 'case 8 - fleft and fright'
    #         change_state(2)
    #     else:
    #         state_description = 'unknown case'
    #         rospy.loginfo(regions)
    #
    # rospy.loginfo(state_description)


def find_wall():
    """
    Generates a Twist Message, that let's the robot turn left
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
    msg.angular.z = 0.5
    return msg


def turn_right():
    """
    Generates a Twist Message, that let's the robot turn right
    """
    msg = Twist()
    msg.linear.x = max_speed
    msg.angular.z = -0.5
    return msg


def turn_right_on_point():
    """
    Generates a Twist Message, that let's the robot turn right
    """
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = -0.5
    return msg


def turn_left_on_point():
    """
    Generates a Twist Message, that let's the robot turn right
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
    msg.linear.x = max_speed
    msg.angular.z = 0
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
            msg = turn_right()
        elif state_ == 2:
            msg = turn_left()
        elif state_ == 3:
            msg = follow_the_wall()
        elif state_ == 4:
            msg = turn_right()
        elif state_ == 5:
            msg = turn_left_on_point()
            pass
        else:
            rospy.logerr('Unknown state!')

        pub_cmd_vel_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()