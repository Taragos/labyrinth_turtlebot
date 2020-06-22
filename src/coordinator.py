#!/usr/bin/env python
import time

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf import transformations
from visualization_msgs.msg import MarkerArray

# Services
srv_client_find_the_entry_ = None
srv_client_wall_follower_ = None
srv_client_go_to_point_ = None

# Publisher
pub_cmd_ = None
pub_path_change = None

# Parameters
path = None
position_ = Point()
yaw_ = 0
a_star_ = False
entry_found_ = False
startStop_ = False
in_lab_ = False
path_found_ = False


def clbk_drive(msg):
    global startStop_
    startStop_ = msg.data


def clbk_lab(msg):
    global in_lab_
    in_lab_ = msg.data


def clbk_entry(msg):
    global entry_found_
    entry_found_ = msg.data


def clbk_path(msg):
    global path_found_, pub_path_change
    path = msg.markers
    deviation = Point()
    tmp_deviation = Point()
    destination_points = []
    pub_path_change.publish(True)
    path_found_ = True

    for i in range(len(path)):
        if i == 0:
            cur = path[i].pose.position
            next = path[i + 1].pose.position

            deviation.x = round(next.x - cur.x, 2)
            deviation.y = round(next.y - cur.y, 2)
        elif i == len(path) - 3:
            destination_points.append(dict({"x": round(path[i + 2].pose.position.x, 2), "y": round(path[i + 2].pose.position.y, 2)}))
            break
        else:
            cur = path[i].pose.position
            next = path[i + 1].pose.position
            tmp_deviation.x = round(next.x - cur.x, 2)
            tmp_deviation.y = round(next.y - cur.y, 2)
            if not deviation.x == tmp_deviation.x or not deviation.y == tmp_deviation.y:
                next_next = path[i + 2].pose.position
                tmp_deviation.x = round(next_next.x - cur.x, 2)
                tmp_deviation.y = round(next_next.y - cur.y, 2)
                if not deviation.x * 2 == tmp_deviation.x or not deviation.y * 2 == tmp_deviation.y:
                    destination_points.append(dict({"x": round(cur.x, 2), "y": round(cur.y, 2)}))
                    deviation.x = round(next.x - cur.x, 2)
                    deviation.y = round(next.y - cur.y, 2)
    rospy.set_param("/path_corner_points", destination_points)


def clbk_odom(msg):
    global position_, yaw_

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


def drive_back():
    global pub_cmd_
    change_state(3)
    twist_msg = Twist()
    twist_msg.linear.x = -0.2
    pub_cmd_.publish(twist_msg)
    time.sleep(2)


# 0 = Find entry,
# 1 = Follow wall,
# 2 = use a-star
def change_state(state):
    global srv_client_find_the_entry_, srv_client_go_to_point_, srv_client_wall_follower_
    if state == 0:
        resp = srv_client_find_the_entry_(True)
        resp = srv_client_wall_follower_(False)
        resp = srv_client_go_to_point_(False)
    elif state == 1:
        resp = srv_client_find_the_entry_(False)
        resp = srv_client_wall_follower_(True)
        resp = srv_client_go_to_point_(False)
    elif state == 2:
        resp = srv_client_find_the_entry_(False)
        resp = srv_client_wall_follower_(False)
        resp = srv_client_go_to_point_(True)
    elif state == 3:
        resp = srv_client_find_the_entry_(False)
        resp = srv_client_wall_follower_(False)
        resp = srv_client_go_to_point_(False)


def coordinator():
    global in_lab_, startStop_, path_found_, entry_found_, a_star_
    global pub_path_change, pub_cmd_
    global srv_client_find_the_entry_, srv_client_wall_follower_, srv_client_go_to_point_

    rospy.init_node('coordinator')

    rospy.wait_for_service('/start_stop')
    rospy.wait_for_service('/in_lab')
    rospy.wait_for_service('/find_the_entry_switch')
    rospy.wait_for_service('/wall_follower_switch')
    rospy.wait_for_service('/go_to_point_switch')

    srv_client_find_the_entry_ = rospy.ServiceProxy('/find_the_entry_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)
    srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)

    pub_path_change = rospy.Publisher('/path_change', Bool, queue_size=10)
    pub_cmd_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub_drive = rospy.Subscriber('/start_stop', Bool, clbk_drive)
    sub_lab = rospy.Subscriber('/in_lab', Bool, clbk_lab)
    sub_path = rospy.Subscriber('/a_path', MarkerArray, clbk_path)
    sub_entry = rospy.Subscriber('/entry', Bool, clbk_entry)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if startStop_:
            if not entry_found_:
                change_state(0)
            elif not a_star_:
                change_state(1)
            else:
                change_state(2)

        rate.sleep()


if __name__ == '__main__':
    try:
        coordinator()
    except rospy.ROSInterruptException:
        pass
