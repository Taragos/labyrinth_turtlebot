#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

# Services
from tf.transformations import euler_from_quaternion

srv_client_find_the_entry_ = None
srv_client_wall_follower_ = None
srv_client_go_to_point_ = None

# Publisher
pub_cmd_ = None
pub_path_change = None

# Parameters
hz = 20
path = None
startStop_ = False
in_lab_ = False
entry_found_ = False
a_star_ = False
path_found_ = False

# Robot position
orientation_ = Point()
position_ = Point()
roll = pitch = yaw = 0.0
x = y = 0.0
yaw_degree = 0.0

# Robot state machines
state_ = 0
state_desc_ = ['Find the entry', 'Go to point', 'wall following']


def clbk_drive(msg):
    """
    Callback for the /start_stop service topic
    Starts/Pauses the roboters activities based on the given Value{True, False}
    """
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
        destination_points.append(
            dict({"x": round(path[i].pose.position.x, 2), "y": round(path[i].pose.position.y, 2)}))
        # if i == 0:
        #     cur = path[i].pose.position
        #     next = path[i + 1].pose.position
        #
        #     deviation.x = round(next.x - cur.x, 2)
        #     deviation.y = round(next.y - cur.y, 2)
        # elif i == len(path) - 3:
        #     destination_points.append(dict({"x": round(path[i + 2].pose.position.x, 2), "y": round(path[i + 2].pose.position.y, 2)}))
        #     break
        # else:
        #     cur = path[i].pose.position
        #     next = path[i + 1].pose.position
        #     tmp_deviation.x = round(next.x - cur.x, 2)
        #     tmp_deviation.y = round(next.y - cur.y, 2)
        #     if not deviation.x == tmp_deviation.x or not deviation.y == tmp_deviation.y:
        #         next_next = path[i + 2].pose.position
        #         tmp_deviation.x = round(next_next.x - cur.x, 2)
        #         tmp_deviation.y = round(next_next.y - cur.y, 2)
        #         if not deviation.x * 2 == tmp_deviation.x or not deviation.y * 2 == tmp_deviation.y:
        #             destination_points.append(dict({"x": round(cur.x, 2), "y": round(cur.y, 2)}))
        #             deviation.x = round(next.x - cur.x, 2)
        #             deviation.y = round(next.y - cur.y, 2)
    rospy.set_param("/path_corner_points", destination_points)


def clbk_position(msg):
    """
    Callback for the /odom Topic
    Get's the odometry data of the roboter and extracts valuable information
    position_: Current world position of roboter
    """
    global position_, orientation_, roll, pitch, yaw, yaw_degree  # position_marker
    position_ = msg.pose.pose.position
    orientation_ = msg.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([orientation_.x, orientation_.y, orientation_.z, orientation_.w])
    yaw_degree = (yaw * (180 / math.pi))


# 0 = Find entry,
# 1 = use a-star,
# 2 = Follow wall
def change_state(state):
    global state_, state_desc_
    global srv_client_find_the_entry_, srv_client_go_to_point_, srv_client_wall_follower_

    state_ = state
    log = "state changed: %s" % state_desc_[state]
    # rospy.loginfo(log)

    if state_ == 0:
        resp = srv_client_find_the_entry_(True)
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(False)
    elif state_ == 1:
        resp = srv_client_find_the_entry_(False)
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    elif state_ == 2:
        resp = srv_client_find_the_entry_(False)
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)


def coordinator():
    """
    Main function of this nodes
    Calls Init functions and wait's for changes to activate different parts of the robot
    """
    global hz, in_lab_, startStop_, path_found_, entry_found_, a_star_

    rospy.init_node('coordinator')

    init_services()
    init_publisher()
    init_subscribers()

    rate = rospy.Rate(hz)

    while not rospy.is_shutdown():
        if startStop_:
            if not entry_found_:
                change_state(0)
            elif a_star_:
                change_state(1)
            else:
                change_state(2)

        rate.sleep()


def init_publisher():
    """
    Initializes Publisher:
    pub_path_change: Publishes to /path_change and executes clbk_laser
    """
    global pub_path_change
    pub_path_change = rospy.Publisher('/path_change', Bool, queue_size=10)


def init_subscribers():
    """
    Initializes Subscribers:
    sub_drive: Subscribes to /start_stop and executes clbk_drive
    sub_odom: Subscribes to /odom and executes clbk_position
    sub_entry: Subscribes to /entry and executes clbk_entry
    """
    sub_drive = rospy.Subscriber('/start_stop', Bool, clbk_drive)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_position)
    sub_entry = rospy.Subscriber('/entry', Bool, clbk_entry)


def init_services():
    """
    Initializes connection to services:
    /find_the_entry_switch:
    /wall_follower_switch:
    /go_to_point_switch:
    """
    global srv_client_find_the_entry_, srv_client_go_to_point_, srv_client_wall_follower_

    rospy.wait_for_service('/find_the_entry_switch')
    rospy.wait_for_service('/wall_follower_switch')
    rospy.wait_for_service('/go_to_point_switch')

    srv_client_find_the_entry_ = rospy.ServiceProxy('/find_the_entry_switch', SetBool)
    srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)


if __name__ == '__main__':
    try:
        coordinator()
    except rospy.ROSInterruptException:
        pass
