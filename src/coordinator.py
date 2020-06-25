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
def clbk_laser(msg):
    global regions_
    regions_ = {
        'right': min(min(msg.ranges[72:107]), 10),
        'fright': min(min(msg.ranges[36:71]), 10),
        'front': min(min(min(msg.ranges[0:35]), min(msg.ranges[324:359])), 10),
        'fleft': min(min(msg.ranges[288:323]), 10),
        'left': min(min(msg.ranges[252:287]), 10),
    }


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

    print('X: ' + str(position_.x) + ' / Y: ' + str(position_.y))
    print('Yaw: ' + str(yaw) + '\n')
    print('Yaw Degree: ' + str(yaw_degree))
    print('Marker Position: ' + str(x) + '/' + str(y))


def marker(x, y):
    global pub_visualization_marker_

    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = Time()
    marker.ns = "labyrinth_turtlebot_algo"
    marker.id = 0
    marker.text = "Algo_Marker"
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
    marker.color.b = 1.0

    pub_visualization_marker_.publish(marker)


def handle_marker_req(req):
    global x, y
    if req.data:
        if 0 <= yaw_degree <= 90:
            x = position_.x + (10 * math.cos(yaw_degree))
            y = position_.y + (10 * math.sin(yaw_degree))
        elif 90 < yaw_degree <= 180:
            x = position_.x - (10 * math.cos(180 - yaw_degree))
            y = position_.y + (10 * math.sin(180 - yaw_degree))
        elif 180 < yaw_degree < 270:
            x = position_.x - (10 * math.cos(180 - abs(yaw_degree)))
            y = position_.y + (10 * math.sin(180 - abs(yaw_degree)))
        else:
            x = position_.x + (10 * math.cos(abs(yaw_degree)))
            y = position_.y + (10 * math.sin(abs(yaw_degree)))

        marker(x, y)
        rospy.loginfo("Set new Marker")
        return SetBoolResponse(success=True, message="New Marker was set")
    else:
        return SetBoolResponse(success=False)


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

    rospy.Service('set_marker', SetBool, handle_marker_req)
    sub_odom = rospy.Subscriber('/odom', Odometry, position)
    pub_visualization_marker_ = rospy.Publisher("/visualization_marker", Marker, queue_size=1)

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


# def coordinator():
#     rospy.init_node('coordinator', anonymous=True)
#
#     pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
#     rospy.Service('start_stop', SetBool, start_stop)
#
#     rate = rospy.Rate(10)
#
#     move_forward = Twist()
#     move_forward.linear.x = 0.22
#
#     while not rospy.is_shutdown():
#         rospy.loginfo("Moving forward, biatch")
#         pub.publish(move_forward)
#         rate.sleep()


if __name__ == '__main__':
    try:
        coordinator()
    except rospy.ROSInterruptException:
        pass
