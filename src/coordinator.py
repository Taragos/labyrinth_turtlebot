#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse
from visualization_msgs.msg import Marker, MarkerArray

# Services
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

srv_client_find_the_entry_ = None
srv_client_wall_follower_ = None
srv_client_go_to_point_ = None

# Publisher
pub_cmd_ = None
pub_path_change = None
pub_visualization_marker_ = None

# Parameters
hz = 20
path = None
startStop_ = False
in_lab_ = False
entry_found_ = False
a_star_ = True
path_found_ = False

# Robot position
orientation_ = Point()
position_ = Point()
roll = pitch = yaw = 0.0


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
    rospy.loginfo("----------------------------- Path")
    for i in range(len(path)):
        destination_points.append(
            dict({"x": round(path[i].pose.position.x, 3), "y": round(path[i].pose.position.y,3)}))
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
    orientation_: Current orientation in relation to absolute axis of world
    """
    global position_, orientation_, roll, pitch, yaw
    position_ = msg.pose.pose.position
    orientation_ = msg.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([orientation_.x, orientation_.y, orientation_.z, orientation_.w])


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


def handle_marker_req(req):
    """
    Service handler to set a marker 10 Meter away from Robot in relation to it's orientation in the world
    :param req: Request Data which was send by the Service
    :return: Message that Marker was Set
    """
    if req.data:
        if 0 <= yaw <= (math.pi/2):             # Robot orientation to positive X & positive Y axis
            x = position_.x + (10 * math.cos(yaw))
            y = position_.y + (10 * math.sin(yaw))
        elif (math.pi/2) < yaw <= math.pi:      # Robot orientation to negative X & positive Y axis
            x = position_.x - (10 * math.cos(math.pi-yaw))
            y = position_.y + (10 * math.sin(math.pi-yaw))
        elif 0 > yaw > (-(math.pi/2)):          # Robot orientation to positive X & negative Y axis
            x = (position_.x + (10 * math.cos(abs(yaw))))
            y = (position_.y - (10 * math.sin(abs(yaw))))
        else:                                   # Robot orientation to negative X & negative Y axis
            x = (position_.x - (10 * math.cos(math.pi-abs(yaw))))
            y = (position_.y - (10 * math.sin(math.pi-abs(yaw))))

        marker(x, y)
        rospy.loginfo("Set new Marker")
        return SetBoolResponse(success=True, message="New Marker was set")
    else:
        return SetBoolResponse(success=False)


def marker(x, y):
    """
    Function used by the set_marker Service to set marker 10 meter away
    :param x: X-Coordinate Position for Marker
    :param y: Y-Coordinate Position for Marker
    """
    global pub_visualization_marker_

    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = rospy.Time.now()
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
    global pub_path_change, pub_visualization_marker_
    pub_path_change = rospy.Publisher('/path_change', Bool, queue_size=10)
    pub_visualization_marker_ = rospy.Publisher("/visualization_marker", Marker, queue_size=1)


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
    sub_path = rospy.Subscriber('/a_path', MarkerArray, clbk_path)

def init_services():
    """
    Initializes connection to services:
    /find_the_entry_switch:
    /wall_follower_switch:
    /go_to_point_switch:
    """
    global srv_client_find_the_entry_, srv_client_go_to_point_, srv_client_wall_follower_

    rospy.Service('set_marker', SetBool, handle_marker_req)
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
