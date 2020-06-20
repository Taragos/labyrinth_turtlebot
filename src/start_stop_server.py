#! /usr/bin/env python
# author: Felix Ricke

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import *


def handle_start_stop_req(req):
    msg = Twist()
    if req.data:
        pub_drive.publish(True)
        rospy.loginfo("Starting")
        # msg.linear.x = 0.2
        # pub_cmd.publish(msg)
        return SetBoolResponse(success=True)
    elif not req.data:
        pub_drive.publish(False)
        rospy.loginfo("Stopping")
        msg.linear.x = 0
        pub_cmd_.publish(msg)
        return SetBoolResponse(success=True)
    else:
        return SetBoolResponse(success=False)


def start_stop_server():
    rospy.Service('start_stop', SetBool, handle_start_stop_req)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node('start_stop_server', anonymous=True)
    pub_drive = rospy.Publisher('/start_stop', Bool, queue_size=10)
    pub_cmd_ = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    start_stop_server()

