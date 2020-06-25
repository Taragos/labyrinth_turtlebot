#! /usr/bin/env python
# author: Felix Ricke

import rospy
from std_msgs.msg import Bool
from std_srvs.srv import *


def handle_in_lab_req(req):
    if req.data:
        pub_drive.publish(True)
        rospy.loginfo("In Lab")
        return SetBoolResponse(success=True)
    elif not req.data:
        pub_drive.publish(False)
        rospy.loginfo("Not in Lab")
        return SetBoolResponse(success=True)
    else:
        return SetBoolResponse(success=False)


def in_lab_server():
    rospy.Service('in_lab', SetBool, handle_in_lab_req)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node('in_lab_server', anonymous=True)
    pub_drive = rospy.Publisher('/in_lab', Bool, queue_size=10)

    in_lab_server()

