#! /usr/bin/env python
# author: Felix Ricke

import rospy
from std_msgs.msg import Bool
from std_srvs.srv import *

def handle_start_stop_req(req):
    if req.data:
        pub_drive.publish(True)
        return SetBoolResponse(success=True)
    elif not req.data:
        pub_drive.publish(False)
        return SetBoolResponse(success=True)
    else:
        return SetBoolResponse(success=False)


def start_stop_server():
    rospy.Service('start_stop', SetBool, handle_start_stop_req)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node('start_stop_server', anonymous=True)
    pub_drive = rospy.Publisher('/start_stop', Bool, queue_size=10)

    start_stop_server()

