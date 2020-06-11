#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

startStop_ = False
in_lab_ = False

def clbk_drive(msg):
    global startStop_
    startStop_ = msg.data
    
def clbk_lab(msg):
    global startStop_
    startStop_ = msg.data

def move():
    msg = Twist()
    if not in_lab_:
        msg.linear.x = 0.2
    
    return msg

def coordinator():
    global in_lab_, startStop_

    rospy.init_node('coordinator')

    pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rospy.wait_for_service('/start_stop')
    rospy.wait_for_service('/in_lab')

    sub_drive = rospy.Subscriber('/start_stop', Bool, clbk_drive)
    sub_lab = rospy.Subscriber('/in_lab', Bool, clbk_lab)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        msg = Twist()
        if not startStop_:
            msg.linear.x = 0
            pub_cmd.publish(msg)
            rate.sleep()
            continue
        else:
            msg = move()
            pub_cmd.publish(msg)
        
        rate.sleep()


if __name__ == '__main__':
    try:
        coordinator()
    except rospy.ROSInterruptException:
        pass
