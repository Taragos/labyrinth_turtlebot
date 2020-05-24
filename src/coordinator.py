#!/usr/bin/env python
import rospy 
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse

startStop = False

def coordinator():
  rospy.init_node('coordinator', anonymous=True)

  pub = rospy.Publisher('cmd_vel',Twist,queue_size=10 )
  rospy.Service('start_stop', SetBool, start_stop)

  rate = rospy.Rate(10)

  move_forward = Twist()
  move_forward.linear.x = 0.22

  while not rospy.is_shutdown():
    rospy.loginfo("Moving forward, biatch")
    pub.publish(move_forward)
    rate.sleep()


def start_stop(msg):
    global startStop
    if msg.data:
        rospy.loginfo("Stopping")
    else:
        rospy.loginfo("Starting")
    startStop = msg.data
    return SetBoolResponse(startStop, "")

if __name__ == '__main__':
  try:
    coordinator()
  except rospy.ROSInterruptException:
    pass