#!/usr/bin/env python
import rospy 
from geometry_msgs.msg import Twist

def coordinator():
  pub = rospy.Publisher('cmd_vel',Twist,queue_size=10 )
  rospy.init_node('coordinator', anonymous=True)
  rate = rospy.Rate(10)

  move_forward = Twist()
  move_forward.linear.x = 0.22

  while not rospy.is_shutdown():
    rospy.loginfo("Moving forward, biatch")
    pub.publish(move_forward)
    rate.sleep()

if __name__ == '__main__':
  try:
    coordinator()
  except rospy.ROSInterruptException:
    pass