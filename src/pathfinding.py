#! /usr/bin/env python

# import ros stuff
import rospy
# import ros message
from nav_msgs.msg import OccupancyGrid

import math


# callbacks
def clbk_map(msg):
    map = msg.map


def main():

    rospy.init_node('pathfinding')

    sub_map = rospy.Subscriber('/map', OccupancyGrid, clbk_map)


if __name__ == "__main__":
    main()
