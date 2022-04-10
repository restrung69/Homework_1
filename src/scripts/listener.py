#!/usr/bin/env python
# LISTENER CODE

import rospy
from geometry_msgs.msg import Pose2D

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('Printing', Pose2D)
    rospy.spin()

if __name__ == '__main__':
    listener()