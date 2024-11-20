#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped

pose_pub = rospy.Publisher('rexrov/cmd_pose', PoseStamped, queue_size=1)

def pose():

  rate = rospy.Rate(100)

  while not rospy.is_shutdown():
    cmd = PoseStamped()
    cmd.header.stamp = rospy.get_rostime()
    cmd.pose.position.x = 0
    cmd.pose.position.y = 0
    cmd.pose.position.z = -1
    cmd.pose.orientation.x = 0
    cmd.pose.orientation.y = 0
    cmd.pose.orientation.z = 0
    cmd.pose.orientation.w = 1
    pose_pub.publish(cmd)
    rate.sleep()


if __name__ == '__main__':
  try:
    rospy.init_node('vehicle_pose', anonymous=True)
    pose()

  except rospy.ROSInterruptException:
    pass
