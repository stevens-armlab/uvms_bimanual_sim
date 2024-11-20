#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

pub3 = rospy.Publisher('grippers', Float32MultiArray, queue_size=1)

def gripper():

  rate = rospy.Rate(100)

  while not rospy.is_shutdown():
    array = Float32MultiArray()
    array.data = [0, 0]
    pub3.publish(array)
    rate.sleep()


if __name__ == '__main__':
  try:
    rospy.init_node('gripper_pub', anonymous=True)
    gripper()

  except rospy.ROSInterruptException:
    pass
