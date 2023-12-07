#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64, Float32MultiArray
# from sensor_msgs.msg import Joy, JointState


q1 = np.vstack([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q2 = np.vstack([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
gripL = 0.0
gripR = 0.0

arm1_pub = []
arm2_pub = []
for i in range(6):
  arm1_pub.append(rospy.Publisher('rexrov/L_j' + str(i+1) + '_PC/command', Float64, queue_size=1))
  arm2_pub.append(rospy.Publisher('rexrov/R_j' + str(i+1) + '_PC/command', Float64, queue_size=1))

arm1_pub.append(rospy.Publisher('rexrov/L_gripL_PC/command', Float64, queue_size=1))
arm1_pub.append(rospy.Publisher('rexrov/L_gripR_PC/command', Float64, queue_size=1))
arm2_pub.append(rospy.Publisher('rexrov/R_gripL_PC/command', Float64, queue_size=1))
arm2_pub.append(rospy.Publisher('rexrov/R_gripR_PC/command', Float64, queue_size=1))




rate_HZ = 100.0
dt = 1/rate_HZ



def arm_cb(cmd):
  global q1
  global q2
  q1 = np.array([cmd.data[0], cmd.data[1], cmd.data[2], cmd.data[3], cmd.data[4], cmd.data[5]])
  q2 = np.array([cmd.data[6], cmd.data[7], cmd.data[8], cmd.data[9], cmd.data[10], cmd.data[11]])



def grip_cb(cmd):
  global gripL
  global gripR
  gripL = cmd.data[0]
  gripR = cmd.data[1]



def joint_cmd():
  global q1
  global q2

  rate = rospy.Rate(rate_HZ)

  while not rospy.is_shutdown():
    cmd1 = Float64()
    cmd2 = Float64()
    for i in range(6):
      cmd1.data = q1[i]
      cmd2.data = q2[i]

      arm1_pub[i].publish(cmd1)
      arm2_pub[i].publish(cmd2)

    for i in range(6,8):
      cmd1 = Float64()
      cmd2 = Float64()
      cmd1.data = gripL
      cmd2.data = gripR
      arm1_pub[i].publish(cmd1)
      arm2_pub[i].publish(cmd2)
    rate.sleep()


if __name__ == '__main__':
  try:
    rospy.init_node('joint_position_command', anonymous=True)
    # rospy.Subscriber('joy', Joy, joy_cb)
    rospy.Subscriber('arm_command', Float32MultiArray, arm_cb)
    rospy.Subscriber('gripper_cmd', Float32MultiArray, grip_cb)
    joint_cmd()

  except rospy.ROSInterruptException:
    pass
