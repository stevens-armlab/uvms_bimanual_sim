#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64, Float32MultiArray
from sensor_msgs.msg import JointState


q1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q2 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
gripL = 0.0
gripR = 0.0

arm1_pub = []
arm2_pub = []
for i in range(6):
  arm1_pub.append(rospy.Publisher('rexrov/L_j' + str(i+1) + '/command', Float64, queue_size=1))
  arm2_pub.append(rospy.Publisher('rexrov/R_j' + str(i+1) + '/command', Float64, queue_size=1))

arm1_pub.append(rospy.Publisher('rexrov/L_gripL/command', Float64, queue_size=1))
arm1_pub.append(rospy.Publisher('rexrov/L_gripR/command', Float64, queue_size=1))
arm2_pub.append(rospy.Publisher('rexrov/R_gripL/command', Float64, queue_size=1))
arm2_pub.append(rospy.Publisher('rexrov/R_gripR/command', Float64, queue_size=1))

joint_arm1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
joint_arm2 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
joint_grip1 = np.array([0.0, 0.0, 0.0, 0.0])
joint_grip2 = np.array([0.0, 0.0, 0.0, 0.0])

vel_arm1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
vel_arm2 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
vel_grip1 = np.array([0.0, 0.0, 0.0, 0.0])
vel_grip2 = np.array([0.0, 0.0, 0.0, 0.0])

# K1 = []
# K2 = [36, 126.66, 2.5182]
# K3 = [54, 322.3881, 2.2612]
# Kp = [6.0, 48.0, 36.0, 6.0, 6.0, 6.0]
# Ki = [25.8844, 207.0751, 155.3063, 25.8844, 25.8844, 25.8844]
# Kd = [0.3477, 2.7816, 2.0862, 0.3477, 0.3477, 0.3477]
Kp = np.array([30.0, 60.0, 50.0, 50.0, 10.0, 3.0])
Ki = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# Kd = np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
Kd = np.array([1, 1, 1, 1, 1, 1])*5

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


def joint_cb(data):
  global joint_arm1, joint_arm2, joint_grip1, joint_grip2
  global vel_arm1, vel_arm2, vel_grip1, vel_grip2

  joint_arm1 = [data.position[1], data.position[2], data.position[3], data.position[4], data.position[5], data.position[6]]
  joint_grip1 = [data.position[7], data.position[8], data.position[9], data.position[10]]
  joint_arm2 = [data.position[12], data.position[13], data.position[14], data.position[15], data.position[16], data.position[17]]
  joint_grip2 = [data.position[18], data.position[19], data.position[20], data.position[21]]

  vel_arm1 = [data.velocity[1], data.velocity[2], data.velocity[3], data.velocity[4], data.velocity[5], data.velocity[6]]
  vel_grip1 = [data.velocity[7], data.velocity[8], data.velocity[9], data.velocity[10]]
  vel_arm2 = [data.velocity[12], data.velocity[13], data.velocity[14], data.velocity[15], data.velocity[16], data.velocity[17]]
  vel_grip2 = [data.velocity[18], data.velocity[19], data.velocity[20], data.velocity[21]]

  print(joint_arm1)
  print(joint_arm2)
  print("")



def joint_cmd():
  global q1, q2
  global joint_arm1, joint_arm2, joint_grip1, joint_grip2
  global vel_arm1, vel_arm2, vel_grip1, vel_grip2

  rate = rospy.Rate(rate_HZ)
  error_int1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
  error_int2 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
  while not rospy.is_shutdown():
    cmd1 = Float64()
    cmd2 = Float64()
    for i in range(6):
      error_pos1 = q1[i] - joint_arm1[i]
      error_pos2 = q2[i] - joint_arm2[i]
      error_int1[i] = error_int1[i] + error_pos1*dt
      error_int2[i] = error_int2[i] + error_pos2*dt
      error_vel1 = 0 - vel_arm1[i]
      error_vel2 = 0 - vel_arm2[i]

      cmd1.data = Kp[i]*error_pos1 + Ki[i]*error_int1[i] + Kd[i]*error_vel1
      cmd2.data = Kp[i]*error_pos2 + Ki[i]*error_int2[i] + Kd[i]*error_vel2

      # arm1_pub[i].publish(cmd1)
      # arm2_pub[i].publish(cmd2)

    # for i in range(6,8):
    #   # cmd1 = Float64()
    #   # cmd2 = Float64()
    #   error_grip1 = gripL - joint_grip1[]
    #   error_grip2 = gripR - joint_grip2[]
    #   cmd1.data = 1*error_grip1
    #   cmd2.data = 1*error_grip2
    #   arm1_pub[i].publish(cmd1)
    #   arm2_pub[i].publish(cmd2)
    rate.sleep()


if __name__ == '__main__':
  try:
    rospy.init_node('joint_position_command', anonymous=True)
    # rospy.Subscriber('joy', Joy, joy_cb)
    rospy.Subscriber('arm_command', Float32MultiArray, arm_cb)
    rospy.Subscriber('gripper_cmd', Float32MultiArray, grip_cb)
    rospy.Subscriber('rexrov/joint_states', JointState, joint_cb)
    joint_cmd()

  except rospy.ROSInterruptException:
    pass
