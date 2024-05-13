#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64, Float32MultiArray
from sensor_msgs.msg import JointState

joint_received = False
q1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q2 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q1_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q2_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q1_end = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q2_end = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

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
duration = 10.0



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



def joint_cb(msg):
  global q1_start, q2_start, joint_received
  if joint_received == False:
    q1_start[0] = msg.position[1]
    q1_start[1] = msg.position[2]
    q1_start[2] = msg.position[3]
    q1_start[3] = msg.position[4]
    q1_start[4] = msg.position[5]
    q1_start[5] = msg.position[6]

    q2_start[0] = msg.position[12]
    q2_start[1] = msg.position[13]
    q2_start[2] = msg.position[14]
    q2_start[3] = msg.position[15]
    q2_start[4] = msg.position[16]
    q2_start[5] = msg.position[17]
    joint_received = True


def joint_cmd():
  global q1, q2
  global q1_start, q2_start

  init_reach = False
  rate = rospy.Rate(rate_HZ)
  rospy.sleep(1)

  
  while not rospy.is_shutdown():
    if joint_received == True:
      print("Initial joint received")
      break
    rate.sleep()

  start_time = rospy.Time.now()

  while not rospy.is_shutdown():
    now_time = rospy.Time.now()
    sim_time = now_time - start_time
    t = (sim_time.secs + sim_time.nsecs*1e-9)/duration

    if t < 0:
      S = 0
    elif t < 1:
      S = 3*(t**2) - 2*(t**3)
    else:
      S = 1

    q1_t = q1_start + S*(q1_end - q1_start)
    q2_t = q2_start + S*(q2_end - q2_start)

    if t <= 1:
      q1_pub = q1_t
      q2_pub = q2_t
    else:
      q1_pub = q1
      q2_pub = q2
      if init_reach == False:
        print("Home configuration reached, using desired joint commands")
        init_reach = True


    cmd1 = Float64()
    cmd2 = Float64()
    for i in range(6):
      cmd1.data = q1_pub[i]
      cmd2.data = q2_pub[i]

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
    rospy.Subscriber('rexrov/joint_states', JointState, joint_cb)
    joint_cmd()

  except rospy.ROSInterruptException:
    pass
