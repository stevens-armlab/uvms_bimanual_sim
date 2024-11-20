#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64, Float32MultiArray
from sensor_msgs.msg import JointState

joint_received = False
q1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q2 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q3 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q1_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q2_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q3_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q1_end = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q2_end = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q3_end = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# q1_end = np.array([0.550362926262917,\
#   0.490678715724746,\
#   -0.360602574462891,\
#   0.698039917772095,\
#   1.05436302785372,\
#   -2.68418185379610e-17])
# q2_end = np.array([-0.551234292773613,\
#   0.533751258134726,\
#   -0.338865809980599,\
#   -0.722637153503516,\
#   0.879677336950005,\
#   1.66946211900635e-17])
# q3_end = np.array([0.0736277354414705,\
#   1.38128527874382,\
#   0.429802499569804,\
#   -0.0232886519144226,\
#   -0.600364263340013,\
#   1.00185373780712e-19])

grip1 = 0.0
grip2 = 0.0
grip3 = 0.0

arm1_pub = []
arm2_pub = []
arm3_pub = []

for i in range(6):
  arm1_pub.append(rospy.Publisher('rexrov/arm1_j' + str(i+1) + '_PC/command', Float64, queue_size=1))
  arm2_pub.append(rospy.Publisher('rexrov/arm2_j' + str(i+1) + '_PC/command', Float64, queue_size=1))
  arm3_pub.append(rospy.Publisher('rexrov/arm3_j' + str(i+1) + '_PC/command', Float64, queue_size=1))

arm1_pub.append(rospy.Publisher('rexrov/arm1_gripL_PC/command', Float64, queue_size=1))
arm1_pub.append(rospy.Publisher('rexrov/arm1_gripR_PC/command', Float64, queue_size=1))
arm2_pub.append(rospy.Publisher('rexrov/arm2_gripL_PC/command', Float64, queue_size=1))
arm2_pub.append(rospy.Publisher('rexrov/arm2_gripR_PC/command', Float64, queue_size=1))
arm3_pub.append(rospy.Publisher('rexrov/arm3_gripL_PC/command', Float64, queue_size=1))
arm3_pub.append(rospy.Publisher('rexrov/arm3_gripR_PC/command', Float64, queue_size=1))




rate_HZ = 100.0
dt = 1/rate_HZ
duration = 10.0



def arm_cb(cmd):
  global q1
  global q2
  global q3
  q1 = np.array([cmd.data[0], cmd.data[1], cmd.data[2], cmd.data[3], cmd.data[4], cmd.data[5]])
  q2 = np.array([cmd.data[6], cmd.data[7], cmd.data[8], cmd.data[9], cmd.data[10], cmd.data[11]])
  q3 = np.array([cmd.data[12], cmd.data[13], cmd.data[14], cmd.data[15], cmd.data[16], cmd.data[17]])




def grip_cb(cmd):
  global grip1
  global grip2
  grip1 = cmd.data[0]
  grip2 = cmd.data[1]
  grip2 = cmd.data[2]



def joint_cb(msg):
  global q1_start, q2_start, q3_start, joint_received
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

    q3_start[0] = msg.position[23]
    q3_start[1] = msg.position[24]
    q3_start[2] = msg.position[25]
    q3_start[3] = msg.position[26]
    q3_start[4] = msg.position[27]
    q3_start[5] = msg.position[28]
    joint_received = True


def joint_cmd():
  global q1, q2, q3
  global q1_start, q2_start, q3_start

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
    q3_t = q3_start + S*(q3_end - q3_start)

    if t <= 1:
      q1_pub = q1_t
      q2_pub = q2_t
      q3_pub = q3_t
    else:
      q1_pub = q1
      q2_pub = q2
      q3_pub = q3
      if init_reach == False:
        print("Home configuration reached, using desired joint commands")
        init_reach = True


    cmd1 = Float64()
    cmd2 = Float64()
    cmd3 = Float64()
    for i in range(6):
      cmd1.data = q1_pub[i]
      cmd2.data = q2_pub[i]
      cmd3.data = q3_pub[i]

      arm1_pub[i].publish(cmd1)
      arm2_pub[i].publish(cmd2)
      arm3_pub[i].publish(cmd3)

    for i in range(6,8):
      cmd1 = Float64()
      cmd2 = Float64()
      cmd3 = Float64()
      cmd1.data = grip1
      cmd2.data = grip2
      cmd3.data = grip3
      arm1_pub[i].publish(cmd1)
      arm2_pub[i].publish(cmd2)
      arm3_pub[i].publish(cmd3)
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
