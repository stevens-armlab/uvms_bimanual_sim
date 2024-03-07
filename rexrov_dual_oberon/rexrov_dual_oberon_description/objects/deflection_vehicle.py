#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import LinkState, LinkStates
from gazebo_msgs.srv import GetLinkState
import numpy as np
from scipy import stats
import time

pose_x = 0.0
pose_y = 0.0
pose_z = 0.0
N = 500
values_x = np.zeros(N)
values_y = np.zeros(N)
values_z = np.zeros(N)
t = np.zeros(N)

def sub1_cb(msg):
	global pose_x, pose_y, pose_z, values_x, values_y, values_z, t

	poses = msg.pose
	pose_x = poses[0].position.x
	pose_y = poses[0].position.y
	pose_z = poses[0].position.z

	values_x = np.append(values_x[1:N], pose_x)
	values_y = np.append(values_y[1:N], pose_y)
	values_z = np.append(values_z[1:N], pose_z)

	now = rospy.get_rostime()
	t = np.append(t[1:N], now.secs + now.nsecs*1e-9)


def avg_window():
	global values_x, values_y, values_z, t

	avg_x = np.average(values_x)
	avg_y = np.average(values_y)
	avg_z = np.average(values_z)

	# x = list(range(0, N))
	# print(x)
	# print(x[1:N])
	# print(np.append(x[1:N], 100))
	# print("")

	print("Avg X: " + str(avg_x))
	print("Avg Y: " + str(avg_y))
	print("Avg Z: " + str(avg_z))
	print("")

	# print("T: " + str(t))
	# print("")

	slope_x, i, r, p, ste = stats.linregress(t, values_x)
	slope_y, i, r, p, ste = stats.linregress(t, values_y)
	slope_z, i, r, p, ste = stats.linregress(t, values_z)

	print("Slope X: " + str(slope_x))
	print("Slope Y: " + str(slope_y))
	print("Slope Z: " + str(slope_z))
	print("")

	S = np.linalg.norm(np.array([slope_x, slope_y, slope_z]))
	if S < 1e-5:
		print("Steady state reached")
		print("")

	print("================================")
	print("")

	time.sleep(1)



if __name__ == '__main__':

	# Initialize the node
	rospy.init_node("link_poses")

	# Initialize the subscribers
	sub1 = rospy.Subscriber("/gazebo/link_states", LinkStates, callback=sub1_cb)

	rate = rospy.Rate(100)

	while not rospy.is_shutdown():
		avg_window()
		rate.sleep()