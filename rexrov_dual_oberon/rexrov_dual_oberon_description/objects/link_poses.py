#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import LinkState, LinkStates
from gazebo_msgs.srv import GetLinkState
import numpy as np


def sub1_cb(msg):
	poses = msg.pose
	poseROV = poses[0]
	print("LinkROV pose X: " + str(poseROV.position.x))
	print("LinkROV pose Y: " + str(poseROV.position.y))
	print("LinkROV pose Z: " + str(poseROV.position.z))
	print("")

# def vehicle_pose():
#     pose = np.array([0, 0, 0, 0, 0, 0])
#     print("calling ROS service...")
#     rospy.wait_for_service('get_link_state')
#     try:
#         posecall = rospy.ServiceProxy('get_link_state', GetLinkState)
#         pose = posecall()
#     except rospy.ServiceException as e:
#         print("Service call failed: %s"%e)

#     print("LinkROV pose X: " + str(poses[0]))
#     print("LinkROV pose Y: " + str(poses[1]))
#     print("LinkROV pose Z: " + str(poses[2]))
#     print("")
#     print("")


if __name__ == '__main__':

	# Initialize the node
	rospy.init_node("link_poses")

	# Initialize the subscribers
	sub1 = rospy.Subscriber("/gazebo/link_states", LinkStates, callback=sub1_cb)

	rate = rospy.Rate(100)

	while not rospy.is_shutdown():
		# vehicle_pose()
		rate.sleep()