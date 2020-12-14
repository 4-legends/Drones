#!/usr/bin/env python

import rospy
import actionlib
import sys
from rob599_project.msg import go_to_targetAction, go_to_targetGoal, go_to_targetResult
from geometry_msgs.msg import PoseStamped

class go_to_target_client:
	def __init__(self):
		self.client = actionlib.SimpleActionClient('go_to_target', go_to_targetAction)


	def done_callback(self, status, result):

		if status == actionlib.GoalStatus.SUCCEEDED:
			rospy.loginfo('Reached goal')
		else:
			rospy.loginfo('Failed to reach goal')


	def active_callback(self):
		rospy.loginfo('Action is active')	


	def main(self):
		goal = PoseStamped()
		goal.header.frame_id = 'world'
		goal.pose.position.x = 2
		goal.pose.position.y = 1
		goal.pose.position.z = 0.5

		self.client.wait_for_server()

		self.client.send_goal(goal, done_cb=self.done_callback, active_cb=self.active_callback)

		self.client.wait_for_result()



if __name__ == '__main__':

	rospy.init_node('go_to_target_client')
	robot_seq_client = go_to_target_client()
	try:
		robot_seq_client.main()
	except KeyboardInterrupt:
		print("Shutting down")
	
