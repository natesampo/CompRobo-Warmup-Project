#!/usr/bin/env python
""" This script drives a robot in a 1m x 1m square """
import rospy
from geometry_msgs.msg import Twist

class square_driver(object):

	def __init__(self):
		rospy.init_node('bumpty_dumpty')
		self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
		self.rate = rospy.Rate(10)
		self.vel_msg = Twist()
		self.vel_msg.linear.x = 0
		self.vel_msg.angular.z = 0

		# set parameters
		self.time_until_turn = 3.2
		self.time_for_turn = 1.57

	def run(self):
		while not rospy.is_shutdown():
			self.move_straight()
			self.turn()
			self.rate.sleep()

	def move_straight(self):
		time_started_moving_straight = rospy.Time.now()
		time_stop_moving_straight = time_started_moving_straight + rospy.Duration(self.time_until_turn)
		self.vel_msg.linear.x = 1
		self.vel_msg.angular.z = 0

		while rospy.Time.now() < time_stop_moving_straight:
			self.publisher.publish(self.vel_msg)

	def turn(self):
		time_started_turning = rospy.Time.now()
		time_stop_turning = time_started_turning + rospy.Duration(self.time_for_turn)
		self.vel_msg.linear.x = 0
		self.vel_msg.angular.z = -1

		while rospy.Time.now() < time_stop_turning:
			self.publisher.publish(self.vel_msg)


if __name__ == '__main__':
	node = square_driver()
    node.run()
