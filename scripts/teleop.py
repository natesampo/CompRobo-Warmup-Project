#!/usr/bin/env python
from __future__ import division
import tty
import select
import sys
import termios
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Vector3, Point, PointStamped
from nav_msgs.msg import Odometry
import Xlib.display as display
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix

def convert_pose_to_xy_and_theta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return (pose.position.x, pose.position.y, angles[2])

def mouseToXY(mousePos):
    return (mousePos[0]/700, mousePos[1]/700)

def getCurrentMousePosition():
    return (display.Display().screen().root.query_pointer()._data['root_x'], display.Display().screen().root.query_pointer()._data['root_y'])

class TeleopC(object):
    def __init__ (self):
        rospy.init_node("teleop")
        self.publisher_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10, latch=True)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.getOdom)
        self.mouseOrigin = (display.Display().screen().root.query_pointer()._data['root_x'], display.Display().screen().root.query_pointer()._data['root_y'])
        self.velocityPublisher = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.rate = rospy.Rate(10)
        self.vel = Twist()
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.pose = (0, 0, 0)
        self.origin = self.pose
        self.getNewOrigin = True

    def getOdom(self, msg):
        self.pose = convert_pose_to_xy_and_theta(msg.pose.pose)
        if self.getNewOrigin:
            self.newOrigin()
            self.getNewOrigin = False

    def newOrigin(self):
        self.origin = self.pose
        self.mouseOrigin = getCurrentMousePosition()

    def getRelativeMousePosition(self, mousePos):
        return (mousePos[0] - self.mouseOrigin[0], mousePos[1] - self.mouseOrigin[1])

    def run(self):
        while not rospy.is_shutdown():
            #print(self.pose)
            print(mouseToXY(self.getRelativeMousePosition(getCurrentMousePosition())))


if __name__ == "__main__":
    myTeleop = TeleopC()
    myTeleop.run()
