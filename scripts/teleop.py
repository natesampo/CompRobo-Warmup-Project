#!/usr/bin/env python
from __future__ import division
import tty
import select
import sys
import termios
import rospy
import os
import thread
import signal
import math
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

def angle_normalize(z):
    """ convenience function to map an angle to the range [-pi,pi] """
    return math.atan2(math.sin(z), math.cos(z))

def angle_diff(a, b):
    """ Calculates the difference between angle a and angle b (both should be in radians)
        the difference is always based on the closest rotation from angle a to angle b
        examples:
            angle_diff(.1,.2) -> -.1
            angle_diff(.1, 2*math.pi - .1) -> .2
            angle_diff(.1, .2+2*math.pi) -> -.1
    """
    a = angle_normalize(a)
    b = angle_normalize(b)
    d1 = a-b
    d2 = 2*math.pi - math.fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if math.fabs(d1) < math.fabs(d2):
        return d1
    else:
        return d2

def distanceTo(a, b):
    return math.sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]))

def mouseToXY(mousePos):
    return (mousePos[0]/1500, mousePos[1]/1500)

def getCurrentMousePosition():
    return (display.Display().screen().root.query_pointer()._data['root_x'], display.Display().screen().root.query_pointer()._data['root_y'])

def input_thread(myTeleop):
    running = True
    while running:
        '''Given code to grab keyboard values'''
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if key == 'q':
            running = False
            os.kill(os.getpid(), signal.SIGINT)
        elif key == 'a':
            myTeleop.newOrigin()
        elif key == 'w':
            myTeleop.man_toggle = not myTeleop.man_toggle
        elif key == 'i' and myTeleop.man_toggle:
            myTeleop.vel.linear.x = 1
            myTeleop.vel.angular.z = 0
        elif key =='k' and myTeleop.man_toggle:
            myTeleop.vel.linear.x = -1
            myTeleop.vel.angular.z = 0
        elif key == 'j' and myTeleop.man_toggle:
            myTeleop.vel.linear.x = 0
            myTeleop.vel.angular.z = -1
        elif key == 'l' and myTeleop.man_toggle:
            myTeleop.vel.linear.x = 0
            myTeleop.vel.angular.z = 1

class TeleopC(object):
    def __init__ (self):
        rospy.init_node("teleop", disable_signals=True)
        self.publisher_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10, latch=True)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.getOdom)
        self.mouseOrigin = (display.Display().screen().root.query_pointer()._data['root_x'], display.Display().screen().root.query_pointer()._data['root_y'])
        self.velocityPublisher = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.angle = 0
        self.speed = 1
        self.rate = rospy.Rate(10)
        self.vel = Twist()
        self.pose = (0, 0, 0)
        self.origin = self.pose
        self.getNewOrigin = True
        self.man_toggle = False

    def getOdom(self, msg):
        self.pose = convert_pose_to_xy_and_theta(msg.pose.pose)
        if self.getNewOrigin:
            self.newOrigin()
            self.getNewOrigin = False

    def newOrigin(self):
        self.origin = self.pose
        #self.angle = self.pose[2]
        self.mouseOrigin = getCurrentMousePosition()

    def getRelativeMousePosition(self, mousePos):
        return (mousePos[0] - self.mouseOrigin[0], mousePos[1] - self.mouseOrigin[1])

    def run(self):
        while not rospy.is_shutdown():
            if not self.man_toggle:
                currMouseOffset = (mouseToXY(self.getRelativeMousePosition(getCurrentMousePosition()))[0] - (self.pose[0] - self.origin[0]), mouseToXY(self.getRelativeMousePosition(getCurrentMousePosition()))[1] - (self.pose[1] - self.origin[1]))
                currAngleOffset = -math.atan2(currMouseOffset[1], currMouseOffset[0])-self.angle
                if abs(angle_diff(self.pose[2], currAngleOffset)) < 0.05:
                    if math.sqrt((currMouseOffset[0]*currMouseOffset[0]) + (currMouseOffset[1]*currMouseOffset[1])) > 0.2:
                        self.vel.linear.x = -self.speed*(math.sqrt((currMouseOffset[0]*currMouseOffset[0]) + (currMouseOffset[1]*currMouseOffset[1]))/2)
                        self.vel.angular.z = 0
                else:
                    if math.sqrt((currMouseOffset[0]*currMouseOffset[0]) + (currMouseOffset[1]*currMouseOffset[1])) > 0.2:
                        if currAngleOffset > 0:
                            self.vel.angular.z = self.speed*(currAngleOffset/3.14)
                        else:
                            self.vel.angular.z = -self.speed*(currAngleOffset/3.14)
                    else:
                        self.vel.angular.z = 0
                    self.vel.linear.x = 0

            print(currAngleOffset)
            self.velocityPublisher.publish(self.vel)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    myTeleop = TeleopC()
    thread.start_new_thread(input_thread, (myTeleop, ))
    try:
        myTeleop.run()
    except KeyboardInterrupt:
        myTeleop.vel.linear.x = 0
        myTeleop.vel.angular.z = 0
        myTeleop.velocityPublisher.publish(myTeleop.vel)
