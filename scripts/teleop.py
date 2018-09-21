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
import Xlib
import Xlib.display as display
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix

def convert_pose_to_xy_and_theta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return (pose.position.x, pose.position.y, angles[2])

def distanceTo(a, b):
    '''Calculate distance from point a to point b'''
    return math.sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]))

def mouseToPercent(mousePos):
    '''Calculate percent of distance mouse is across the screen in both the x and y directions'''
    return (mousePos[0]/(display.Display().screen().root.get_geometry().width/2), mousePos[1]/(display.Display().screen().root.get_geometry().height/2))

def getCurrentMousePosition():
    '''Grabs the current mouse position'''
    return (display.Display().screen().root.query_pointer()._data['root_x'], display.Display().screen().root.query_pointer()._data['root_y'])

def input_thread(myTeleop):
    '''Seperate thread for user input, loops, checking for button input'''
    running = True
    while running:
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if key == 'q':
            '''Kill the program and stop the robot'''
            running = False
            os.kill(os.getpid(), signal.SIGINT)
        elif key == 'a':
            '''Reset origin'''
            myTeleop.newOrigin()
        elif key == 'w':
            '''Toggle manual mode'''
            myTeleop.man_toggle = not myTeleop.man_toggle
            myTeleop.vel.linear.x = 0
            myTeleop.vel.angular.z = 0
        elif key == 'i' and myTeleop.man_toggle:
            '''Forward'''
            myTeleop.vel.linear.x = 1
            myTeleop.vel.angular.z = 0
        elif key =='k' and myTeleop.man_toggle:
            '''Back'''
            myTeleop.vel.linear.x = -1
            myTeleop.vel.angular.z = 0
        elif key == 'j' and myTeleop.man_toggle:
            '''Left'''
            myTeleop.vel.linear.x = 0
            myTeleop.vel.angular.z = -1
        elif key == 'l' and myTeleop.man_toggle:
            '''Right'''
            myTeleop.vel.linear.x = 0
            myTeleop.vel.angular.z = 1
        elif key == 's'and myTeleop.man_toggle:
            '''Stop'''
            myTeleop.vel.linear.x = 0
            myTeleop.vel.angular.z = 0

class TeleopC(object):
    def __init__ (self):
        rospy.init_node("teleop", disable_signals=True)
        self.publisher_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10, latch=True)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.getOdom)
        self.mouseOrigin = (display.Display().screen().root.query_pointer()._data['root_x'], display.Display().screen().root.query_pointer()._data['root_y'])
        self.velocityPublisher = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.speed = 0.3
        self.rate = rospy.Rate(10)
        self.vel = Twist()
        self.pose = (0, 0, 0)
        self.getNewOrigin = True
        self.man_toggle = False

    def getOdom(self, msg):
        '''Receive and process odom messages, store them as self.pose'''
        self.pose = convert_pose_to_xy_and_theta(msg.pose.pose)
        if self.getNewOrigin:
            self.newOrigin()
            self.getNewOrigin = False

    def newOrigin(self):
        '''Reset the mouse origin on the screen'''
        self.mouseOrigin = getCurrentMousePosition()

    def getRelativeMousePosition(self, mousePos):
        '''Get the current mouse position relative to the mouse origin'''
        return (mousePos[0] - self.mouseOrigin[0], mousePos[1] - self.mouseOrigin[1])

    def run(self):
        while not rospy.is_shutdown():
            if not self.man_toggle:
                mouseXY = mouseToPercent(self.getRelativeMousePosition(getCurrentMousePosition()))

                self.vel.linear.x = -mouseXY[1]*self.speed
                self.vel.angular.z = -mouseXY[0]*self.speed

            self.velocityPublisher.publish(self.vel)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    myTeleop = TeleopC()

    #Start new thread for input so it is not on the same thread as the robot processing
    thread.start_new_thread(input_thread, (myTeleop, ))
    try:
        myTeleop.run()
    except KeyboardInterrupt:
        myTeleop.vel.linear.x = 0
        myTeleop.vel.angular.z = 0
        myTeleop.velocityPublisher.publish(myTeleop.vel)
