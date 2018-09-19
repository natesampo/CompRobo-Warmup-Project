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
    
def input_thread(driveSquare):
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

class driveSquare(object):
    def __init__ (self):
        rospy.init_node("square", disable_signals=True)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.getOdom)
        self.velocityPublisher = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.targetAngle = -1.57
        self.moving = True
        self.angle = 0
        self.speed = 0.5
        self.topSpeed = 0.7
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
        self.mouseOrigin = getCurrentMousePosition()

    def getRelativePosition(self):
        return (self.pose[0] - self.origin[0], self.pose[1] - self.origin[1])

    def getRelativeAngle(self):
        return angle_diff(self.pose[2], self.origin[2])

    def run(self):
        while not rospy.is_shutdown():
            if not self.man_toggle:
                print self.getRelativeAngle()
                if not self.moving and self.getRelativeAngle() < self.targetAngle + 0.1 and self.getRelativeAngle() > self.targetAngle - 0.1: # not turning
                    self.moving = True
                    #self.targetAngle += 3.14
                else: # turn baby turn disco inferno
                    self.vel.linear.x = 0
                    self.vel.angular.z = -min(abs(max(self.speed*(abs(self.targetAngle-self.getRelativeAngle())), 0.2)), self.topSpeed)

                if abs(self.getRelativePosition()[0]) <= 1: # you are not there yet
                    if self.moving:
                        self.vel.linear.x = self.speed#*((1.5-(self.pose[0]+self.pose[1])/2))
                        self.vel.angular.z = 0
                else: # you are
                    self.vel.linear.x = 0
                    self.vel.angular.z = 0
                    self.newOrigin()
                    self.moving = False

            self.velocityPublisher.publish(self.vel)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    driveSquare = driveSquare()
    thread.start_new_thread(input_thread, (driveSquare, ))
    try:
        driveSquare.run()
    except KeyboardInterrupt:
        driveSquare.vel.linear.x = 0
        driveSquare.vel.angular.z = 0
        driveSquare.velocityPublisher.publish(driveSquare.vel)
