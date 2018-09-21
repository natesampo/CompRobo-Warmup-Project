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
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3, Point, PointStamped
from nav_msgs.msg import Odometry
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

def input_thread(personFollower):
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

class personFollower(object):
    def __init__ (self):
        rospy.init_node("person_follower", disable_signals=True)
        self.scan_subscriber = rospy.Subscriber('scan', LaserScan, self.getScan)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.getOdom)
        self.velocityPublisher = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.speed = 0.3
        self.rate = rospy.Rate(10)
        self.vel = Twist()
        self.pose = (0, 0, 0)
        self.scan = []
        self.maxPersonDistance = 0.1
        self.personFollowDistance = 1
        self.minimumPersonPoints = 2

    def getOdom(self, msg):
        '''Receive and process odom messages, store them as self.pose'''
        self.pose = convert_pose_to_xy_and_theta(msg.pose.pose)

    def getScan(self, msg):
        '''Receive and process scan messages, store them as self.scan'''
        self.scan = msg.ranges

    def convertToXY(self, angle, r):
        '''Given a polar coordinate, return the corresponding cartesian coordinate'''
        newX = math.cos(math.radians(angle))*r
        newY = math.sin(math.radians(angle))*r
        return (newX, newY)

    def processScan(self, angle, prev):
        '''Given an angle and whether or not the previous point was part of an object, create or add to an object and return if this point is part of an object'''
        if self.scan[angle] > 0.0:
            newPoint = self.convertToXY(angle, self.scan[angle])
            if not prev or distanceTo(self.objects[-1][-1], newPoint) > self.maxPersonDistance:
                self.objects.append([])
                self.objects[-1].append((newPoint[0], newPoint[1], angle))
                prev = True
            else:
                self.objects[-1].append((newPoint[0], newPoint[1], angle))
                prev = True
        else:
            prev = False

        return prev

    def run(self):
        while not rospy.is_shutdown():
            prev = False
            self.objects = []
            closestPerson = []
            closestPersonDistance = 0
            closestPointDistance = 5
            if len(self.scan) > 0:
                for angle in range(360):
                    prev = self.processScan(angle, prev)

                if distanceTo((self.objects[0][0][0], self.objects[0][0][1]), (self.objects[-1][-1][0], self.objects[-1][-1][1])) < self.maxPersonDistance:
                    self.objects[0] = self.objects[0] + self.objects[-1]
                    self.objects = self.objects[:-1]

                if len(self.objects) > 0:
                    for object in self.objects:
                        if len(object) >= self.minimumPersonPoints:
                            for point in object:
                                if math.sqrt((point[0]*point[0])+(point[1]*point[1])) < closestPointDistance:
                                    closestPersonDistance = len(object)
                                    closestPerson = object
                                    closestPersonClosestPoint = point
                                    closestPointDistance = math.sqrt((point[0]*point[0])+(point[1]*point[1]))

                    if closestPersonClosestPoint[2] > 5 and closestPersonClosestPoint[2] <= 100:
                        self.vel.angular.z = self.speed
                        self.vel.linear.x = self.speed*closestPointDistance/2
                    elif closestPersonClosestPoint[2] < 355 and closestPersonClosestPoint[2] >= 260:
                        self.vel.angular.z = -self.speed
                        self.vel.linear.x = self.speed*closestPointDistance/2
                    elif closestPersonClosestPoint[2] <=5 or closestPersonClosestPoint[2] >= 355:
                        self.vel.angular.z = 0
                        self.vel.linear.x = self.speed*closestPointDistance/2
                    else:
                        self.vel.linear.x = 0
                        self.vel.angular.z = 0

                if len(closestPerson) > 1:
                    if closestPointDistance < self.personFollowDistance/1.8:
                        self.vel.linear.x = 0

            self.velocityPublisher.publish(self.vel)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    personFollower = personFollower()

    #Start new thread for input so it is not on the same thread as the robot processing
    thread.start_new_thread(input_thread, (personFollower, ))
    try:
        personFollower.run()
    except KeyboardInterrupt:
        personFollower.vel.linear.x = 0
        personFollower.vel.angular.z = 0
        personFollower.velocityPublisher.publish(personFollower.vel)
