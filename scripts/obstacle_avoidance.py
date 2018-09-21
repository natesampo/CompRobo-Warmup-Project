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

def input_thread(avoid_obst):
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

class avoid_obst(object):
    def __init__ (self):
        rospy.init_node("avoid_obst", disable_signals=True)
        self.scan_subscriber = rospy.Subscriber('scan', LaserScan, self.getScan)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.getOdom)
        self.velocityPublisher = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.speed = 0.1
        self.rate = rospy.Rate(10)
        self.vel = Twist()
        self.pose = (0, 0, 0)
        self.scan = []
        self.minimumObjectPoints = 2
        self.maxObjectDistance = 0.2
        self.maxTrackDistance = 1
        self.totalForce = (0, 0)
        self.goal = (0, 0)

    def getOdom(self, msg):
        '''Receive and process odom messages, store them as self.pose'''
        self.pose = convert_pose_to_xy_and_theta(msg.pose.pose)
        if self.goal == (0, 0):
            self.goal = (self.pose[0], self.pose[1]-2)

    def getScan(self, msg):
        '''Receive and process scan messages, store them as self.scan'''
        self.scan = msg.ranges

    def convertToXY(self, angle, r):
        '''Given a polar coordinate, return the corresponding cartesian coordinate'''
        newX = math.cos(math.radians(angle-90))*r
        newY = math.sin(math.radians(angle-90))*r
        return (newX, newY)

    def convertToPolar(self, x, y):
        '''Given a cartesian coordinate, return the corresponding polar coordinate'''
        r = math.sqrt(x*x + y*y)
        theta = math.atan2(y, x)+1.57
        return (r, theta)

    def processScan(self, angle, prev):
        '''Given an angle and whether or not the previous point was part of an object, create or add to an object and return if this point is part of an object'''
        if self.scan[angle] > 0.0 and self.scan[angle] < self.maxTrackDistance:
            newPoint = self.convertToXY(angle, self.scan[angle])
            if not prev or distanceTo(self.objects[-1][-1], newPoint) > self.maxObjectDistance:
                self.objects.append([])
                self.objects[-1].append((newPoint[0], newPoint[1], angle))
                prev = True
            else:
                self.objects[-1].append((newPoint[0], newPoint[1], angle))
                prev = True
        else:
            prev = False

        return prev

    def processObjects(self):
        '''Use the current object list to calculate the forces each object applies to the robot by finding each object's closest point'''
        closestPoints = []
        if distanceTo((self.objects[0][0][0], self.objects[0][0][1]), (self.objects[-1][-1][0], self.objects[-1][-1][1])) < self.maxObjectDistance:
            self.objects[0] = self.objects[0] + self.objects[-1]
            self.objects = self.objects[:-1]

        for object in self.objects:
            if len(object) >= self.minimumObjectPoints:
                closestPoints.append(5)
                for point in object:
                    if math.sqrt((point[0]*point[0])+(point[1]*point[1])) < closestPoints[-1]:
                        closestPoints[-1] = point
            else:
                self.objects.remove(object)

        for point in closestPoints:
            self.totalForce = (self.totalForce[0]-point[0], self.totalForce[1]-point[1]*10)

        return closestPoints

    def run(self):
        while not rospy.is_shutdown():
            prev = False
            self.objects = []
            closestPoints = []
            self.vel.linear.x = self.speed/4
            self.vel.angular.z = 0
            if len(self.scan) > 0:
                theta = math.atan2(self.goal[1]-self.pose[1], self.goal[0]-self.pose[0])
                self.totalForce = (distanceTo(self.goal, (self.pose[0], self.pose[1]))*math.cos(theta), distanceTo(self.goal, (self.pose[0], self.pose[1]))*math.sin(theta))
                for angle in range(360):
                    prev = self.processScan(angle, prev)

                if len(self.objects) > 0:
                    closestPoints = self.processObjects()

                self.vel.linear.x = self.speed*self.convertToPolar(self.totalForce[0], self.totalForce[1])[0]/2
                self.vel.angular.z = self.speed*self.convertToPolar(self.totalForce[0], self.totalForce[1])[1]*5

            self.velocityPublisher.publish(self.vel)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    avoid_obst = avoid_obst()

    #Start new thread for input so it is not on the same thread as the robot processing
    thread.start_new_thread(input_thread, (avoid_obst, ))
    try:
        avoid_obst.run()
    except KeyboardInterrupt:
        avoid_obst.vel.linear.x = 0
        avoid_obst.vel.angular.z = 0
        avoid_obst.velocityPublisher.publish(avoid_obst.vel)
