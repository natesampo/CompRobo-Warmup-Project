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

def input_thread(wallFollower):
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

class wallFollower(object):
    def __init__ (self):
        rospy.init_node("wall_follower", disable_signals=True)
        self.scan_subscriber = rospy.Subscriber('scan', LaserScan, self.getScan)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.getOdom)
        self.velocityPublisher = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.speed = 0.2
        self.rate = rospy.Rate(10)
        self.vel = Twist()
        self.pose = (0, 0, 0)
        self.scan = []
        self.maxWallDistance = 0.3
        self.wallFollowDistance = 1
        self.walls = []

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
            if not prev or distanceTo(self.walls[-1][-1], newPoint) > self.maxWallDistance:
                self.walls.append([])
                self.walls[-1].append((newPoint[0], newPoint[1], angle))
                prev = True
            else:
                self.walls[-1].append((newPoint[0], newPoint[1], angle))
                prev = True
        else:
            prev = False

        return prev

    def run(self):
        while not rospy.is_shutdown():
            prev = False
            self.walls = []
            largestWall = []
            largestWallLength = 0
            closestPointDistance = 5
            if len(self.scan) > 0:
                for angle in range(360):
                    prev = self.processScan(angle, prev)

                if distanceTo((self.walls[0][0][0], self.walls[0][0][1]), (self.walls[-1][-1][0], self.walls[-1][-1][1])) < self.maxWallDistance:
                    self.walls[0] = self.walls[0] + self.walls[-1]
                    self.walls = self.walls[:-1]

                for wall in self.walls:
                    if len(wall) > largestWallLength:
                        largestWallLength = len(wall)
                        largestWall = wall

                for point in largestWall:
                    if math.sqrt((point[0]*point[0])+(point[1]*point[1])) < closestPointDistance:
                        largestWallClosestPoint = point
                        closestPointDistance = math.sqrt((point[0]*point[0])+(point[1]*point[1]))

                if largestWallClosestPoint[2] > 180 and largestWallClosestPoint[2] < 280 or largestWallClosestPoint[2] >= 0 and largestWallClosestPoint[2] < 80:
                    self.vel.angular.z = -self.speed
                    self.vel.linear.x = self.speed
                elif largestWallClosestPoint[2] <= 180 and largestWallClosestPoint[2] > 100 or largestWallClosestPoint[2] <= 360 and largestWallClosestPoint[2] > 260:
                    self.vel.angular.z = self.speed
                    self.vel.linear.x = self.speed
                else:
                    self.vel.angular.z = 0
                    self.vel.linear.x = self.speed

                if len(largestWall[0]) > 1:
                    if closestPointDistance > self.wallFollowDistance:
                        if largestWallClosestPoint[2] > 50 and largestWallClosestPoint[2] < 180:
                            self.vel.angular.z = self.speed
                        elif largestWallClosestPoint[2] < 310 and largestWallClosestPoint[2] >= 180:
                            self.vel.angular.z = -self.speed
                    elif closestPointDistance < self.wallFollowDistance/1.5:
                        if largestWallClosestPoint[2] < 100:
                            self.vel.linear.x = 0
                            self.vel.angular.z = -self.speed
                        elif largestWallClosestPoint[2] > 260:
                            self.vel.linear.x = 0
                            self.vel.angular.z = self.speed

            self.velocityPublisher.publish(self.vel)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    wallFollower = wallFollower()

    #Start new thread for input so it is not on the same thread as the robot processing
    thread.start_new_thread(input_thread, (wallFollower, ))
    try:
        wallFollower.run()
    except KeyboardInterrupt:
        wallFollower.vel.linear.x = 0
        wallFollower.vel.angular.z = 0
        wallFollower.velocityPublisher.publish(wallFollower.vel)
