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

def input_thread(avoid_obst):
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

def pointToForce(point):
    theta = math.atan2(point[1], point[0])
    distance = math.sqrt((point[0])*(point[0]) + (point[1])*(point[1]))
    return (-0.5*distance*math.cos(theta), -0.5*distance*math.sin(theta))

class avoid_obst(object):
    def __init__ (self):
        rospy.init_node("avoid_obst", disable_signals=True)
        self.scan_subscriber = rospy.Subscriber('scan', LaserScan, self.getScan)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.getOdom)
        self.velocityPublisher = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.speed = 0.2
        self.rate = rospy.Rate(10)
        self.vel = Twist()
        self.pose = (0, 0, 0)
        self.scan = []
        self.minimumObjectPoints = 2
        self.maxObjectDistance = 0.2
        self.maxTrackDistance = 1.5
        self.totalForce = (0, 0)
        self.goal = (0, 0)

    def getOdom(self, msg):
        self.pose = convert_pose_to_xy_and_theta(msg.pose.pose)
        if self.goal == (0, 0):
            self.goal = (self.pose[0], 2+self.pose[1])

    def getScan(self, msg):
        self.scan = msg.ranges

    def convertToXY(self, angle, r):
        newX = math.cos(math.radians(angle-90))*r
        newY = math.sin(math.radians(angle-90))*r
        return (newX, newY)

    def convertToRadians(self, x, y):
        r = math.sqrt(x*x + y*y)
        theta = math.degrees(math.arctan2(y, x))-90
        return (r, theta)

    def run(self):
        a=0
        while not rospy.is_shutdown():
            prev = False
            objects = []
            closestPoints = []
            self.vel.linear.x = self.speed/4
            self.vel.angular.z = 0
            if len(self.scan) > 0:
                theta = math.atan2(self.goal[1]-self.pose[1], self.goal[0]-self.pose[0])
                self.totalForce = (distanceTo(self.goal, (self.pose[0], self.pose[1]))*math.cos(theta), distanceTo(self.goal, (self.pose[0], self.pose[1]))*math.sin(theta))
                for angle in range(120):
                    if self.scan[angle] > 0.0 and self.scan[angle] < self.maxTrackDistance:
                        newPoint = self.convertToXY(angle, self.scan[angle])
                        if not prev or distanceTo(objects[-1][-1], newPoint) > self.maxObjectDistance:
                            objects.append([])
                            objects[-1].append((newPoint[0], newPoint[1], angle))
                            prev = True
                        else:
                            objects[-1].append((newPoint[0], newPoint[1], angle))
                            prev = True
                    else:
                        prev = False

                for angle in range(240, 360):
                    if self.scan[angle] > 0.0 and self.scan[angle] < self.maxTrackDistance:
                        newPoint = self.convertToXY(angle, self.scan[angle])
                        if not prev or distanceTo(objects[-1][-1], newPoint) > self.maxObjectDistance:
                            objects.append([])
                            objects[-1].append((newPoint[0], newPoint[1], angle))
                            prev = True
                        else:
                            objects[-1].append((newPoint[0], newPoint[1], angle))
                            prev = True
                    else:
                        prev = False

                if len(objects) > 0:
                    if distanceTo((objects[0][0][0], objects[0][0][1]), (objects[-1][-1][0], objects[-1][-1][1])) < self.maxObjectDistance:
                        objects[0] = objects[0] + objects[-1]
                        objects = objects[:-1]

                    for object in objects:
                        if len(object) >= self.minimumObjectPoints:
                            closestPoints.append(5)
                            for point in object:
                                if math.sqrt((point[0]*point[0])+(point[1]*point[1])) < closestPoints[-1]:
                                    closestPoints[-1] = point
                        else:
                            objects.remove(object)

                    for point in closestPoints:
                        self.totalForce = (self.totalForce[0]+pointToForce(point)[0], self.totalForce[1]+pointToForce(point)[1])

                self.vel.linear.x = self.speed*math.sqrt(self.totalForce[0]*self.totalForce[0] + self.totalForce[1]*self.totalForce[1])/6
                self.vel.angular.z = self.speed*(math.atan2(self.totalForce[1], self.totalForce[0]))/2

            a += 1
            if a%5000 == 0:
                print self.convertToRadians()
            self.velocityPublisher.publish(self.vel)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    avoid_obst = avoid_obst()
    thread.start_new_thread(input_thread, (avoid_obst, ))
    try:
        avoid_obst.run()
    except KeyboardInterrupt:
        avoid_obst.vel.linear.x = 0
        avoid_obst.vel.angular.z = 0
        avoid_obst.velocityPublisher.publish(avoid_obst.vel)
