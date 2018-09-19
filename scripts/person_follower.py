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

def input_thread(personFollower):
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
        self.pose = convert_pose_to_xy_and_theta(msg.pose.pose)

    def getScan(self, msg):
        self.scan = msg.ranges

    def convertToXY(self, angle, r):
        newX = math.cos(math.radians(angle))*r
        newY = math.sin(math.radians(angle))*r
        return (newX, newY)

    def run(self):
        while not rospy.is_shutdown():
            prev = False
            objects = []
            closestPerson = []
            closestPersonDistance = 0
            closestPointDistance = 5
            if len(self.scan) > 0:
                for angle in range(360):
                    if self.scan[angle] > 0.0:
                        newPoint = self.convertToXY(angle, self.scan[angle])
                        if not prev or distanceTo(objects[-1][-1], newPoint) > self.maxPersonDistance:
                            objects.append([])
                            objects[-1].append((newPoint[0], newPoint[1], angle))
                            prev = True
                        else:
                            objects[-1].append((newPoint[0], newPoint[1], angle))
                            prev = True
                    else:
                        prev = False

                if distanceTo((objects[0][0][0], objects[0][0][1]), (objects[-1][-1][0], objects[-1][-1][1])) < self.maxPersonDistance:
                    objects[0] = objects[0] + objects[-1]
                    objects = objects[:-1]

                for object in objects:
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
                    if closestPointDistance < self.personFollowDistance/2:
                        self.vel.linear.x = 0


            self.velocityPublisher.publish(self.vel)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    personFollower = personFollower()
    thread.start_new_thread(input_thread, (personFollower, ))
    try:
        personFollower.run()
    except KeyboardInterrupt:
        personFollower.vel.linear.x = 0
        personFollower.vel.angular.z = 0
        personFollower.velocityPublisher.publish(personFollower.vel)