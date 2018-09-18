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
import csv

#This file helps tune a PID loop for the Neato using the ziegler nicols method


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

def input_thread(PID_Test):
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
            PID_Test.newOrigin()
        elif key == 'w':
            PID_Test.man_toggle = not PID_Test.man_toggle
        elif key == 'i' and PID_Test.man_toggle:
            PID_Test.vel.linear.x = 1
            PID_Test.vel.angular.z = 0
        elif key =='k' and PID_Test.man_toggle:
            PID_Test.vel.linear.x = -1
    def print_to_csv(PID_Test):
    #takes the pid_test and makes a file for the
    if PID_Test.test_active: #creating a time stamped csv to store data.
                PID_Test.vel.angular.z = 0
        elif key == 'j' and PID_Test.man_toggle:
            PID_Test.vel.linear.x = 0
            PID_Test.vel.angular.z = -1
        elif key == 'l' and PID_Test.man_toggle:
            PID_Test.vel.linear.x = 0
            PID_Test.vel.angular.z = 1
        elif key == 's':
            PID_Test.vel.linear.x = 0
            PID_Test.vel.angular.z = 0
        elif key == 'b' and PID_Test.mantoggle:
            #starts and ends the pid test.
            PID_Test.test_active = not PID_Test.test_active
            PID_Test.test_num +=1




class test_pid(object):
    def __init__ (self):
        rospy.init_node("square", disable_signals=True)
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
        self.test_lin = True
        self.Kp_ang = 1 #propostional Angular constant
        self.Ki_ang = 0 # intergral Angular constant
        self.Kd_ang = 0 # derivative Angular constant
        self.Kp_lin = 1 #proportional Linear constant
        self.Ki_lin = 0 #intergral Linear constant
        self.Kd_lin = 0 #derivative Linear Constant
        self.test_active = False
        self.test_num = 0
        self.test_list = []
        self.error_lin = 0
        self.error_ang = 0
        self.real_x  = self.pose[0]
        self.real_z = self.pose[2]

    def getOdom(self, msg):
        self.pose = convert_pose_to_xy_and_theta(msg.pose.pose)
        if self.getNewOrigin:
            self.newOrigin()
            self.getNewOrigin = False

    def newOrigin(self):self.error_lin = self.pose[0]-self.origin[0]
        self.origin = self.pose
        #self.angle = self.pose[2]
        #self.mouseOrigin = getCurrentMousePosition()

    def Store_val(self):
        #this function saves the current pos and stuff to a text_fileself.
        #rewrite old file and add things to the list.
        test = ""
        with open("PID_TEST.csv", 'a') as csv_file:
            csv_writer = csv.writer(csv_file)
            if self.test_lin:
                test = "linear"
            else:
                test = "angular"
            csv_writer.writerow([self.test_num,test,rospy.Time.now(),self.Pose[0]-self.origin[0], self.Pose[1]-self.origin[1], self.Pose[2]-self.origin[2]])
            #each row has test_num, test_type, time, X position, y position, z position

    self.time_start_test = 0
    def run(self):
        while not rospy.is_shutdown():
            if not self.man_toggle and self.test_active:
                self.real_x = self.Pose[0]-self.origin[0]
                self.real_z = self.Pose[2]-self.origin[2]
                if not self.test_lin: # testing Angular

                else: # testing lin target = 1 m
                    self.error_lin
                    if self.error_lin != 1: #go forward
                        self.linear
                        self.vel.linear.x = self.speed*((self.Kp_lin*(1-self.pose[0]+self.origin)[0])+(self.Ki_lin*(1-self.pose[0]+self.origin[0]))+(self.Kd_lin*(1-self.pose[0]+self.origin[0])))
                        self.vel.angular.z = 0
            #print(self.pose[2])
            self.velocityPublisher.publish(self.vel)
if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    PID_Test = PID_Test()
    thread.start_new_thread(input_thread, (PID_Test, ))
    try:
        PID_Test.run()
    except KeyboardInterrupt:
        PID_Test.vel.linear.x = 0
        PID_Test.vel.angular.z = 0
        PID_Test.velocityPublisher.publish(PID_Test.vel)
