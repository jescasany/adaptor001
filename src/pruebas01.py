#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 20 12:04:02 2017

@author: juan
"""

import pdb

import rospy
import numpy as np
import scipy.optimize as sci
import math
import pylab
import actionlib
from actionlib import GoalStatus
from geometry_msgs.msg import Pose, Twist, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from tf.transformations import quaternion_from_euler
from rbx2_msgs.srv import *      # just set battery level message
from pi_trees_ros.pi_trees_ros import *
from task_setup import *
from advance import *
import sensor_msgs.msg
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle

# for fancy terminal displays
class bcolors:
    HEADER = '\033[7;1;33m'
    OKBLUE = '\033[94m'
    OKRED = '\033[31;1m'
    OKGREEN = '\033[1;92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    REVERSE = '\033[7m'

# A class to track black_board.variables
class BlackBoard():
    def __init__(self):
        
        # The robot's current position on the map
        self.robot_position = Point()
        self.robot_rotation = Quaternion()
        self.robot_pose = (Point(), Quaternion())
        # Create a dictionary to hold navigation waypoints
        self.waypoints = list()
        
        # Initialize the patrol counter
        self.move_count = 0
        
    def print_position(self):
        print self.move_count
        print self.robot_position
        print self.robot_rotation
    
    def plotter(self, y, name):    
        # Uncomment these next few lines to display a constantly updating graph of data.
        # Note: when the graph appears, you must close it first, and then the graph will 
        # reopen and display the data as it comes in.
        x = []
        x = range(len(y))
        pylab.figure(name)
        pylab.clf()
        pylab.plot(x,y)
        pylab.draw()
        pylab.show()
        #raw_input("Press a key to continue...")
        
    def regression(self, y):
        x = range(len(y))
        x = np.asarray(x)
        y = np.asarray(y)
        fitfunc = lambda p, x: p[0] * x + p[1]
        errfunc = lambda p, x, y: fitfunc(p, x) - y
        guess = (0, 0)
        coefficients, success = sci.leastsq(errfunc, guess[:], args = (x, y), maxfev = 10000)
        return coefficients
        
    def right_wall_param(self, y):
        y = y[0:200]
        coefficients = self.regression(y)
        # approximate wall_angle (degrees)
        black_board.right_wall_angle = normalize_angle(math.atan(coefficients[0]))*360./(2*math.pi)
        # approximate dist to wall (meters)
        black_board.distance_to_right_wall = coefficients[1]
        
    def left_wall_param(self, y):
        y = y[438:639]
        coefficients = self.regression(y)
        # approximate wall_angle (degrees)
        black_board.left_wall_angle = normalize_angle(math.atan(coefficients[0]))*360./(2*math.pi)
        # approximate dist to wall (meters)
        black_board.distance_to_left_wall = coefficients[1]

# Initialize the blackboard
black_board = BlackBoard()
# Initialize a number of variables for the blackboard
black_board.kinect_scan = list()
black_board.filtered_scan = list()
# Initialize distance and angle to advance
black_board.adv_distance = 1.0      # meters
black_board.adv_angle = 0.0     # radians

black_board.driving_forward = True

black_board.distance_to_right_wall = 1.5
black_board.distance_to_left_wall = 5.0
black_board.distance_to_front = 7.0
black_board.Front = False
black_board.Front_All = False
black_board.Left = False
black_board.Right = False
black_board.Right_Corner = False
black_board.Right_Length = 200
black_board.Left_Length = 200
(black_board.robot_position, black_board.robot_rotation) = black_board.robot_pose 
black_board.odom_angle = 0.0

black_board.chs = 1     # used to change the sign(+/-) of the PI/2 turn

class Pruebas():
    def __init__(self):
        pdb.set_trace()
        rospy.init_node("pruebas_tree")

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # Set the low battery threshold (between 0 and 100)
        self.low_battery_threshold = rospy.get_param('~low_battery_threshold', 30)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait up to 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))    
        
        rospy.loginfo("Connected to move_base action server")
        
        # Create the root node
        BEHAVE = Sequence("BEHAVE")

        MOVE_AVOID = Sequence("MOVE AVOID")
        
        BEHAVE.add_child(MOVE_AVOID)
        
        MOVE_ADV = CallbackTask("Move_Adv", self.move_adv)
        
        MOVE_AVOID.add_child(MOVE_ADV)
        
        
        
        
         # Display the tree before beginning execution
        print bcolors.HEADER + "Pruebas Behavior Tree" + bcolors.ENDC
        print_tree(BEHAVE, indent=0, use_symbols=True)
        print_dot_tree(BEHAVE, dotfilepath='/home/juan/catkin_ws/src/adaptor001/tree.dot')
        # Run the tree
        while not rospy.is_shutdown():
            BEHAVE.run()
            rospy.sleep(0.1)
            
    def move_adv(self):
        rospy.loginfo("Estoy en move advance")
        #pdb.set_trace()
        try:
            raw_input("Press a key to continue...")
            if black_board.move_count == 0:
                # black_board.robot_pose is a tuple (position, rotation)
                (black_board.robot_position, black_board.robot_rotation) = advance(0.0, 0.0)
                black_board.print_position()
                black_board.waypoints.append((black_board.robot_position, black_board.robot_rotation))
                black_board.move_count += 1
                rospy.loginfo("move_adv done.")
                return 1
            angle = math.pi
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            
            x = black_board.robot_position.x + black_board.distance_to_right_wall * math.tan(math.pi/3)

            y = black_board.robot_position.y + black_board.distance_to_right_wall
            
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = Pose(Point(x, y, 0.0), q)
            
            move_adv_task = SimpleActionTask("MOVE_ADV", "move_adv_task", MoveBaseAction, goal, reset_after=False)
            
            (black_board.robot_position, black_board.robot_rotation) = advance(0.0, 0.0)
            black_board.print_position()
#            raw_input("Press a key to continue...")
            black_board.waypoints.append((black_board.robot_position, black_board.robot_rotation))
            black_board.move_count += 1
            rospy.loginfo("move_adv done.")
        except:
            rospy.loginfo("move_adv failed.")
            return 0
        return 1
            
    def laser_scan(self):
        rospy.loginfo("Waiting for /base_scan topic...")
        rospy.wait_for_message('/base_scan', sensor_msgs.msg.LaserScan)
        # Subscribe the /base_scan topic to get the range readings  
        rospy.Subscriber('/base_scan', sensor_msgs.msg.LaserScan, self.scan_callback, queue_size = 10)
        rospy.loginfo("laser_scan done")
        
    def clamp(self, val, minimum, maximum):
        if val < minimum:
            return minimum
        elif val > maximum:
            return maximum
        else:
            return val

    def round_to_zero(self, val, tolerance):
        if -1 * tolerance < val < tolerance:
            return 0
        else:
            return val
    
    def scan_callback(self, msg):
        black_board.kinect_scan = list(msg.ranges) # transform to list since ranges is a tuple
    
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    tree = Pruebas()