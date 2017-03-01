#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 16 11:59:11 2017

@author: juan
"""
import pdb
import rospy
import numpy as np
import scipy.optimize as sci
import math
import pylab
from geometry_msgs.msg import Twist, Point, Quaternion
from rbx2_msgs.srv import *
from pi_trees_ros.pi_trees_ros import *
from rbx2_tasks.task_setup import *
from advance import *
import sensor_msgs.msg
from rbx1_nav.transform_utils import quat_to_angle

# A class to track black_board.variables
class BlackBoard():
    def __init__(self):
        
        # The robot's current position on the map
        self.robot_pose = (Point(), Quaternion())
        self.robot_position = Point()
        self.robot_rotation = Quaternion()
        
        # Create a dictionary to hold navigation waypoints
        self.waypoints = list()
        
        # Initialize the patrol counter
        self.move_count = 0
        
    def print_position(self):
        print self.move_count
        print self.robot_position
        print self.robot_rotation
    
    def plotter(self, y):    
        # Uncomment these next few lines to display a constantly updating graph of data.
        # Note: when the graph appears, you must close it first, and then the graph will reopen
        # and display the data as it comes in.
        x = []
        x = range(len(y))
        pylab.figure(1)
        pylab.clf()
        pylab.plot(x,y)
        pylab.draw()
        pylab.show()
        #raw_input("Press a key to continue...")
        
    def wall_ang(self, y):
        x = []
        y = y[0:200]
        x = range(len(y))
        x = np.asarray(x)
        y = np.asarray(y)
        fitfunc = lambda p, x: p[0] * x + p[1]
        errfunc = lambda p, x, y: fitfunc(p, x) - y
        guess = (0, 0)
        coefficients, success = sci.leastsq(errfunc, guess[:], args = (x, y), maxfev = 10000)
        # update wall_angle
        black_board.wall_angle = math.atan(coefficients[0])*360./(2*math.pi)
        

# Initialize the black board
black_board = BlackBoard()
# Initialize a list to hold the waypoint poses
black_board.kinect_scan = list()
black_board.filtered_scan = list()

black_board.distance_to_wall = 0.0
black_board.Front = False
black_board.Back = False
black_board.Left = False
black_board.Right = False
(black_board.robot_position, black_board.robot_rotation) = black_board.robot_pose 
black_board.odom_angle = 0.0

class Pruebas():
    def __init__(self):
        pdb.set_trace()
        rospy.init_node("pruebas_tree")

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # Create the root node
        BEHAVE = Sequence("BEHAVE")
        
        WALL_FOLLOW = Sequence("WALL_FOLLOW")

        MOVE_ADV = CallbackTask("move_3", self.move_adv)
        
        IS_VISITED = CallbackTask("is_visited", self.is_visited)
        
        IS_POSSIBLE = CallbackTask("is_possible", self.is_possible)
        
        BEHAVE.add_child(WALL_FOLLOW)
        
        WALL_FOLLOW.add_child(MOVE_ADV)
        WALL_FOLLOW.add_child(IS_VISITED)
        WALL_FOLLOW.add_child(IS_POSSIBLE)
        
        # Display the tree before beginning execution
        print "Patrol Behavior Tree"
        print_tree(BEHAVE)
        
        # Run the tree
        while not rospy.is_shutdown():
            BEHAVE.run()
            rospy.sleep(0.1)
        
    def move_adv(self):
        try:
            raw_input("Press a key to continue...")
            if black_board.move_count == 0:
                # black_board.robot_pose is a tuple (position, rotation)
                (black_board.robot_position, black_board.robot_rotation) = advance(0.0, 0.0)
                black_board.print_position()
                black_board.waypoints.append((black_board.robot_position, black_board.robot_rotation))
                black_board.move_count += 1
                rospy.loginfo("move_adv done.")
            adv_distance = 1.5
            (black_board.robot_position, black_board.robot_rotation) = advance(adv_distance, 0.0)
            black_board.print_position()
#          raw_input("Press a key to continue...")
            black_board.waypoints.append((black_board.robot_position, black_board.robot_rotation))
            black_board.move_count += 1
            rospy.loginfo("move_adv done.")
        except:
            rospy.loginfo("move_adv failed.")
            
        return True
    
    def is_visited(self):
        if (black_board.robot_position, black_board.robot_rotation) in black_board.waypoints:
            return True
        rospy.loginfo("is not visited.")
        return False
    
    def formule(self, L):
        f = 0
        tolerance = 0.2
        f = L[1] - (L[0] + L[2])/2
        f = self.round_to_zero(f, tolerance)
        return f
        
    def moving_window_filtro(self, x, n_neighbors=1):
        n = len(x)
        width = n_neighbors*2 + 1
        x = [x[0]]*n_neighbors + x + [x[-1]]*n_neighbors
        # To complete the function,
        # return a list of the filtered values from i to i+width for all values i from 0 to n-1.
        filtro = []
        singularity = []
        for i in range(n):
            fi = self.formule(x[i:i+width])
            if fi > 0:
                filtro.append(fi)
                singularity.append(i)
            else:
                filtro.append(0.0)
            
        return filtro, singularity
    
    def is_possible(self):
        self.laser_scan()
        rospy.sleep(2)
        y = list()
        y = black_board.kinect_scan
        #black_board.plotter(black_board.kinect_scan)
        black_board.filtered_scan = self.moving_window_filtro(y, 1)[0]
        black_board.singularity = self.moving_window_filtro(y, 1)[1]
        print "singularity: ", black_board.singularity
        black_board.plotter(black_board.filtered_scan)
        black_board.wall_ang(y)
        black_board.robot_rotation = quat_to_angle(black_board.robot_rotation)
        print "odom_angle: ", math.degrees(black_board.robot_rotation) 
        print "wall_angle: ", black_board.wall_angle
        #update distanceToWall
        black_board.distance_to_wall = sum(y) / float(len(y))
        print "distance_to_wall: ", black_board.distance_to_wall
        
#        raw_input("Press a key to continue...")
        
        return True
    
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
        black_board.kinect_scan = list(msg.ranges)
    
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    tree = Pruebas()