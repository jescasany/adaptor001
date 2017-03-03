#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 02 2017

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
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle

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
        # Note: when the graph appears, you must close it first, and then the graph will reopen
        # and display the data as it comes in.
        x = []
        x = range(len(y))
        pylab.figure(name)
        pylab.clf()
        pylab.plot(x,y)
        pylab.draw()
        pylab.show()
        #raw_input("Press a key to continue...")
        
    def reg(self, y):
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
        coefficients = self.reg(y)
        # approximate wall_angle (degrees)
        black_board.right_wall_angle = normalize_angle(math.atan(coefficients[0]))*360./(2*math.pi)
        # approximate dist to wall (meters)
        black_board.distance_to_right_wall = coefficients[1]
        
    def left_wall_param(self, y):
        y = y[438:639]
        coefficients = self.reg(y)
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
black_board.adv_distance = 1.5      # meters
black_board.adv_angle = 0.0     # radians

black_board.distance_to_right_wall = 0.0
black_board.distance_to_left_wall = 0.0
black_board.distance_to_front = 7.0
black_board.Front = False
black_board.Front_All = False
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

        MOVE_ADV = CallbackTask("move_adv", self.move_adv)
        
        I_F_IS_VISITED =IgnoreFailure("I_F IS VISITED")
        
        IS_VISITED = CallbackTask("is_visited", self.is_visited)
        
        IS_POSSIBLE = Sequence("IS_POSSIBLE")
        
        RIGHT_SENSING = CallbackTask("right sensing", self.right_status)
        FRONT_FREE = CallbackTask("front free", self.front_free)
        
        SINGULARITIES = Selector("SINGULARITIES")
        
        LONG_RIGHT = Selector("LONG_RIGHT")
        SHORT_RIGHT = Selector("SHORT_RIGHT")
        
        SAME = CallbackTask("right sensing -- left appears", self.same)
        
        SMALL_CROSS = CallbackTask("nothing on right -- left appears", self.small_cross)
        FRONT_LEFT_RIGHT = CallbackTask("nothing on right -- right and left appear", self.front_left_right)
        FRONT_RIGHT = CallbackTask("nothing on right -- right appears", self.front_right)
        FRONT_ALL = CallbackTask("nothing on right -- all front appears", self.front_all)
        FRONT_NONE = CallbackTask("nothing on right nor front", self.front_none)
        
        CORNER = CallbackTask("CORNER", self.corner)
        
        BEHAVE.add_child(WALL_FOLLOW)
        
        WALL_FOLLOW.add_child(MOVE_ADV)
        WALL_FOLLOW.add_child(I_F_IS_VISITED)
        I_F_IS_VISITED.add_child(IS_VISITED)
        WALL_FOLLOW.add_child(IS_POSSIBLE)
        WALL_FOLLOW.add_child(SINGULARITIES)
        
        IS_POSSIBLE.add_child(RIGHT_SENSING)
        IS_POSSIBLE.add_child(FRONT_FREE)
        
        SINGULARITIES.add_child(LONG_RIGHT)
        SINGULARITIES.add_child(SHORT_RIGHT)
        SINGULARITIES.add_child(CORNER)
        
        LONG_RIGHT.add_child(SAME)
        
        SHORT_RIGHT.add_child(SMALL_CROSS)
        SHORT_RIGHT.add_child(FRONT_LEFT_RIGHT)
        SHORT_RIGHT.add_child(FRONT_RIGHT)
        SHORT_RIGHT.add_child(FRONT_ALL)
        SHORT_RIGHT.add_child(FRONT_NONE)
        
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
                return True
            
            
            (black_board.robot_position, black_board.robot_rotation) = advance(black_board.adv_distance, black_board.adv_angle)
            black_board.print_position()
#          raw_input("Press a key to continue...")
            black_board.waypoints.append((black_board.robot_position, black_board.robot_rotation))
            black_board.move_count += 1
            rospy.loginfo("move_adv done.")
        except:
            rospy.loginfo("move_adv failed.")
            return False
            
        return True
    
    def is_visited(self):
        if black_board.move_count == 0:
            rospy.loginfo("is not visited.")
            return False
        if (black_board.robot_position, black_board.robot_rotation) in black_board.waypoints[0:-1]:
            rospy.loginfo("is visited.")
            return True
        else:
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
    
    def front_free(self):
        self.laser_scan()
        rospy.sleep(2)
        y = list()
        y = black_board.kinect_scan
        #black_board.plotter(black_board.kinect_scan)
        black_board.filtered_scan = self.moving_window_filtro(y[200:438], 1)[0]
        # returns the reading has the discontinuity
        black_board.front_singularity = self.moving_window_filtro(y[200:438], 1)[1]
        black_board.front_singularity = [x+200 for x in black_board.front_singularity]
        print "front singularities: ", black_board.front_singularity
        singular_readings = []
        for i in black_board.front_singularity:
            singular_readings.append(y[i])
        print "front singular readings: ", singular_readings
        black_board.plotter(black_board.filtered_scan, "Front")
        #update average_distance_to_front
        black_board.average_distance_to_front = sum(y[200:438])/238.
        print "average distance to front: ", black_board.average_distance_to_front
        
        black_board.distance_to_obstacle = min(y[290:350])
        print "distance to obstacle: ", black_board.distance_to_obstacle
#        raw_input("Press a key to continue...")
        if black_board.distance_to_obstacle > black_board.adv_distance + 0.75:
            rospy.loginfo("Front is free.")
            black_board.Front = True    
        else:
            rospy.loginfo("Front is blocked.")
            black_board.Front = False
        return True
    
    def front_status(self):
        self.laser_scan()
        rospy.sleep(2)
        y = list()
        y = black_board.kinect_scan
        #black_board.plotter(black_board.kinect_scan)
        black_board.filtered_scan = self.moving_window_filtro(y[200:438], 1)[0]
        # returns the reading has the discontinuity
        black_board.front_singularity = self.moving_window_filtro(y[200:438], 1)[1]
        black_board.front_singularity = [x+200 for x in black_board.front_singularity]
        print "front singularities: ", black_board.front_singularity
        singular_readings = []
        for i in black_board.front_singularity:
            singular_readings.append(y[i])
        print "front singular readings: ", singular_readings
        black_board.plotter(black_board.filtered_scan, "Front")
        #update average_distance_to_front
        black_board.average_distance_to_front = sum(y[200:438])/238.
        print "average distance to front: ", black_board.average_distance_to_front
        
        tester = min(y[210:-1])
        if tester > black_board.adv_distance + 0.75:
            rospy.loginfo("Front is free.")
            black_board.Front_All = False
            black_board.adv_angle = 0.0
            return False
        else:
            rospy.loginfo("Front is blocked.")
            black_board.Front_All = True
            black_board.adv_angle = math.pi/2
            return True
        
    def right_status(self):
        self.laser_scan()
        rospy.sleep(2)
        y = list()
        y = black_board.kinect_scan
        #black_board.plotter(black_board.kinect_scan)
        black_board.filtered_scan = self.moving_window_filtro(y, 1)[0]
        # returns the reading has the discontinuity
        black_board.right_singularity = self.moving_window_filtro(y, 1)[1]
        print "right singularities: ", black_board.right_singularity
        black_board.plotter(black_board.filtered_scan, "Right")
        singular_readings = []
        for i in black_board.right_singularity:
            singular_readings.append(y[i])
        print "right singular readings: ", singular_readings
        black_board.right_wall_param(y)
        robot_rotation_angle = quat_to_angle(black_board.robot_rotation)
        print "odom_angle: ", math.degrees(normalize_angle(robot_rotation_angle))
        print "wall_angle: ", black_board.right_wall_angle
        #update average_distance_to_right_wall
        black_board.average_distance_to_right_wall = sum(y[0:200]) / 200.
        print "average distance to right wall: ", black_board.average_distance_to_right_wall
        print "distance to right wall(coefficients[1]): ", black_board.distance_to_right_wall
#        raw_input("Press a key to continue...")
        if black_board.distance_to_right_wall < 4.5:
            rospy.loginfo("Right sensing")
            black_board.Right = True
        else:
            rospy.loginfo("Nothing on the right")
            black_board.Right = False
        return True
    
    def left_status(self):
        self.laser_scan()
        rospy.sleep(2)
        y = list()
        y = black_board.kinect_scan
        #black_board.plotter(black_board.kinect_scan)
        black_board.filtered_scan = self.moving_window_filtro(y[438:-1], 1)[0]
        # returns the reading has the discontinuity
        black_board.left_singularity = self.moving_window_filtro(y[438:-1], 1)[1]
        black_board.left_singularity = [x+438 for x in black_board.left_singularity]
        print "left singularities: ", black_board.left_singularity
        black_board.plotter(black_board.filtered_scan, "Left")
        singular_readings = []
        for i in black_board.left_singularity:
            singular_readings.append(y[i])
        print "left singular readings: ", singular_readings
        black_board.left_wall_param(y)
        robot_rotation_angle = quat_to_angle(black_board.robot_rotation)
        print "odom_angle: ", math.degrees(normalize_angle(robot_rotation_angle)) 
        print "wall_angle: ", black_board.left_wall_angle
        #update average_distance_to_left_wall
        black_board.average_distance_to_left_wall = sum(y[438:-1])/200.
        print "average distance to left wall: ", black_board.average_distance_to_left_wall
        print "distance to left wall(coefficients[1]): ", black_board.distance_to_left_wall
#        raw_input("Press a key to continue...")
        if black_board.distance_to_left_wall < 4.5:
            rospy.loginfo("Left sensing")
            black_board.Left = True
        else:
            rospy.loginfo("Nothing on the left")
            black_board.Left = False
        return True
    
    def same(self):
        self.left_status()
        
        return True
    
    def small_cross(self):
        self.left_status()
        
        return True
    
    def front_left_right(self):
        self.left_status()
        
        return True
    
    def front_right(self):
        self.left_status()
        
        return True
    
    def front_all(self):
        self.left_status()
        
        return True
    
    def front_none(self):
        self.left_status()
        
        return True
    
    def corner(self):
        self.left_status()
        
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