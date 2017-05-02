#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 02 2017
Last visited on 18/04/2017
@author: juan

The Enactive Cognitive Architecture (ECA)
based on the paper of Georgeon, Marshall, and Manzotti (2013). ECA: An enactivist cognitive
architecture based on sensorimotor modeling. Biologically Inspired Cognitive
Architectures, 6:46-57.

Implemented following the Behavior Trees model and the enactive agents code from Katja Abramova.
"""
__author__ = 'juan'

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

import argparse
import abc
from json import loads, dumps
from collections import OrderedDict, Counter
from experiment import *
from result import Result
from anticipation import *
import random

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
    
# to translate from 'e#r#' to clear 'meaning'
class Decode:
    def __init__(self, raw):
        #pdb.set_trace()
        self.raw = raw
    
    def get_translation(self):
        #pdb.set_trace()
        self.raw = self.raw.replace('e1r10','move forward fail')
        self.raw = self.raw.replace('e1r1','move forward wall')
        self.raw = self.raw.replace('e1r4','move forward no wall')
        self.raw = self.raw.replace('e2r2','turn left')
        self.raw = self.raw.replace('e3r3','turn right')
        self.raw = self.raw.replace('e4r4','front free')
        self.raw = self.raw.replace('e4r5','front busy')
        self.raw = self.raw.replace('e5r6','right1 sensing')
        self.raw = self.raw.replace('e5r8','right2 sensing')
        self.raw = self.raw.replace('e5r12','right3 sensing')
        self.raw = self.raw.replace('e5r14','nothing on right1')
        self.raw = self.raw.replace('e6r7','left1 sensing')
        self.raw = self.raw.replace('e6r9','left2 sensing')
        self.raw = self.raw.replace('e6r11','left3 sensing')
        self.raw = self.raw.replace('e6r13','nothing on left1')
        self.raw = self.raw.replace('e1','move forward')
        self.raw = self.raw.replace('e2','turn left')
        self.raw = self.raw.replace('e3','turn right')
        self.raw = self.raw.replace('e4','front?')
        self.raw = self.raw.replace('e5','right?')
        self.raw = self.raw.replace('e6','left?')
        return self.raw


""" A class to track black_board.variables """
class BlackBoard:
    def __init__(self):
        # The agent's current position on the map
        self.agent_position = Point()
        self.agent_rotation = Quaternion()
        self.agent_pose = (Point(), Quaternion())
        # Create a dictionary to hold navigation waypoints
        self.waypoints = list()
        
        # Initialize the patrol counter
        self.move_count = 0
        # initialize one simulation step (that might consist of several primitive steps)
        self.sim_step = 1
        
    def print_position(self):
        print self.move_count
        print self.agent_position
        print self.agent_rotation
    
    def plotter(self, y, name):    
        # Uncomment these next few lines to display a constantly updating graph of data.
        # Note: when the graph appears, you must close it first, and then the graph will 
        # reopen and display the data as it comes in.
        x = []
        x = range(len(y))
        pylab.figure(name)
        pylab.clf()
        pylab.plot(x,y, color = 'black')
        pylab.plot(x,y, 'bo')
        pylab.draw()
        pylab.show()
        #raw_input("Press a key to continue...")
        
    def regression(self, y):
        # data provided as numpy arrays
        x = range(len(y))
        x = np.asarray(x)
        y = np.asarray(y)
        # here, create lambda function for Line fit
        # p is a tuple that contains the parameters of the fit
        fitfunc = lambda p, x: p[0] * x + p[1]
        # errfunc is the diference between the fitfunc and the y "experimental" data
        errfunc = lambda p, x, y: fitfunc(p, x) - y
        # guess contains the "first guess" of the parameters
        guess = (0, 0)
        # leastsq finds the set of parameters in the tuple p that minimizes
        # errfunc = yfit - yExperimental
        coefficients, success = sci.leastsq(errfunc, guess[:], args = (x, y), maxfev = 10000)
        return coefficients
        
    def right_wall_param(self, y):
        #pdb.set_trace()
        coefficients = self.regression(y)
        # approximate wall_angle (degrees)
        black_board.right_wall_angle = normalize_angle(math.atan(coefficients[0]))*360./(2*math.pi)
        # approximate dist to wall (meters)
        if coefficients[1] < 5.0:
            black_board.distance_to_right_wall = coefficients[1]
        else:
            black_board.distance_to_right_wall = coefficients[1]
        
    def left_wall_param(self, y):
        coefficients = self.regression(y)
        # approximate wall_angle (degrees)
        black_board.left_wall_angle = normalize_angle(math.atan(coefficients[0]))*360./(2*math.pi)
        # approximate dist to wall (meters)
        if coefficients[1] < 5.0:
            black_board.distance_to_left_wall = coefficients[1]
        else:
            black_board.distance_to_left_wall = coefficients[1]
            
    def laser_scan(self):
        #rospy.loginfo("Waiting for /base_scan topic...")
        rospy.wait_for_message('/base_scan', sensor_msgs.msg.LaserScan)
        # Subscribe the /base_scan topic to get the range readings  
        rospy.Subscriber('/base_scan', sensor_msgs.msg.LaserScan, self.scan_callback, queue_size = 10)
        rospy.sleep(0.1)
        black_board.distance_front = min(black_board.kinect_scan[300:338])
        if black_board.distance_front <= 1.7:
            black_board.driving_forward = False
        else:
            if black_board.Right1:
                black_board.adv_distance = black_board.distance_front - 1.7
            elif black_board.Right2 or black_board.Right3:
                black_board.adv_distance = 1.0
            else:
                black_board.adv_distance = 0.0
                
            black_board.driving_forward = True
        #rospy.loginfo("laser_scan done")
        
    def scan_callback(self, msg):
        black_board.kinect_scan = list(msg.ranges) # transformed to list since msg.ranges is a tuple
        
    def formule(self, L, tolerance):
        f = 0
        f = L[1] - (L[0] + L[2])/2
        f = self.round_to_zero(f, tolerance)
        return f
        
    def moving_window_filtro(self, x, tolerance=0.2, n_neighbors=1):
        n = len(x)
        width = n_neighbors*2 + 1
        x = [x[0]]*n_neighbors + x + [x[-1]]*n_neighbors
        # To complete the function,
        # return a list of the filtered values from i to i+width for all values i from 0 to n-1.
        filtro = []
        singularity = []
        for i in range(n):
            fi = abs(self.formule(x[i:i+width], tolerance))
            filtro.append(fi)
            if fi != 0.0:
                singularity.append(i)
            
        return filtro, singularity
    
    def round_to_zero(self, val, tolerance):
        if -1 * tolerance < val < tolerance:
            return 0
        else:
            return val
        
    def arrange(self):
        # makes the robot turn to adopt a rotation angle of 0, +-(pi/2) or pi 
        agent_rotation_angle = math.degrees(normalize_angle(quat_to_angle(black_board.agent_rotation)))
        # pdb.set_trace()
        angle_rad_rot = math.radians(agent_rotation_angle)
        if abs(agent_rotation_angle) < 45:
            turn = -angle_rad_rot
            print agent_rotation_angle, degrees(turn)
            (black_board.agent_position, black_board.agent_rotation) = advance(0.0, turn, da=True)
        if abs(agent_rotation_angle) < 90 and abs(agent_rotation_angle) >= 45:
            turn = self.sign(angle_rad_rot)*abs(math.pi/2 - abs(angle_rad_rot))
            print agent_rotation_angle, degrees(turn)
            (black_board.agent_position, black_board.agent_rotation) = advance(0.0, turn, da=True)
        if abs(agent_rotation_angle) < 135 and abs(agent_rotation_angle) >= 90:
            turn = -self.sign(angle_rad_rot)*abs(math.pi/2 - abs(angle_rad_rot))
            print agent_rotation_angle, degrees(turn)
            (black_board.agent_position, black_board.agent_rotation) = advance(0.0, turn, da=True)
        if abs(agent_rotation_angle) >= 135:
            turn = self.sign(angle_rad_rot)*abs(math.pi - abs(angle_rad_rot))
            print agent_rotation_angle, degrees(turn)
            (black_board.agent_position, black_board.agent_rotation) = advance(0.0, turn, da=True)
        agent_rotation_angle = math.degrees(normalize_angle(quat_to_angle(black_board.agent_rotation)))
        return agent_rotation_angle
        
    def move_adv(self):
        rospy.loginfo("Estoy en move advance")
        #pdb.set_trace()
        try:
            #raw_input("Press a key to continue...")
            if black_board.move_count == 0:
                print bcolors.OKGREEN + "PREPARING THINGS IN MY PLACE" + bcolors.ENDC
                # black_board.agent_pose is a tuple (position, rotation)
                (black_board.agent_position, black_board.agent_rotation) = advance(0.0, 0.0, da=True)
                black_board.print_position()
                black_board.waypoints.append((black_board.agent_position, black_board.agent_rotation))
                black_board.move_count += 1
                rospy.loginfo("move_adv done.")
                return 1
            
            self.laser_scan()
            if black_board.driving_forward or black_board.adv_distance == 0.0:
                if abs(black_board.adv_angle) < radians(2):    
                    (black_board.agent_position, black_board.agent_rotation) = advance(black_board.adv_distance, 0.0, da=True)
                else:
                    (black_board.agent_position, black_board.agent_rotation) = advance(black_board.adv_distance, black_board.adv_angle, da=True)
                black_board.adv_angle = 0.0
                black_board.print_position()
    #            raw_input("Press a key to continue...")
                black_board.waypoints.append((black_board.agent_position, black_board.agent_rotation))
                black_board.move_count += 1
                rospy.loginfo("move_adv done.")
                black_board.move_fail = False
            else:
                rospy.loginfo("move_adv failed.")
                (black_board.agent_position, black_board.agent_rotation) = advance(0.0, 0.0, da=True)
                black_board.move_fail = True
        except:
            rospy.loginfo("move_adv failed.")
            (black_board.agent_position, black_board.agent_rotation) = advance(0.0, 0.0, da=True)
            black_board.move_fail = True
            return 1
        return 1
    
    def right_status(self):
        rospy.loginfo("Estoy en right status")
        
        self.laser_scan()
        rospy.sleep(1)
        if black_board.distance_front < 2.5:
            (black_board.agent_position, black_board.agent_rotation) = advance(0.0, math.pi/2, da=True)
            self.laser_scan()
            rospy.sleep(1)
        y = list()
        y = black_board.kinect_scan
        
        #black_board.plotter(y[0:200], "Right")
        
        black_board.filtered_scan, singularities = self.moving_window_filtro(y[0:200], tolerance=0.1, n_neighbors=1)
        print singularities
#        if len(singularities) != 0:
#            black_board.plotter(black_board.filtered_scan, "Right-filtered")
        
        y1 = y[0:65]
        y2 = y[65:130]
        y3 = y[130:200]
        
        black_board.right_wall_param(y1)
        
        agent_rotation_angle = self.arrange() 
        
        print "odom_angle: ", agent_rotation_angle
        print "wall_angle: ", black_board.right_wall_angle
        # update average_distance_to_right_wall
        black_board.average_distance_to_right_wall = sum(y1)/65.
        print "average distance to right wall: ", black_board.average_distance_to_right_wall
        print bcolors.OKGREEN + "distance to right wall(coefficients[1]): " +  str(black_board.distance_to_right_wall) + bcolors.ENDC
        print bcolors.OKGREEN + "distance to front obstacle: " +  str(black_board.distance_front) + bcolors.ENDC
        #raw_input("Press ENTER to continue...")
        if black_board.distance_to_right_wall < 4.5:
            rospy.loginfo(bcolors.OKGREEN + "Right1 sensing" + bcolors.ENDC)
            black_board.Right1 = True
            black_board.adv_distance = black_board.distance_front - 1.7            
            #pdb.set_trace()
        else:
            rospy.loginfo(bcolors.OKGREEN + "Nothing on the right1" + bcolors.ENDC)
            black_board.adv_distance = 0.0
            black_board.Right1 = False
            #black_board.Right_Corner = False
            
        av_distance2 = sum(y2)/65.
        if av_distance2 < 4.5:
            rospy.loginfo(bcolors.OKGREEN + "Right2 sensing" + bcolors.ENDC)
            black_board.Right2 = True
        else:
            rospy.loginfo(bcolors.OKGREEN + "Nothing on the right2" + bcolors.ENDC)
            black_board.Right2 = False
            
        av_distance3 = sum(y3)/70.
        if av_distance3 < 4.5:
            rospy.loginfo(bcolors.OKGREEN + "Right3 sensing" + bcolors.ENDC)
            black_board.Right3 = True
        else:
            rospy.loginfo(bcolors.OKGREEN + "Nothing on the right3" + bcolors.ENDC)
            black_board.Right3 = False
        
        return 1
    
    def left_status(self):
        rospy.loginfo("Estoy en left status")
        self.laser_scan()
        rospy.sleep(2)
        
        y = list()
        y = black_board.kinect_scan
        #black_board.plotter(black_board.kinect_scan)
        
        y1 = y[574:639]
        y2 = y[508:574]
        y3 = y[438:508]
        
        black_board.right_wall_param(y1)
        agent_rotation_angle = self.arrange()  
        print "odom_angle: ",  math.degrees(normalize_angle(agent_rotation_angle))
        print "wall_angle: ", black_board.left_wall_angle
        #update average_distance_to_left_wall
        black_board.average_distance_to_left_wall = sum(y1)/65.
        print "average distance to left wall: ", black_board.average_distance_to_left_wall
        print "distance to left wall(coefficients[1]): ", black_board.distance_to_left_wall
        print bcolors.OKGREEN + "distance to front obstacle: " +  str(black_board.distance_front) + bcolors.ENDC
#        raw_input("Press a key to continue...")
        if black_board.distance_to_left_wall < 4.5:
            rospy.loginfo(bcolors.OKGREEN + "Left1 sensing" + bcolors.ENDC)
            black_board.Left1 = True
            if black_board.Rigth1:
                black_board.adv_distance = black_board.distance_front - 1.7
            else:
                black_board.adv_distance = 0.0
        else:
            rospy.loginfo(bcolors.OKGREEN + "Nothing on the left1" + bcolors.ENDC)
            black_board.Left1 = False

        av_distance2 = sum(y2)/65.
        if av_distance2 < 4.5:
            rospy.loginfo(bcolors.OKGREEN + "Left2 sensing" + bcolors.ENDC)
            black_board.Left2 = True
        else:
            rospy.loginfo(bcolors.OKGREEN + "Nothing on the left2" + bcolors.ENDC)
            black_board.Left2 = False
            
        av_distance3 = sum(y3)/70.
        if av_distance3 < 4.5:
            rospy.loginfo(bcolors.OKGREEN + "Left3 sensing" + bcolors.ENDC)
            black_board.Left3 = True
        else:
            rospy.loginfo(bcolors.OKGREEN + "Nothing on the left3" + bcolors.ENDC)
            black_board.Left3 = False
            
        return 1
    
    def sign(self, x):
        if x < 0.0:
            return -1
        elif x == 0.0:
            return 0
        elif x > 0.0:
            return 1


# Initialize the blackboard
black_board = BlackBoard()
# Initialize a number of variables for the blackboard
black_board.kinect_scan = list()
black_board.filtered_scan = list()
# Initialize distance and angle to advance
black_board.adv_distance = 1.0      # meters
black_board.adv_angle = 0.0     # radians

black_board.driving_forward = True      # is True if there is no obstacle ahead 
black_board.move_fail = False

black_board.distance_to_right_wall = 1.5
black_board.last_distance = 1.5
black_board.distance_to_left_wall = 5.0
black_board.left_wall_angle = 0.0
black_board.distance_to_front = 7.0
black_board.Front = False
black_board.Front_All = False
black_board.Left1 = False
black_board.Left2 = False
black_board.Left3 = False
black_board.Right1 = False
black_board.Right2 = False
black_board.Right3 = False
black_board.Right_Corner = False
black_board.Right_Length = 200
black_board.Left_Length = 200
(black_board.agent_position, black_board.agent_rotation) = black_board.agent_pose 
black_board.odom_angle = 0.0

black_board.chs = 1     # used to change the sign(+/-) of the PI/2 turn
black_board.agent_mechanism = ''    # to choose among simple, recursive and constructive mechanisms
black_board.process_boredom = False


class EcaAgent02:
    INTERACTION_ENACTION_HISTORY_SIZE = 50
    def __init__(self):
        #pdb.set_trace()
        rospy.init_node("eca_agent01_tree")
        # Set the shutdown function (stop the agent)
        rospy.on_shutdown(self.shutdown)
        # Publisher to manually control the agent (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        rate = rospy.Rate(10)

         # initialize existence
        black_board.ex = None
        # initialize primitive interactions
        primitive_interactions = {"move forward wall": ("e1", "r1", 50),\
                                  "move forward no wall": ("e1", "r4", -20),\
                                  "move forward fail": ("e1", "r10", -50),\
                                  "turn left": ("e2", "r2", 15),\
                                  "turn right": ("e3", "r3", 25),\
#                                  "front free": ("e4", "r4", 1),\
#                                  "front busy": ("e4", "r5", -2),\
                                  "right1 sensing": ("e5", "r6", 20),\
                                  "right2 sensing": ("e5", "r8", 10),\
                                  "right3 sensing": ("e5", "r12", 10),\
                                  "nothing on right1": ("e5", "r14", -50),\
                                  "left1 sensing": ("e6", "r7", 0),\
                                  "left2 sensing": ("e6", "r9", 0),\
                                  "left3 sensing": ("e6", "r11", 0),\
                                  "nothing on left1": ("e6", "r13", 0)
}
        # initialize environments and existences
        self.mechanism = black_board.agent_mechanism
        if self.mechanism == "simple":
            black_board.environment = Environment()
            black_board.ex = Existence(primitive_interactions, black_board.environment)
        elif self.mechanism == "recursive":
            black_board.environment = Environment()
            black_board.ex = RecursiveExistence(primitive_interactions, black_board.environment)
        elif self.mechanism == "constructive":
            black_board.environment = ConstructiveEnvironment()
            black_board.ex = ConstructiveExistence(primitive_interactions, black_board.environment)
        # Create the root node
        ECAAGENT02 = Sequence("ECAAGENT02")
        
        START_STEP = CallbackTask("START STEP", black_board.ex.step)
        
        I_F_IS_VISITED =IgnoreFailure("I_F IS VISITED")
        
        IS_VISITED = CallbackTask("is visited", self.is_visited)
        
        ECAAGENT02.add_child(START_STEP)
        ECAAGENT02.add_child(I_F_IS_VISITED)
        
        # Display the tree before beginning execution
        print bcolors.HEADER + "ECAAGENT02 Behavior Tree" + bcolors.ENDC
        print_tree(ECAAGENT02, indent=0, use_symbols=True)
        print_dot_tree(ECAAGENT02, dotfilepath='/home/juan/catkin_ws/src/adaptor001/tree02.dot')
        
        # Run the tree
        while not rospy.is_shutdown():
            #pdb.set_trace()
            ECAAGENT02.run()
            decoded = Decode(black_board.step_trace)
            translated = decoded.get_translation()
            print bcolors.OKGREEN + str(black_board.sim_step) + " " +  str(translated) + bcolors.ENDC
            print "\n"
            
            #raw_input(bcolors.WARNING + "Press ENTER to continue..." + bcolors.ENDC)
            
            if len(black_board.ex.INTERACTIONS) >= self.INTERACTION_ENACTION_HISTORY_SIZE:
                black_board.ex.INTERACTIONS.popitem(last=False)
                
            if black_board.sim_step >= 3:
                black_board.process_boredom = True
            black_board.sim_step += 1
            rate.sleep()

    def front_free(self):
        #pdb.set_trace()
        rospy.loginfo("Estoy en front free")
#        self.laser_scan()   # updates black_board.kinect_scan (ranges)
#        rospy.sleep(2)
        black_board.distance_to_obstacle = 5.0
        tolerance = 0.2
        y = list()
        y = black_board.kinect_scan
        
        black_board.filtered_scan = self.moving_window_filtro(y[290:350], tolerance, n_neighbors=1)[0]

        #black_board.plotter(black_board.filtered_scan, "Front")
        
        # returns the readings having the discontinuity
        front_singularity = self.moving_window_filtro(y[290:350], tolerance, n_neighbors=1)[1]
        black_board.distance_to_obstacle = min(y[290:350])
        #update average_distance_to_front
        average_distance_to_front = sum(y[290:350])/60.
        print "average distance to front: ", average_distance_to_front
        #black_board.distance_to_obstacle = average_distance_to_front
        print bcolors.OKGREEN + "distance to obstacle: " +  \
            str(black_board.distance_to_obstacle) + bcolors.ENDC
#        raw_input("Press a key to continue...")
        if black_board.distance_to_obstacle > black_board.adv_distance:
            print bcolors.OKGREEN + "FRONT FREE" + bcolors.ENDC
            black_board.adv_distance = 1.0
            if black_board.Right_Corner == False:
                black_board.adv_angle = 0.0
            black_board.driving_forward = True
            black_board.Front = True
            return 1
        else:
            print bcolors.OKRED + "OBSTACLE IN FRONT -> (+/-)PI/2" + bcolors.ENDC
            black_board.adv_distance = 0.0
            black_board.driving_forward = False
            
            black_board.chs *=-1
            black_board.Front = False
            return 0     
    
    def is_visited(self):
        if black_board.move_count == 0:
            rospy.loginfo("Waypoint is not visited.")
            return True
        if (black_board.agent_position, black_board.agent_rotation) in black_board.waypoints[0:-1]:
            rospy.loginfo("Waypoint is visited.")
            return True
        else:
            rospy.loginfo("Waypoint is not visited.")
            return True
    
    def front_status(self):
        #pdb.set_trace()
        rospy.loginfo("Estoy en front status")
        self.laser_scan()
        rospy.sleep(2)
        tester = 5.0
        tolerance = 0.1
        y = list()
        y = black_board.kinect_scan

        black_board.filtered_scan = self.moving_window_filtro(y[200:438], tolerance, n_neighbors=1)[0]
        
        #black_board.plotter(black_board.filtered_scan, "Front")        
        
        # returns the reading having the discontinuity
        front_singularity = self.moving_window_filtro(y[200:438], tolerance, n_neighbors=1)[1]
        tester = min(y[200:-1])
        if len(front_singularity) != 0:
            front_singularity = [x+200 for x in front_singularity]
            print "front singularities: ", front_singularity
            singular_readings = []
            for i in front_singularity:
                singular_readings.append(y[i])
            print "front singular readings: ", singular_readings
            tester = min(y[front_singularity[-1]:-1])
            if tester > black_board.adv_distance:
                print bcolors.OKGREEN + "Front is free." + bcolors.ENDC
                black_board.Front_All = True
                #black_board.adv_angle = 0.0
            else:
                print bcolors.OKGREEN + "Front is blocked -> PI/2" + bcolors.ENDC
                black_board.Front_All = False
                #black_board.adv_angle = math.pi/2
        else:
            #update average_distance_to_front
            average_distance_to_front = sum(y[200:438])/238.
            print "average distance to front: ", average_distance_to_front
            tester = average_distance_to_front
            if tester > black_board.adv_distance:
                print bcolors.OKGREEN + "Front is free." + bcolors.ENDC
                black_board.Front_All = True
                #black_board.adv_angle = 0.0
            else:
                print bcolors.OKGREEN + "Front is blocked -> PI/2" + bcolors.ENDC
                black_board.Front_All = False
                #black_board.adv_angle = math.pi/2
        return 2
             
    def clamp(self, val, minimum, maximum):
        if val < minimum:
            return minimum
        elif val > maximum:
            return maximum
        else:
            return val
    
    def shutdown(self):
        rospy.loginfo("Stopping the agent...")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
        

class Existence:
    """
    A class implementing the agent control-and-learning mechanism.
    The agent operates by executing and learning interactions, and is 
    motivated to perform those interactions that have positive valences.
    Interactions can be of two types. Primitive interactions are tuples of 
    (experiment, result, valence). Composite interactions are interactions 
    which consist of primitive interactions.
    When a given experiment is performed and a given result is obtained, 
    the corresponding interaction is considered enacted.
    """
    EXPERIMENTS = OrderedDict()
    INTERACTIONS = OrderedDict()
    RESULTS = OrderedDict()

    def __init__(self, primitive_interactions, environment):
        """
        Initialize existence with a set of primitive interactions and 
        environment.
        :param primitive_interactions: (dict) of primitive interactions 
        of the form {(str) interaction meaning: ((str) experiment, 
        (str) result, (int) valence)}
        :param environment: (Environment) that controls which results are 
        returned for a given primitive experiment
        :return: (Existence)
        """
        self.context_interaction = None
        self.mood = None
        self.environment = environment
        self.primitive_interactions = primitive_interactions
        self.initialize_interactions(primitive_interactions)
        
    def step(self):
        """
        Execute a single simulation step.
        :return: (str) performed interaction and mood
        """
        #pdb.set_trace
        print bcolors.OKGREEN + "Context: " + str(self.context_interaction) + bcolors.ENDC
        anticipations = self.anticipate()  # anticipate possible interactions
        experiment = self.select_experiment(anticipations)  # select the best experiment
        result_label = self.environment.return_result(experiment)  # consult the world and return result
        result = self.addget_result(result_label)  # add result to the dictionary
        enacted_interaction = self.get_interaction(experiment.get_label() + result.get_label())
        print bcolors.OKGREEN + "Enacted " + str(enacted_interaction) + bcolors.ENDC

        if enacted_interaction.get_valence() > 0:
            self.mood = 'HAPPY'
        else:
            self.mood = 'SAD'

        self.learn_composite_interaction(self.context_interaction, enacted_interaction)
        self.context_interaction = enacted_interaction

        black_board.step_trace = experiment.get_label() + result.get_label() + " " + self.mood
        
        return 1

    def initialize_interactions(self, primitive_interactions):
        """
        Add primitive interactions to existence
        :param primitive_interactions: a set of primitive interactions 
        provided as a dictionary
        {(str) interaction meaning: ((str) experiment, 
        (str) result, (int) valence)}
        """
        for interaction in primitive_interactions:
            meaning = interaction
            experiment_label = primitive_interactions[interaction][0]
            result_label = primitive_interactions[interaction][1]
            valence = primitive_interactions[interaction][2]
            result = self.addget_result(result_label)
            experiment = self.addget_experiment(experiment_label)
            self.addget_primitive_interaction(experiment, result, valence, meaning)

    def addget_primitive_interaction(self, experiment, result, valence=None, meaning=None):
        """
        If a primitive interaction is not in the INTERACTIONS dictionary, 
        add it. Otherwise just return it.
        :param experiment: (str) primitive experiment
        :param result: (str) primitive result
        :param valence: (int) valence of the interaction
        :param meaning: (str) observer's meaning of the interaction
        :return: (interaction) primitive interaction from the INTERACTIONS dictionary
        """
        label = experiment.get_label() + result.get_label()
        if label not in self.INTERACTIONS:
            interaction = Interaction(label)
            interaction.set_experiment(experiment)
            interaction.set_result(result)
            interaction.set_valence(valence)
            interaction.set_meaning(meaning)
            self.INTERACTIONS[label] = interaction
        return self.INTERACTIONS[label]

    def learn_composite_interaction(self, context_interaction, enacted_interaction):
        """
        Learn a new composite interaction or reinforce it if already known.
        :param context_interaction: (Interaction) at time t-1
        :param enacted_interaction: (Interaction) just performed
        """
        #pdb.set_trace
        if context_interaction is not None:
            label = context_interaction.get_label() + enacted_interaction.get_label()
            if label not in self.INTERACTIONS:
                # valence is a sum of the two valences of both primitive interactions
                valence = context_interaction.get_valence() + enacted_interaction.get_valence()
                interaction = Interaction(label)
                interaction.set_pre_interaction(context_interaction)
                interaction.set_post_interaction(enacted_interaction)
                interaction.set_valence(valence)
                self.INTERACTIONS[label] = interaction
                print bcolors.OKGREEN + "Learn " + label + bcolors.ENDC
            else:
                interaction = self.INTERACTIONS[label]
                print bcolors.OKGREEN + 'Incrementing weight for ' + str(interaction) + bcolors.ENDC
                interaction.increment_weight()

    def anticipate(self):
        """
        Anticipate possible interactions based on current context.
        :return: (list) of Anticipations
        """
        #pdb.set_trace
        anticipations = []
        if self.context_interaction is not None:
            activated_interactions = self.get_activated_interactions()
            for activated_interaction in activated_interactions:
                # retrieve proposed interactions
                proposed_interaction = activated_interaction.get_post_interaction()
                # proclivity is a product of the weight of the whole interaction and a valence of proposed
                proclivity = activated_interaction.get_weight() * proposed_interaction.get_valence()
                anticipations.append(Anticipation(proposed_interaction, proclivity))
                print bcolors.OKGREEN + "Afforded: ", proposed_interaction, " proclivity: " + str(proclivity) + bcolors.ENDC
        return anticipations

    def get_activated_interactions(self):
        """
        Retrieve activated interactions based on current context.
        :return: (list) of Interactions
        """
        activated_interactions = []
        # loop through all known interactions
        for key in self.INTERACTIONS:
            activated_interaction = self.INTERACTIONS[key]
            # see if known interaction's pre-interactions is the same as interaction performed at t-1
            if activated_interaction.get_pre_interaction() == self.context_interaction:
                activated_interactions.append(activated_interaction)
        return activated_interactions

    def select_experiment(self, anticipations):
        """Select experiment from proposed anticipations"""
        #pdb.set_trace
        if len(anticipations) > 0:
            #anticipations.sort(key=lambda x: x.compare(), reverse=True)  # choose by proclivity
            anticipations.sort(key=lambda x: x.compare(), reverse=True)  # choose by valence
            afforded_interaction = anticipations[0].get_interaction()
            if afforded_interaction.get_valence() >= 0:
                intended_interaction = afforded_interaction
                print bcolors.OKGREEN + "Intending " + str(intended_interaction) + bcolors.ENDC
                chosen_experiment = intended_interaction.get_experiment()
            else:
                # if proposed interaction leads to negative valence, choose at random
                chosen_experiment = self.get_random_experiment(afforded_interaction)
                print bcolors.OKGREEN + "Don't like the affordance, intending experiment " + chosen_experiment.get_label() + bcolors.ENDC
        else:
            # if nothing was anticipated, choose at random
            # we have decided to use the max valence interaction instead
            valence_max = 0.0
            for interaction in self.primitive_interactions:
                valence = self.primitive_interactions[interaction][2]
                if valence_max < valence:
                    valence_max = valence
                    experiment = self.primitive_interactions[interaction][0]
            chosen_experiment = self.EXPERIMENTS[experiment]
            #chosen_experiment = self.get_random_experiment(None)
            print bcolors.OKGREEN + "Don't know what to do, intending experiment " + chosen_experiment.get_label() + bcolors.ENDC
        return chosen_experiment

    def get_random_experiment(self, interaction):
        random_experiment = random.choice(self.EXPERIMENTS.values())
        # we have decided to use the max valence interaction instead
        if interaction is None:
            valence_max = 0.0
            for interact in self.primitive_interactions:
                valence = self.primitive_interactions[interact][2]
                if valence_max < valence:
                    valence_max = valence
                    experiment = self.primitive_interactions[interact][0]
            best_experiment = self.EXPERIMENTS[experiment]
            return best_experiment
        else:
            # trying to choose a random experiment but avoid choosing one that was part of the rejected interaction
            bad_experiment = interaction.get_experiment()
            chosen_experiment = random.choice(self.EXPERIMENTS.values())
            while chosen_experiment == bad_experiment:
                chosen_experiment = random.choice(self.EXPERIMENTS.values())
            return random_experiment

    def addget_result(self, label):
        if label not in self.RESULTS:
            self.RESULTS[label] = Result(label)
        return self.RESULTS[label]

    def addget_experiment(self, label):
        if label not in self.EXPERIMENTS:
            self.EXPERIMENTS[label] = Experiment(label)
        return self.EXPERIMENTS[label]

    def addget_interaction(self, label):
        if label not in self.INTERACTIONS:
            self.INTERACTIONS[label] = Interaction(label)
        return self.INTERACTIONS[label]

    def get_interaction(self, label):
        if label in self.INTERACTIONS:
            return self.INTERACTIONS[label]
        else:
            return None
        

class RecursiveExistence(Existence):
    """
    Implements recursive self-programming.
    Context is now of depth 2: prev_context_interaction at t-2, 
    and context_interaction at t-1
    """
    def __init__(self, primitive_interactions, environment):
        """
        Initialize existence with a set of primitive interactions provided 
        as a dictionary:
        {(str) interaction meaning: ((str) experiment, (str) result, 
        (int) valence)
        """
        Existence.__init__(self, primitive_interactions, environment)
        self.context_pair_interaction = None  # context at previous two steps (t-2, t-1)

    def step(self):
        #pdb.set_trace()
        print bcolors.OKGREEN + "Memory: " + bcolors.ENDC
        # translate the coded way of interactions in its clear meaning
        for i in self.INTERACTIONS:
            decoded = Decode(str(i))
            translated = decoded.get_translation()
            print bcolors.OKGREEN + translated + bcolors.ENDC
        print "\n"
        #raw_input(bcolors.OKGREEN + "Press ENTER to continue..." + bcolors.ENDC)
        
        anticipations = self.anticipate()
        for anticipation in anticipations:
            print bcolors.OKGREEN + "Anticipated: " + str(anticipation) + bcolors.ENDC
        experiment = self.select_experiment(anticipations)  # recursive experiment
        print bcolors.OKGREEN + "Selected experiment: " + experiment.get_label() + bcolors.ENDC
        intended_interaction = experiment.get_intended_interaction()
        print bcolors.OKGREEN + "Intending: " + str(intended_interaction) + bcolors.ENDC
        print bcolors.OKGREEN + "Intending experiment: ", intended_interaction.get_experiment().get_label() + bcolors.ENDC
        enacted_interaction = self.enact(intended_interaction)

        print bcolors.OKGREEN + "Enacted " + str(enacted_interaction) + bcolors.ENDC
        if enacted_interaction != intended_interaction and experiment.is_abstract:
            failed_result = self.addget_result(enacted_interaction.get_label().upper())
            print bcolors.OKGREEN + "failed result: ", failed_result.get_label() + bcolors.ENDC
            valence = enacted_interaction.get_valence()
            print bcolors.OKGREEN + "experiment: ", str(experiment) + bcolors.ENDC
            enacted_interaction = self.addget_primitive_interaction(experiment, failed_result, valence)
            print bcolors.OKGREEN + "Really enacted " + str(enacted_interaction) + bcolors.ENDC

        if enacted_interaction.get_valence() >= 0:
            self.mood = 'HAPPY'
        else:
            self.mood = 'SAD'

        # learn context_pair_interaction, context_interaction, enacted_interaction
        self.learn_recursive_interaction(enacted_interaction)
        black_board.step_trace = enacted_interaction.__repr__() + " " + self.mood
        
#        print self.EXPERIMENTS
#        print "\n"
#        print self.INTERACTIONS
#        print "\n"
#        print self.RESULTS
#        print "\n"
#        
#        raw_input(bcolors.WARNING + "Press ENTER to continue..." + bcolors.ENDC)
        
        return 1

    def initialize_interactions(self, primitive_interactions):
        for interaction in primitive_interactions:
            meaning = interaction
            experiment_label = primitive_interactions[interaction][0]
            result_label = primitive_interactions[interaction][1]
            valence = primitive_interactions[interaction][2]
            experiment = self.addget_abstract_experiment(experiment_label)
            result = self.addget_result(result_label)
            self.addget_primitive_interaction(experiment, result, valence, meaning)

        for experiment in self.EXPERIMENTS.values():
            interaction = Interaction(experiment.get_label() + "r2")
            interaction.set_valence(1)
            interaction.set_experiment(experiment)
            experiment.set_intended_interaction(interaction)

    def addget_abstract_experiment(self, label):
        if label not in self.EXPERIMENTS:
            experiment = RecursiveExperiment(label)
            self.EXPERIMENTS[label] = experiment
        return self.EXPERIMENTS[label]

    def enact(self, intended_interaction):
        if intended_interaction.is_primitive():
            return self.enact_primitive_interaction(intended_interaction)
            # experiment = intended_interaction.get_experiment()
            # result = self.return_result(experiment)
            # return experiment, result
        else:
            # enact pre-interaction
            enacted_pre_interaction = self.enact(intended_interaction.get_pre_interaction())
            if enacted_pre_interaction != intended_interaction.get_pre_interaction():
                return enacted_pre_interaction
            else:
                # enact post-interaction
                enacted_post_interaction = self.enact(intended_interaction.get_post_interaction())
                return self.addget_composite_interaction(enacted_pre_interaction, enacted_post_interaction)

    def enact_primitive_interaction(self, intended_interaction):
        """
        Implements the cognitive coupling between the agent and the environment
        Tries to enact primitive intended_interaction.
        """
        experiment = intended_interaction.get_experiment()
        result_label = self.environment.return_result(experiment)
        result = self.addget_result(result_label)
        return self.addget_primitive_interaction(experiment, result)

    def select_experiment(self, anticipations):
        #pdb.set_trace()
        if black_board.move_count > 1:
            anticipations.sort(key=lambda x: x.compare(), reverse=True)  # choose by valence
            selected_anticipation = anticipations[0]
            return selected_anticipation.get_experiment()
        else:
            # if nothing was anticipated, choose at random
            # we have decided to use the max valence interaction instead
            valence_max = 0.0
            for interaction in self.primitive_interactions:
                valence = self.primitive_interactions[interaction][2]
                if valence_max < valence:
                    valence_max = valence
                    experiment = self.primitive_interactions[interaction][0]
            chosen_experiment = self.EXPERIMENTS[experiment]
            return chosen_experiment

    def anticipate(self):
        anticipations = self.get_default_anticipations()
        # print "Default anticipations: ", anticipations
        if self.context_interaction is not None:
            activated_interactions = self.get_activated_interactions()
            for activated_interaction in activated_interactions:
                # print "activated interaction: ", activated_interaction
                experiment = activated_interaction.get_post_interaction().get_experiment()
                # print "activated experiment: " + experiment.get_label()
                proclivity = activated_interaction.get_weight() * activated_interaction.get_post_interaction().get_valence()
                # print "activated proclivity: " + str(proclivity)
                anticipation = RecursiveAnticipation(experiment, proclivity)
                # print "activated anticipation: ", anticipation
                if anticipation not in anticipations:
                    anticipations.append(anticipation)
                else:
                    index = anticipations.index(anticipation)
                    anticipations[index].add_proclivity(proclivity)
                # print "Afforded ", anticipation
        return anticipations

    def get_default_anticipations(self):
        """All known experiments are proposed by default with proclivity 0"""
        anticipations = []
        for experiment in self.EXPERIMENTS.values():
            if not experiment.is_abstract:
                anticipation = RecursiveAnticipation(experiment, 0)
                anticipations.append(anticipation)
        random.shuffle(anticipations) # shuffle order
        return anticipations

    def get_activated_interactions(self):
        context_interactions = []
        if self.context_interaction is not None:
            context_interactions.append(self.context_interaction)
            if not self.context_interaction.is_primitive():
                context_interactions.append(self.context_interaction.get_post_interaction())
            if self.context_pair_interaction is not None:
                context_interactions.append(self.context_pair_interaction)
        print bcolors.OKGREEN + "Context: " + repr(context_interactions) + bcolors.ENDC
        activated_interactions = []
        for key in self.INTERACTIONS:
            activated_interaction = self.INTERACTIONS[key]
            if not activated_interaction.is_primitive():
                if activated_interaction.get_pre_interaction() in context_interactions:
                    activated_interactions.append(activated_interaction)
        for activated_interaction in activated_interactions:
            print bcolors.OKGREEN + "Activated: " + str(activated_interaction) + bcolors.ENDC
        return activated_interactions

    def get_context_interaction(self):
        return self.context_interaction

    def set_context_interaction(self, enacted_interaction):
        self.context_interaction = enacted_interaction

    def get_context_pair_interaction(self):
        return self.context_pair_interaction

    def set_context_pair_interaction(self, enacted_pair_interaction):
        self.context_pair_interaction = enacted_pair_interaction

    # Learning:
    def learn_recursive_interaction(self, enacted_interaction):
        enacted_pair_interaction = None
        if self.context_interaction is not None:
            # if hist[-1]: learn(hist[-1], enacted)
            enacted_pair_interaction = self.addreinforce_composite_interaction(self.context_interaction, enacted_interaction)

            if self.context_pair_interaction is not None:
                # if hist[-1] and hist[-2]
                # learn <penultimate <previous current>>
                self.addreinforce_composite_interaction(self.context_pair_interaction.get_pre_interaction(), enacted_pair_interaction)
                # learn <<penultimate previous> current>
                self.addreinforce_composite_interaction(self.context_pair_interaction, enacted_interaction)

        self.set_context_interaction(enacted_interaction)
        self.set_context_pair_interaction(enacted_pair_interaction)

    def addreinforce_composite_interaction(self, pre_interaction, post_interaction):
        #pdb.set_trace
        composite_interaction = self.addget_composite_interaction(pre_interaction, post_interaction)
        composite_interaction.increment_weight()

        if composite_interaction.get_weight() == 1:
            print bcolors.OKGREEN + "Learned: " + str(composite_interaction) + bcolors.ENDC
        else:
            print bcolors.OKGREEN + "Reinforced: " + str(composite_interaction) + bcolors.ENDC

        return composite_interaction

    def addget_composite_interaction(self, pre_interaction, post_interaction):
        """Record in or get from a composite interaction in memory.
        If a new composite interaction is created, then a new abstract 
        experience is also created and associated to it.
        """
        pdb.set_trace
        label = "<" + pre_interaction.get_label() + post_interaction.get_label() + ">"
        interaction = self.get_interaction(label)
        if interaction is None:
            interaction = self.addget_interaction(label)
            interaction.set_pre_interaction(pre_interaction)
            interaction.set_post_interaction(post_interaction)
            valence = pre_interaction.get_valence() + post_interaction.get_valence()
            interaction.set_valence(valence)
            experiment_label = interaction.get_label().upper()
            new_experiment = self.addget_abstract_experiment(experiment_label)
            new_experiment.set_abstract()
            new_experiment.set_intended_interaction(interaction)
            interaction.set_experiment(new_experiment)
        return interaction
    

class ConstructiveExistence(RecursiveExistence):
    """
    In constructive existence the basic unit of analysis and implementation
    is interaction, not experiments and results.
    """
    def __init__(self, primitive_interactions, environment):
        RecursiveExistence.__init__(self, primitive_interactions, environment)

    # Existence 50.2
    def step(self):
        #pdb.set_trace
        print bcolors.OKGREEN + "Memory: " + bcolors.ENDC
        for i in self.INTERACTIONS:
            decoded = Decode(str(i))
            translated = decoded.get_translation()
            print bcolors.OKGREEN + translated + bcolors.ENDC
        print "\n"
        #raw_input(bcolors.OKGREEN + "Press ENTER to continue..." + bcolors.ENDC)
        
        anticipations = self.anticipate()
        for anticipation in anticipations:
            print bcolors.OKGREEN + "Anticipated: " + str(anticipation) + bcolors.ENDC
        intended_interaction = self.select_interaction(anticipations)
        print bcolors.OKGREEN + "Intended interaction: " + str(intended_interaction) + bcolors.ENDC
        enacted_interaction = self.enact(intended_interaction)
        print bcolors.OKGREEN + "Enacted interaction: " + str(enacted_interaction) + bcolors.ENDC

        # if intended interaction failed, record the alternative
        if enacted_interaction != intended_interaction:
            intended_interaction.add_alternative_interaction(enacted_interaction)
            print bcolors.OKGREEN + "Alternative interactions:" + str(intended_interaction.get_alternative_interactions()) + bcolors.ENDC

        if enacted_interaction.get_valence() >= 0:
            self.mood = 'HAPPY'
        else:
            self.mood = 'SAD'

        self.learn_recursive_interaction(enacted_interaction)
        black_board.step_trace = enacted_interaction.__repr__() + " " + self.mood
        
#        print self.EXPERIMENTS
#        print "\n"
#        print self.INTERACTIONS
#        print "\n"
#        print self.RESULTS
#        print "\n"
#        
#        raw_input(bcolors.WARNING + "Press ENTER to continue..." + bcolors.ENDC)
        
        return 1

    def initialize_interactions(self, primitive_interactions):
        for key in primitive_interactions:
            meaning = key
            experiment_label = primitive_interactions[key][0]
            result_label = primitive_interactions[key][1]
            interaction_label = experiment_label + result_label
            valence = primitive_interactions[key][2]
            primitive_interaction = self.addget_interaction(interaction_label)
            primitive_interaction.set_valence(valence)
            primitive_interaction.set_meaning(meaning)
            # creating default experiments to begin with
            self.addget_abstract_experiment(primitive_interaction)

    def addget_abstract_experiment(self, interaction):
        """
        All experiments are now abstract, namely they are interactions.
        """
        #pdb.set_trace
        label = interaction.get_label().upper()
        if label not in self.EXPERIMENTS:
            abstract_experiment = RecursiveExperiment(label)
            abstract_experiment.set_intended_interaction(interaction)
            abstract_experiment.set_abstract()
            interaction.set_experiment(abstract_experiment)
            self.EXPERIMENTS[label] = abstract_experiment
        return self.EXPERIMENTS[label]

    def addget_composite_interaction(self, pre_interaction, post_interaction):
        """
        Record in or get from a composite interaction in memory.
        If a new composite interaction is created, then a new abstract 
        experience is also created and associated to it.
        """
        #pdb.set_trace
        label = "<" + pre_interaction.get_label() + post_interaction.get_label() + ">"
        interaction = self.get_interaction(label)
        if interaction is None:
            interaction = self.addget_interaction(label)
            interaction.set_pre_interaction(pre_interaction)
            interaction.set_post_interaction(post_interaction)
            valence = pre_interaction.get_valence() + post_interaction.get_valence()
            interaction.set_valence(valence)
            self.addget_abstract_experiment(interaction)
        return interaction

    # Existence 50.2
    def anticipate(self):
        #pdb.set_trace
        anticipations = self.get_default_anticipations()
        print bcolors.OKGREEN + "Default anticipations: " + str(anticipations) + bcolors.ENDC
        activated_interactions = self.get_activated_interactions()
        # print "Activated interactions: ", activated_interactions
        if self.context_interaction is not None:
            for activated_interaction in activated_interactions:
                proposed_interaction = activated_interaction.get_post_interaction()
                # print "activated experiment: " + experiment.get_label()
                proclivity = activated_interaction.get_weight() * proposed_interaction.get_valence()
                anticipation = ConstructiveAnticipation(proposed_interaction, proclivity)
                # print "activated anticipation: " + anticipation.__repr__()
                if anticipation not in anticipations:
                    anticipations.append(anticipation)
                else:
                    index = anticipations.index(anticipation)
                    # increment proclivity if anticipation is already in the list
                    anticipations[index].add_proclivity(proclivity)
                # print "Afforded " + anticipation.__repr__()

            for anticipation in anticipations:
                index = anticipations.index(anticipation)
                alternative_interactions = anticipation.get_interaction().get_alternative_interactions()
                for interaction in alternative_interactions:
                    for activated_interaction in activated_interactions:
                        # combine proclivity with alternative interactions
                        if interaction == activated_interaction.get_post_interaction():
                            proclivity = activated_interaction.get_weight() * interaction.get_valence()
                            anticipations[index].add_proclivity(proclivity)
        return anticipations

    # Existence 50.2
    def get_default_anticipations(self):
        anticipations = []
        for interaction in self.INTERACTIONS.values():
            if interaction.is_primitive():
                # print "interaction is primitive"
                anticipation = ConstructiveAnticipation(interaction, 0)
                # print "adding anticipation", anticipation
                anticipations.append(anticipation)
        # sort default anticipations by valence - this could be random...
        anticipations.sort(key=lambda x: x.get_interaction().get_valence(), reverse=True)
        #anticipations.sort(key=lambda x: x.get_interaction().get_label())
        return anticipations

    def enact(self, intended_interaction):
        #pdb.set_trace
        # if interaction is primivite, consult the world and get what was actually enacted
        if intended_interaction.is_primitive():
            enacted_interaction_label = self.environment.enact_primitive_interaction(intended_interaction)
            enacted_interaction = self.addget_interaction(enacted_interaction_label)
            return enacted_interaction
        else:
            # if interaction is composite, try to enact its pre-interaction
            enacted_pre_interaction = self.enact(intended_interaction.get_pre_interaction())
            # if enacting failed, break the sequence and return
            if enacted_pre_interaction != intended_interaction.get_pre_interaction():
                return enacted_pre_interaction
            else:
                # if enacting pre-interaction succeeded, try to enact post-interaction
                enacted_post_interaction = self.enact(intended_interaction.get_post_interaction())
                return self.addget_composite_interaction(enacted_pre_interaction, enacted_post_interaction)

    def select_interaction(self, anticipations):
        anticipations.sort(key=lambda x: x.compare(), reverse=True)  # choose by proclivity
        selected_anticipation = anticipations[0]
        intended_interaction = selected_anticipation.get_interaction()
        # if intended_interaction.get_valence() < 0:
        #     intended_interaction = self.get_random_interaction(intended_interaction)
        #     print "Don't like the affordance, intending random interaction..."
        return intended_interaction

    def get_random_interaction(self, interaction):
        random_interaction = random.choice(self.INTERACTIONS.values())
        if interaction is None:
            return random_interaction
        else:
            bad_experiment = interaction.get_experiment()
            chosen_experiment = random_interaction.get_experiment()
            #while chosen_experiment == bad_experiment:
            while chosen_experiment != 'e1':    
                random_interaction = random.choice(self.INTERACTIONS.values())
            return random_interaction


class Interaction:
    """
    An interaction is a basic sensorimotor pattern available to the agent.
    An interaction can be primitive or composite. If primitive, it is an association of experiment and result.
    If composite, it has pre- and post-interaction parts.
    Each interaction has valence and weight.
    """
    def __init__(self, label):
        self.label = label
        self.valence = 0
        self.experiment = None
        self.result = None
        self.meaning = None
        self.weight = 0
        self.pre_interaction = None
        self.post_interaction = None
        self.alternative_interactions = []

    def get_label(self):
        return self.label

    def get_experiment(self):
        return self.experiment

    def set_experiment(self, experiment):
        self.experiment = experiment

    def get_result(self):
        return self.result

    def set_result(self, result):
        self.result = result

    def get_valence(self):
        #pdb.set_trace
        interaction = self
        if black_board.process_boredom:
            return boredom_handler.process_boredom(black_board.ex.INTERACTIONS, interaction, self.valence)
        else:
            if self.is_primitive():
                return self.valence
            else:
                pre = self.get_pre_interaction()
                post = self.get_post_interaction()
                self.valence = pre.get_valence() + post.get_valence()
                return self.valence

    def set_valence(self, valence):
        self.valence = valence

    def get_meaning(self):
        return self.meaning

    def set_meaning(self, meaning):
        self.meaning = meaning

    def get_pre_interaction(self):
        return self.pre_interaction

    def set_pre_interaction(self, pre_interaction):
        self.pre_interaction = pre_interaction

    def get_post_interaction(self):
        return self.post_interaction

    def set_post_interaction(self, post_interaction):
        self.post_interaction = post_interaction

    def is_primitive(self):
        return self.pre_interaction is None

    def get_weight(self):
        return self.weight

    def increment_weight(self):
        self.weight += 1

    def add_alternative_interaction(self, interaction):
        if interaction not in self.alternative_interactions:
            self.alternative_interactions.append(interaction)

    def get_alternative_interactions(self):
        return self.alternative_interactions
    
    def unwrap(self):
        if self.is_primitive():
            return[self]
        else:
            """
            Unwrap the composite interaction.
            :return: A list of primitive interactions.
            """
            return self.get_pre_interaction().unwrap() + self.get_post_interaction().unwrap()

    def __repr__(self):
        return "{0}, valence {1}, weight {2}".format(self.get_label(), self.get_valence(), self.get_weight())
       

class Environment:
    """
    Class that implements the basic real-world environment.
    """
    def __init__(self):
        self.last_result = None

    def return_result(self, experiment):
        """
        Consult the world and return primitive result in response to 
        the experiment initiated by the agent.
        :param experiment: (Experiment) experiment issued by the agent
        :return: (str) result
        """
        result = None
        black_board.laser_scan()
        black_board.right_status()
        if experiment.get_label() == 'e1':
            if black_board.Right1:
                black_board.adv_distance = black_board.distance_front - 1.7
            else:
                black_board.adv_distance = 0.0
            black_board.adv_angle = 0.0
            black_board.move_adv()
            if not black_board.move_fail and black_board.Right1:
                result = 'r1'  # moved forward following a wall on the right
            elif not black_board.move_fail:
                result = 'r4'  # moving forward sensing no wall 
            else:
                result = 'r10' # move failed: if robot bumps 
        elif experiment.get_label() == 'e2':
            black_board.adv_distance = 0.0
            black_board.adv_angle = math.pi/2
            black_board.move_adv()
            result = 'r2'   # turn left
        elif experiment.get_label() == 'e3':
            black_board.adv_distance = 0.0
            black_board.adv_angle = -math.pi/2
            black_board.move_adv()
            result = 'r3'   # turn right
        elif experiment.get_label() == 'e4':
            if black_board.driving_forward:
                result = 'r4'  # front free: no wall
            else:
                result = 'r5'  # front busy: wall in front
        elif experiment.get_label() == 'e5':
            black_board.right_status()
            if black_board.Right1:
                result = 'r6'   # right sensing: wall on the right
            else:
                result = 'r14'   # nothing on the right
        elif experiment.get_label() == 'e6':
            black_board.left_status()
            if black_board.Left1:
                result = 'r7'   # left sensing: wall on the left
            else:
                result = 'r13'   # nothing on the left

        self.last_result = result
        return result

class ConstructiveEnvironment:
    """
    Class that implements constructive environment, in which interactions 
    are the basic primitives.
    """
    # TIMESTEP = 1
    def __init__(self):
        self.last_interaction = None

    def enact_primitive_interaction(self, intended_interaction):
        """
        Consult the world and return enacted interaction in response to 
        the agent's intended interaction.
        :param intended_interaction: (Interaction) interaction attempted by the agent
        :return: (Interaction) interaction actually enacted
        """
        #pdb.set_trace()
        
        experiment = intended_interaction.get_label()[:2]
        result = None
        black_board.laser_scan()
        black_board.right_status()
        if experiment == 'e1':
            if black_board.Right1:
                black_board.adv_distance = black_board.distance_front - 1.7
            else:
                black_board.adv_distance = 0.0
            black_board.adv_angle = 0.0
            black_board.move_adv()
            if not black_board.move_fail and black_board.Right1:
                result = 'r1'   # moved forward following a wall on the right
            elif not black_board.move_fail:
                result = 'r4'  # moving forward sensing no wall    
            else:
                result = 'r10' # move failed: if robot bumps
        elif experiment == 'e2':
            black_board.adv_distance = 0.0
            black_board.adv_angle = math.pi/2
            black_board.move_adv()
            result = 'r2'   # turn left
        elif experiment == 'e3':
            black_board.adv_distance = 0.0
            black_board.adv_angle = -math.pi/2
            black_board.move_adv()
            result = 'r3'   # turn right
        elif experiment == 'e4':
            if black_board.driving_forward:
                result = 'r4'  # front free: no wall
            else:
                result = 'r5'  # front busy: wall in front
        elif experiment == 'e5':
            black_board.right_status()
            if black_board.Right1:
                result = 'r6'   # right sensing: wall on the right
            else:
                result = 'r14'   # nothing on the right
        elif experiment == 'e6':
            black_board.left_status()
            if black_board.Left1:
                result = 'r7'   # left sensing: wall on the left
            else:
                result = 'r13'   # nothing on the left
                
        enacted_interaction = experiment+result
        self.last_interaction = enacted_interaction

        return enacted_interaction
 
    
# class that represent an agent's boredom handler.    
class BoredomHandler(object):
    """
    Abstract boredom handler class.
    """
    @abc.abstractmethod
    def process_boredom(self, INTERACTIONS, interaction, unmodified_valence):
        """
        Modifies the valence of an interaction such that boredom is handled.

        :param INTERACTIONS: The interaction memory
        :param interaction: The interaction to process boredom for
        :param unmodified_valence: The unmodified (raw) valence of the interaction
        :return: The modified valence taking boredom into account
        """
        raise NotImplementedError("Should be implemented by child")


class PassthroughBoredomHandler(BoredomHandler):
    """
    A boredom handler not implementing any boredom measures.
    """
    def process_boredom(self, INTERACTIONS, interaction, unmodified_valence):
        return unmodified_valence

class WeightBoredomHandler(BoredomHandler):
    """
    A boredom handler taking into account the weight of interactions. The sum
    of the hierarchical weight of an interaction is calculated, and its
    contribution to the total weight is calculated. This is used to discount
    interactions that have a high contribution.
    """
    def interaction_total_weight(self, INTERACTIONS, interaction):
        """
        Get the total (hierarchical) weight of an interaction. This takes the
        sum of all weights of all interactions inside the hierarchy of this
        interaction. E.g., for a composite interaction <i1, i2> the sum is
        weight(<i1, i2>) = <i1, i2>.weight + weight(i1) + weight(i2).

        :param INTERACTIONS: The interaction memory
        :param interaction: The interaction to get the hierarchical weight for
        :return: The hierarchical weight of the interaction
        """
        if interaction.is_primitive():
            return interaction.get_weight()
        else:
            return (
                interaction.get_weight() 
                + self.interaction_total_weight(INTERACTIONS, interaction.get_pre_interaction()) 
                + self.interaction_total_weight(INTERACTIONS, interaction.get_post_interaction())
            )
    
    def process_boredom(self, INTERACTIONS, interaction, unmodified_valence):
        if unmodified_valence > 0:
            
            sum = 0
            for key in INTERACTIONS:
                sum += INTERACTIONS[key].get_weight()
            weight = self.interaction_total_weight(INTERACTIONS, interaction)
            modifier = (1 - float(weight)/float(sum))
            return unmodified_valence * modifier
        else:
            return unmodified_valence

class RepetitiveBoredomHandler(BoredomHandler):
    """
    A boredom handler taking into the account the last few (primitive)
    interactions enacted by the agent, and compares the similarity of those
    with the proposed interaction. The more similar, the more the interaction
    is penalized.
    """
    
    HISTORY_CONSIDER_SIZE = 15

    def count_interactions(self, interaction_sequence):
        """
        Count the interaction occurrences in a sequence.
        :param interaction_sequence: The interaction sequence
        :return: A Counter (dictionary) object mapping from interactions to
                 their frequency in the sequence.
        """
        count = Counter()
        for interaction_ in interaction_sequence:
            count[interaction_] += 1

        return count

    def similarity(self, count1, count2):
        """
        Calculate the cosine similarity between two counts (Counter dictionaries, seen as vectors).
        :param count1: The first interaction count
        :param count2: The second interaction count
        :return: The cosine similarity between the two counts
        """
        c1_dot_c2 = 0
        c1_len_squared = 0
        c2_len_squared = 0

        for interaction_name in count1:
            c1_dot_c2 += count1[interaction_name] * count2[interaction_name]
            c1_len_squared += count1[interaction_name]**2

        for interaction_name in count2:
            c2_len_squared += count2[interaction_name]**2

        if c1_len_squared == 0:
            return -1
        else:
            return c1_dot_c2 / (math.sqrt(c1_len_squared) * math.sqrt(c2_len_squared))

    def process_boredom(self, INTERACTIONS, interaction, unmodified_valence):
        #pdb.set_trace()
        history = INTERACTIONS.keys()[-self.HISTORY_CONSIDER_SIZE:]
        history_count = self.count_interactions(history)
        interaction_count = self.count_interactions(interaction.unwrap())

        similarity = self.similarity(history_count, interaction_count)
        modifier = 1 - similarity

        return unmodified_valence * modifier

class WeightRepetitiveBoredomHandler(BoredomHandler):
    """
    A boredom handler combining the weight boredom handler and repetitive
    boredom handler by taking the average valence output of the two.
    """
    def __init__(self):
        self.weightBoredomHandler = WeightBoredomHandler()
        self.repetitiveBoredomHandler = RepetitiveBoredomHandler()

    def process_boredom(self, INTERACTIONS, interaction, unmodified_valence):
        modified_valence = (
            self.weightBoredomHandler.process_boredom(INTERACTIONS, interaction, unmodified_valence)
            +
            self.repetitiveBoredomHandler.process_boredom(INTERACTIONS, interaction, unmodified_valence)
            )/2
        print bcolors.OKGREEN + "Interaction modified: " + repr(interaction) + "Modified valence: " + str(modified_valence) + bcolors.ENDC
        return modified_valence

# the kind of boredom handler is going to be used
boredom_handler = RepetitiveBoredomHandler()
    
if __name__ == '__main__':
    #pdb.set_trace()
    # run with  i.e. rosrun eca_agent01.py constructive
    parser = argparse.ArgumentParser()
    parser.add_argument("mechanism", type=str, help="specify the learning mechanism to be used",
                        choices=["simple", "recursive", "constructive"])
    args = parser.parse_args()
    
    black_board.agent_mechanism = args.mechanism
    
    tree = EcaAgent02()
  