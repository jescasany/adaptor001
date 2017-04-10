#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 02 2017
Last visited on 30/03/2017
@author: juan

The Enactive Cognitive Architecture (ECA)
based on Georgeon, Marshall, and Manzotti (2013). ECA: An enactivist cognitive
architecture based on sensorimotor modeling. Biologically Inspired Cognitive
Architectures, 6:46-57.

Implemented following the Behavior Trees model.
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

import os
import argparse
from interaction import *
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
        if coefficients[1] != 5.0:
            black_board.distance_to_right_wall = coefficients[1]/2
        else:
            black_board.distance_to_right_wall = coefficients[1]
        
    def left_wall_param(self, y):
        y = y[438:639]
        coefficients = self.regression(y)
        # approximate wall_angle (degrees)
        black_board.left_wall_angle = normalize_angle(math.atan(coefficients[0]))*360./(2*math.pi)
        # approximate dist to wall (meters)
        if coefficients[1] != 5.0:
            black_board.distance_to_left_wall = coefficients[1]/2
        else:
            black_board.distance_to_left_wall = coefficients[1]
            
    def laser_scan(self):
        rospy.loginfo("Waiting for /base_scan topic...")
        rospy.wait_for_message('/base_scan', sensor_msgs.msg.LaserScan)
        # Subscribe the /base_scan topic to get the range readings  
        rospy.Subscriber('/base_scan', sensor_msgs.msg.LaserScan, self.scan_callback, queue_size = 10)
        rospy.sleep(0.1)
        if min(black_board.kinect_scan) < 1.2:
            black_board.driving_forward = False
        else:
            black_board.driving_forward = True
        rospy.loginfo("laser_scan done")
        
    def formule(self, L, tolerance):
        f = 0
        f = L[1] - (L[0] + L[2])/2
        f = self.round_to_zero(f, tolerance)
        return f
        
    def moving_window_filtro(self, x, tolerance, n_neighbors=1):
        n = len(x)
        width = n_neighbors*2 + 1
        x = [x[0]]*n_neighbors + x + [x[-1]]*n_neighbors
        # To complete the function,
        # return a list of the filtered values from i to i+width for all values i from 0 to n-1.
        filtro = []
        singularity = []
        for i in range(n):
            fi = self.formule(x[i:i+width], tolerance)
            if fi > 0:
                filtro.append(fi)
                singularity.append(i)
            else:
                filtro.append(0.0)
            
        return filtro, singularity
    
    def round_to_zero(self, val, tolerance):
        if -1 * tolerance < val < tolerance:
            return 0
        else:
            return val
        
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
            if black_board.driving_forward:
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
                (black_board.agent_position, black_board.agent_rotation) = advance(-1.0, 0.0, da=True)
                black_board.move_fail = True
        except:
            rospy.loginfo("move_adv failed.")
            (black_board.agent_position, black_board.agent_rotation) = advance(-1.0, 0.0, da=True)
            black_board.move_fail = True
            return 1
        return 1
    
    def right_status(self):
        rospy.loginfo("Estoy en right status")
        #pdb.set_trace()
        self.laser_scan()
        rospy.sleep(2)
        tolerance = 0.1
        y = list()
        y = black_board.kinect_scan
        #black_board.plotter(black_board.kinect_scan)
        black_board.filtered_scan = self.moving_window_filtro(y[0:200], tolerance, n_neighbors=1)[0]
        # returns the reading has the discontinuity
        black_board.right_singularity = self.moving_window_filtro(y[0:200], tolerance, n_neighbors=1)[1]
        print "right singularities: ", black_board.right_singularity
        #black_board.plotter(black_board.filtered_scan, "Right")
        singular_readings = []
        stretches = []
        for i in black_board.right_singularity:
            stretches.append(sum(y[0:i])/len(y[0:i]))
            singular_readings.append(y[i])
        print "right singular readings: ", singular_readings
        print "right stretches: ", stretches
        black_board.right_wall_param(y)
        agent_rotation_angle = quat_to_angle(black_board.agent_rotation)
        print "odom_angle: ", math.degrees(normalize_angle(agent_rotation_angle))
        print "wall_angle: ", black_board.right_wall_angle
        #update average_distance_to_right_wall
        black_board.average_distance_to_right_wall = sum(y[0:200]) / 200.
        print "average distance to right wall: ", black_board.average_distance_to_right_wall
        print bcolors.OKGREEN + "distance to right wall(coefficients[1]): " +  str(black_board.distance_to_right_wall) + bcolors.ENDC
#        raw_input("Press a key to continue...")
        if black_board.distance_to_right_wall < 4.5:
            rospy.loginfo(bcolors.OKGREEN + "Right sensing" + bcolors.ENDC)
            black_board.Right = True
            black_board.adv_distance = 1.0
            
            l = [r for r in y[0:200]  if r != 5.0]
            right_length = len(l)
            
            #pdb.set_trace()   
            if right_length < black_board.Right_Length and black_board.Right_Length != 200:
                print bcolors.OKGREEN + "RIGHT SHORTEN" + bcolors.ENDC
                print bcolors.OKGREEN + "CORNER NEXT at: " + str(black_board.Right_Length) + " length" + bcolors.ENDC
                black_board.adv_distance = 1.0
                
            black_board.Right_Length = right_length
            if black_board.Right_Length == 0:
                black_board.Right = False
                black_board.Right_Corner = True
                black_board.Right_Length = 200
        else:
            rospy.loginfo(bcolors.OKGREEN + "Nothing on the right" + bcolors.ENDC)
            black_board.Right = False
            #black_board.Right_Corner = False
        return 1
    
    def left_status(self):
        rospy.loginfo("Estoy en left status")
        self.laser_scan()
        rospy.sleep(2)
        tolerance = 0.1
        y = list()
        y = black_board.kinect_scan
        #black_board.plotter(black_board.kinect_scan)
        black_board.filtered_scan = self.moving_window_filtro(y[438:-1], tolerance, n_neighbors=1)[0]
        # returns the reading has the discontinuity
        black_board.left_singularity = self.moving_window_filtro(y[438:-1], tolerance, n_neighbors=1)[1]
        black_board.left_singularity = [x+438 for x in black_board.left_singularity]
        print "left singularities: ", black_board.left_singularity
        #black_board.plotter(black_board.filtered_scan, "Left")
        singular_readings = []
        for i in black_board.left_singularity:
            singular_readings.append(y[i])
        print "left singular readings: ", singular_readings
        black_board.left_wall_param(y)
        agent_rotation_angle = quat_to_angle(black_board.agent_rotation)
        print "odom_angle: ",  math.degrees(normalize_angle(agent_rotation_angle))
        print "wall_angle: ", black_board.left_wall_angle
        #update average_distance_to_left_wall
        black_board.average_distance_to_left_wall = sum(y[438:-1])/200.
        print "average distance to left wall: ", black_board.average_distance_to_left_wall
        print "distance to left wall(coefficients[1]): ", black_board.distance_to_left_wall
#        raw_input("Press a key to continue...")
        if black_board.distance_to_left_wall < 4.5:
            rospy.loginfo("Left sensing")
            black_board.Left = True
            black_board.adv_distance = 1.0
            
            l = [r for r in y[438:-1] if r!=5.0]
            left_length = len(l)
            
            if left_length < black_board.Left_Length and black_board.Left_Length != 200:
                
                print bcolors.OKGREEN + "LEFT APPEARS" + bcolors.ENDC
                print bcolors.OKGREEN + "LEFT NEXT at: " + str(black_board.Left_Length) + " reading" + bcolors.ENDC
                black_board.Left = True
                black_board.Left_Corner = True
                black_board.adv_distance = 1.0
                
            black_board.Left_Length = left_length
        else:
            rospy.loginfo(bcolors.OKGREEN + "Nothing on the left" + bcolors.ENDC)
            black_board.Left = False
            
    def scan_callback(self, msg):
        black_board.kinect_scan = list(msg.ranges) # transform to list since ranges is a tuple

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
black_board.distance_to_front = 7.0
black_board.Front = False
black_board.Front_All = False
black_board.Left = False
black_board.Right = False
black_board.Right_Corner = False
black_board.Right_Length = 200
black_board.Left_Length = 200
(black_board.agent_position, black_board.agent_rotation) = black_board.agent_pose 
black_board.odom_angle = 0.0

black_board.chs = 1     # used to change the sign(+/-) of the PI/2 turn
black_board.agent_mechanism = ''


class EcaAgent00:
    
    def __init__(self):
        
        #pdb.set_trace()
        rospy.init_node("eca_agent00_tree")
        # Set the shutdown function (stop the agent)
        rospy.on_shutdown(self.shutdown)
        # Publisher to manually control the agent (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
#        self.laser_scan()
        rate = rospy.Rate(10)

         # initialize existence
        ex = None
        # initialize primitive interactions
        primitive_interactions = {"move forward": ("e1", "r1", 10),\
                                  "move forward fail": ("e1", "r10", -10),\
                                  "turn left": ("e2", "r2", -5),\
                                  "turn right": ("e3", "r3", -2),\
                                  "front free": ("e4", "r4", -2),\
                                  "front busy": ("e4", "r5", -2),\
                                  "right sensing": ("e5", "r6", 10),\
                                  "nothing on right": ("e5", "r8", -5),\
                                  "left sensing": ("e6", "r7", -2),\
                                  "nothing on left": ("e6", "r9", -2)}
        # initialize environments and existences
        self.mechanism = black_board.agent_mechanism
        if self.mechanism == "simple":
            environment = Environment()
            ex = Existence(primitive_interactions, environment)
        elif self.mechanism == "recursive":
            environment = Environment()
            ex = RecursiveExistence(primitive_interactions, environment)
        elif self.mechanism == "constructive":
            environment = ConstructiveEnvironment()
            ex = ConstructiveExistence(primitive_interactions, environment)
        # Create the root node
        ECAAGENT00 = Sequence("ECAAGENT00")
        
        START_STEP = CallbackTask("START STEP", ex.step)
        
        I_F_IS_VISITED =IgnoreFailure("I_F IS VISITED")
        
        IS_VISITED = CallbackTask("is visited", self.is_visited)
        
        ECAAGENT00.add_child(START_STEP)
        ECAAGENT00.add_child(I_F_IS_VISITED)
        
        # Display the tree before beginning execution
        print bcolors.HEADER + "ECAAGENT00 Behavior Tree" + bcolors.ENDC
        print_tree(ECAAGENT00, indent=0, use_symbols=True)
        print_dot_tree(ECAAGENT00, dotfilepath='/home/juan/catkin_ws/src/adaptor001/tree.dot')
        # Run the tree
        while not rospy.is_shutdown():
            ECAAGENT00.run()
            print bcolors.OKGREEN + str(black_board.sim_step) + black_board.step_trace + bcolors.ENDC
            print "\n"
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
            return False
        if (black_board.agent_position, black_board.agent_rotation) in black_board.waypoints[0:-1]:
            rospy.loginfo("Waypoint is visited.")
            return True
        else:
            rospy.loginfo("Waypoint is not visited.")
            return False
    
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
    EXPERIMENTS = dict()
    INTERACTIONS = dict()
    RESULTS = dict()

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
        self.initialize_interactions(primitive_interactions)
        
    def step(self):
        """
        Execute a single simulation step.
        :return: (str) performed interaction and mood
        """
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
        if context_interaction is not None:
            label = context_interaction.get_label() + enacted_interaction.get_label()
            if label not in self.INTERACTIONS:
                # valence is a sum of primitive interactions
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
            chosen_experiment = self.get_random_experiment(None)
            print bcolors.OKGREEN + "Don't know what to do, intending experiment " + chosen_experiment.get_label() + bcolors.ENDC
        return chosen_experiment

    def get_random_experiment(self, interaction):
        random_experiment = random.choice(self.EXPERIMENTS.values())
        if interaction is None:
            return random_experiment
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
        print bcolors.OKGREEN + "Memory: ", str(self.INTERACTIONS.keys()) + bcolors.ENDC
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
        Implements the cognitive coupling between the agent and the environment.
        Tries to enact primitive intended_interaction.
        """
        experiment = intended_interaction.get_experiment()
        result_label = self.environment.return_result(experiment)
        result = self.addget_result(result_label)
        return self.addget_primitive_interaction(experiment, result)

    def select_experiment(self, anticipations):
        anticipations.sort(key=lambda x: x.compare(), reverse=True)  # choose by valence
        selected_anticipation = anticipations[0]
        return selected_anticipation.get_experiment()

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
        print bcolors.OKGREEN + "Context: " + str(context_interactions) + bcolors.ENDC
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
        # print "Memory: ", self.INTERACTIONS.keys()
        print bcolors.OKGREEN + "Memory: ", str(self.INTERACTIONS.keys()) + bcolors.ENDC
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
            while chosen_experiment == bad_experiment:
                random_interaction = random.choice(self.INTERACTIONS.values())
            return random_interaction

        

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
        if experiment.get_label() == 'e1':
            black_board.adv_distance = 1.0
            black_board.adv_angle = 0.0
            black_board.move_adv()
            if black_board.move_fail:
                result = 'r1'  # moved forward
            else:
                result = 'r10' # move failed
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
                result = 'r4'  # front free
            else:
                result = 'r5'  # front busy
        elif experiment.get_label() == 'e5':
            black_board.right_status()
            if black_board.Right:
                result = 'r6'   # right sensing
            else:
                result = 'r8'   # nothing on the right
        elif experiment.get_label() == 'e6':
            black_board.left_status()
            if black_board.Right:
                result = 'r7'   # left sensing
            else:
                result = 'r9'   # nothing on the left

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
        if experiment == 'e1':
            black_board.adv_distance = 1.0
            black_board.adv_angle = 0.0
            black_board.move_adv()
            if black_board.move_fail:
                result = 'r1'  # moved forward
            else:
                result = 'r10' # move failed
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
                result = 'r4'  # front free
            else:
                result = 'r5'  # front busy
        elif experiment == 'e5':
            black_board.right_status()
            if black_board.Right:
                result = 'r6'   # right sensing
            else:
                result = 'r8'   # nothing on the right
        elif experiment == 'e6':
            black_board.left_status()
            if black_board.Right:
                result = 'r7'   # left sensing
            else:
                result = 'r9'   # nothing on the left

        enacted_interaction = experiment+result
        self.last_interaction = enacted_interaction

        return enacted_interaction

if __name__ == '__main__':
    #pdb.set_trace()
    # run with  i.e. rosrun eca_agent00.py constructive
    parser = argparse.ArgumentParser()
    parser.add_argument("mechanism", type=str, help="specify the learning mechanism to be used",
                        choices=["simple", "recursive", "constructive"])
    args = parser.parse_args()
    
    black_board.agent_mechanism = args.mechanism
    
    tree = EcaAgent00()
  