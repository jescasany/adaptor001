#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 02 2017
Last visited on 25/03/2017
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
(black_board.robot_position, black_board.robot_rotation) = black_board.robot_pose 
black_board.odom_angle = 0.0

black_board.chs = 1     # used to change the sign(+/-) of the PI/2 turn

class Pruebas():
    def __init__(self):
        #pdb.set_trace()
        rospy.init_node("pruebas_tree")

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        self.laser_scan()
        rate = rospy.Rate(10)
        # Create the root node
        BEHAVE = Sequence("BEHAVE")

        MOVE_AVOID = Selector("MOVE AVOID") 
        
        I_F_IS_VISITED =IgnoreFailure("I_F IS VISITED")
        
        IS_VISITED = CallbackTask("is visited", self.is_visited)
        
        IS_POSSIBLE = Sequence("IS POSSIBLE")
        
        # The move advance task (uses CallbackTask)
        MOVE_ADV = CallbackTask("move_adv", self.move_adv)
        
        RIGHT_SENSING = CallbackTask("right sensing", self.right_status)
        FRONT_STATUS = CallbackTask("front status", self.front_status)
        
        SINGULARITIES = Selector("SINGULARITIES")
        
        SHORT_RIGHT = Selector("SHORT_RIGHT")
        
        SAME = CallbackTask("right sensing -- left appears", self.same)
        
        SMALL_CROSS = CallbackTask("nothing on right -- left appears", self.small_cross)
        FRONT_LEFT_RIGHT = CallbackTask("nothing on right -- right and left appear", self.front_left_right)
        FRONT_RIGHT = CallbackTask("nothing on right -- right appears", self.front_right)
        FRONT_ALL = CallbackTask("nothing on right -- all front appears", self.front_all)
        FRONT_NONE = CallbackTask("nothing on right nor front", self.front_none)
        
        CORNER = CallbackTask("CORNER", self.corner)
        
        
        TURN_BACK = CallbackTask("TURN_BACK", self.turn_back)
        
        # Add the subtrees to the root node in order of priority
        BEHAVE.add_child(MOVE_AVOID)
        BEHAVE.add_child(I_F_IS_VISITED)
        BEHAVE.add_child(IS_POSSIBLE)
        BEHAVE.add_child(MOVE_ADV)
        BEHAVE.add_child(SINGULARITIES)
        
        I_F_IS_VISITED.add_child(IS_VISITED)
        
        IS_POSSIBLE.add_child(RIGHT_SENSING)
        IS_POSSIBLE.add_child(FRONT_STATUS)
        IS_POSSIBLE.add_child(TURN_BACK)
        
        SINGULARITIES.add_child(SAME)
        SINGULARITIES.add_child(SHORT_RIGHT)
        SINGULARITIES.add_child(CORNER)
        
        SHORT_RIGHT.add_child(SMALL_CROSS)
        SHORT_RIGHT.add_child(FRONT_LEFT_RIGHT)
        SHORT_RIGHT.add_child(FRONT_RIGHT)
        SHORT_RIGHT.add_child(FRONT_ALL)
        SHORT_RIGHT.add_child(FRONT_NONE)
        
        # Add the front free and move advance tasks to the "move avoid" task
        with MOVE_AVOID:
            # The front free condition (uses CallbackTask)
            FRONT_FREE = CallbackTask("front free", self.front_free)
            
            AVOID = CallbackTask("AVOID", self.avoid)
            
            # Add the front free and move advance tasks to the move avoid selector
            MOVE_AVOID.add_child(FRONT_FREE)
            MOVE_AVOID.add_child(AVOID)
            
        
        # Display the tree before beginning execution
        print bcolors.HEADER + "Pruebas Behavior Tree" + bcolors.ENDC
        print_tree(BEHAVE, indent=0, use_symbols=True)
        print_dot_tree(BEHAVE, dotfilepath='/home/juan/catkin_ws/src/adaptor001/tree.dot')
        # Run the tree
        while not rospy.is_shutdown():
            BEHAVE.run()
            rate.sleep()

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
              
    def move_adv(self):
        rospy.loginfo("Estoy en move advance")
        #pdb.set_trace()
        try:
            #raw_input("Press a key to continue...")
            if black_board.move_count == 0:
                print bcolors.OKGREEN + "PREPARING THINGS IN MY PLACE" + bcolors.ENDC
                # black_board.robot_pose is a tuple (position, rotation)
                (black_board.robot_position, black_board.robot_rotation) = advance(0.0, 0.0, da=True)
                black_board.print_position()
                black_board.waypoints.append((black_board.robot_position, black_board.robot_rotation))
                black_board.move_count += 1
                rospy.loginfo("move_adv done.")
                return 1
            
            self.laser_scan()
            if black_board.driving_forward:
                (black_board.robot_position, black_board.robot_rotation) = advance(black_board.adv_distance, 0.0, da=True)
                black_board.adv_angle = 0.0
                black_board.print_position()
    #            raw_input("Press a key to continue...")
                black_board.waypoints.append((black_board.robot_position, black_board.robot_rotation))
                black_board.move_count += 1
                rospy.loginfo("move_adv done.")
        except:
            rospy.loginfo("move_adv failed.")
            return 0
        return 1
    
    def is_visited(self):
        if black_board.move_count == 0:
            rospy.loginfo("Waypoint is not visited.")
            return False
        if (black_board.robot_position, black_board.robot_rotation) in black_board.waypoints[0:-1]:
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
        robot_rotation_angle = quat_to_angle(black_board.robot_rotation)
        print "odom_angle: ", math.degrees(normalize_angle(robot_rotation_angle))
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
        robot_rotation_angle = quat_to_angle(black_board.robot_rotation)
        print "odom_angle: ",  math.degrees(normalize_angle(robot_rotation_angle))
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

    def same(self):
        rospy.loginfo("Estoy en same")
        #pdb.set_trace()
        #self.left_status()
        if black_board.Left == True and black_board.Right == True and black_board.Front == True:
            print bcolors.OKGREEN + "SAME: right sensing -- left appears at " + str(black_board.Left_Length) + bcolors.ENDC
            return True
        else:
            return False
    
    def small_cross(self):
        rospy.loginfo("Estoy en small cross")
        #pdb.set_trace()
        #self.left_status()
        if black_board.Left == True and black_board.Right == False and black_board.Front == True:
            print bcolors.OKGREEN + "nothing on right -- left appears at " + str(black_board.Left_Length) + bcolors.ENDC
            return True
        else:
            return False
    
    def front_left_right(self):
        rospy.loginfo("Estoy en front left right")
        #pdb.set_trace()
        #self.left_status()
        if black_board.Left == True and black_board.Right == True and black_board.Front == True:
            print bcolors.OKGREEN + "nothing on right -- right and left appear at " + str(black_board.corner_dist) + " and at " + str(black_board.Left_Length) + bcolors.ENDC
            return True
        else:
            return False
    
    def front_right(self):
        rospy.loginfo("Estoy en front right")
        #pdb.set_trace()
        #self.left_status()
        if black_board.Left == False and black_board.Right == True and black_board.Front == True and len(black_board.right_singularity) > 1:
            print bcolors.OKGREEN + "nothing on right -- right appears" + bcolors.ENDC
            return True
        else:
            return False
        
    def front_all(self):
        rospy.loginfo("Estoy en front all")
        #pdb.set_trace()
        #self.left_status()
        if black_board.Left == True and black_board.Right == False and black_board.Front_All == False:
            print bcolors.OKGREEN + "nothing on right -- wall in front appears" + bcolors.ENDC
            self.laser_scan()
            if black_board.driving_forward:
                advance(1.0, -math.pi/2)
            return True
        else:
            return False
    
    def front_none(self):
        rospy.loginfo("Estoy en front none")
        #pdb.set_trace()
        #self.left_status()
        if black_board.Left == False and black_board.Right == False and black_board.Front_All == True and black_board.Right_Corner == True:
            print bcolors.OKGREEN + "ON RIGHT CORNER: nothing on right nor front" + bcolors.ENDC
            self.laser_scan()
            if black_board.driving_forward:
                advance(3.0, -math.pi/2)
            black_board.Right_Corner == False
            return True
        else:
            return False
    
    def corner(self):
        rospy.loginfo("Estoy en corner")
        #pdb.set_trace()
        #self.left_status()
        if black_board.Left == True and black_board.Right == True and black_board.Front_All == False:
            print bcolors.OKGREEN + "IN CORNER: both right and front blocked" + bcolors.ENDC
            self.laser_scan()
            if black_board.driving_forward:
                advance(1.0, math.pi/2)
            return True
        else:
            return False
        
    def avoid(self):
        rospy.loginfo("Estoy en AVOID")
        #pdb.set_trace()
        self.laser_scan()
        if black_board.driving_forward:
            return 1
        while not black_board.driving_forward:
            if black_board.Right and black_board.Left:
                advance(0.0, math.pi/2)
            elif black_board.Right:
                advance(0.0, math.pi/2)
            elif black_board.Left:
                advance(0.0, -math.pi/2)
            else:
                advance(0.0, math.pi/2)
            self.laser_scan()
        return 1
        
    def turn_back(self):
        rospy.loginfo("Estoy en turn back")
        #pdb.set_trace()
        if (black_board.distance_to_right_wall < 1.3 or black_board.distance_to_right_wall > 2.0) and black_board.last_distance != black_board.distance_to_right_wall:
            if black_board.distance_to_left_wall < 1.5:
                m = (black_board.distance_to_left_wall + black_board.distance_to_right_wall)/2
                if m < 1.5:
                    black_board.adv_angle = math.pi/2
                    black_board.adv_distance = 1.5 - m - 0.8   # minus distance travelled while turning
                    black_board.adv_distance  = self.clamp(black_board.adv_distance, 0.0, 2.0)  
                    self.go()
                return 1
            if black_board.distance_to_right_wall > 2.0:
                black_board.adv_angle = -math.pi/2
                black_board.adv_distance = black_board.distance_to_right_wall - 1.5 - 0.8
                black_board.adv_distance  = self.clamp(black_board.adv_distance, 0.0, 2.0)
                self.go()
            elif black_board.distance_to_right_wall < 1.3:
                black_board.adv_angle = math.pi/2
                black_board.adv_distance = 1.5 - black_board.distance_to_right_wall
                black_board.adv_distance  = self.clamp(black_board.adv_distance, 0.0, 2.0)
                self.go()
            black_board.last_distance = black_board.distance_to_right_wall
            return 1
        else:
            return 1
    
    def go(self):
        print "Moving " + str(math.degrees(black_board.adv_angle)) + " deg   " +  "0.0 m"
        (black_board.robot_position, black_board.robot_rotation) = advance(0.0, black_board.adv_angle)
        self.laser_scan()
        if black_board.driving_forward:
            print "Moving " +  "0.0 deg   " + str(black_board.adv_distance) + " m "  
            (black_board.robot_position, black_board.robot_rotation) = advance(black_board.adv_distance, 0.0)
        else:
            print bcolors.OKRED + "CAN'T ADVANCE - front blocked" + bcolors.ENDC
        print "Moving " + str(math.degrees(black_board.adv_angle)) + " deg   " + "0.0 m "
        (black_board.robot_position, black_board.robot_rotation) = advance(0.0, -black_board.adv_angle)
        black_board.adv_angle = 0.0
        black_board.adv_distance = 1.0
        return 1
        
    def laser_scan(self):
        rospy.loginfo("Waiting for /base_scan topic...")
        rospy.wait_for_message('/base_scan', sensor_msgs.msg.LaserScan)
        # Subscribe the /base_scan topic to get the range readings  
        rospy.Subscriber('/base_scan', sensor_msgs.msg.LaserScan, self.scan_callback, queue_size = 10)
        rospy.sleep(0.1)
        if min(black_board.kinect_scan) < 0.8:
            black_board.driving_forward = False
        else:
            black_board.driving_forward = True
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