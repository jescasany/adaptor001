#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed May 10 11:14:29 2017

@author: juan
"""

import pdb

import rospy
import numpy as np
import scipy.optimize as sci
import math
import pylab
from geometry_msgs.msg import Twist, Point, Quaternion, Point32

from advance import advance
import sensor_msgs.msg
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle

import display_lines
from adaptor001.msg import ExtractedLine
from adaptor001.msg import ExtractedLines
from visualization_msgs.msg import Marker

from fancy_prompts import bcolors

""" A class to track black_board.variables """
class BlackBoard:
    def __init__(self):
        # Initialize a number of variables for the blackboard
        self.kinect_scan = list()
        self.filtered_scan = list()
        # Initialize distance and angle to advance
        self.adv_distance = 1.0      # meters
        self.adv_angle = 0.0     # radians
        
        self.driving_forward = True      # is True if there is no obstacle ahead 
        self.move_fail = False       #  whether move_adv fails or not
        
        self.distance_to_right_wall = 1.5
        self.last_distance = 1.5
        self.distance_to_left_wall = 5.0
        self.left_wall_angle = 0.0
        self.distance_to_front = 7.0
        self.Left1 = False
        self.Left2 = False
        self.Left3 = False
        self.Right1 = False
        self.Right2 = False
        self.Right3 = False
        # The agent's current position on the map
        self.agent_position = Point()
        self.agent_rotation = Quaternion()
        self.agent_pose = (Point(), Quaternion())
        # Create a dictionary to hold navigation waypoints
        self.waypoints = list()
        (self.agent_position, self.agent_rotation) = self.agent_pose 
        self.odom_angle = 0.0
        self.agent_mechanism = ''    # to choose among simple, recursive and constructive mechanisms
        self.boredom = False
        self.lines = list()
        
        self.follow_offset = rospy.get_param('follow_offset')
        self.follow_advance = rospy.get_param('follow_advance')
        
        self.lines_topic = rospy.get_param('lines_topic')
        self.vis_lines_topic = rospy.get_param('vis_lines_topic')
        self.vis_scanpoints_topic = rospy.get_param('vis_scanpoints_topic')
        # Read parameters managed by the ROS parameter server
        self.orthog_distance_threshold = rospy.get_param('orthog_distance_threshold')
        self.min_points_per_line = rospy.get_param('min_points_per_line')
        self.maximum_range = rospy.get_param('maximum_range')
        
        self.scan = list()
        
        self.cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # Create our publisher for the lines extracted within 'base_scan' callback
        self.extracted_publisher = rospy.Publisher(self.lines_topic, ExtractedLines, queue_size=10)
        
        # We will publish to vis_lines_topic which will show the lines (as
        # line segments) in RViz.  We will also publish the first and last scan
        # points to topic vis_scanpoints_topic in the same colour so that it
        # is clear from what range of scan points the lines are generated.
        self.lines_publisher = rospy.Publisher(self.vis_lines_topic, Marker, queue_size=10)
        self.scanpoints_publisher = rospy.Publisher(self.vis_scanpoints_topic, Marker, queue_size=10)
        self.selected_lines_publisher = rospy.Publisher('/extracted_lines_wf', \
                               ExtractedLines, queue_size=10)
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
        
    def right_wall_param(self, y, start_index, end_index):
        #pdb.set_trace()
        coefficients = self.regression(y)
        
        first_scan_point = Point32()
        last_scan_point = Point32()
        dist = y[0]
        angle = self.angle_min + start_index * self.angle_increment
        if dist <= self.maximum_range:
            first_scan_point.x = dist * math.cos(angle)
            first_scan_point.y = dist * math.sin(angle)
        dist = y[end_index]
        angle = self.angle_min + end_index * self.angle_increment
        if dist <= self.maximum_range:
            last_scan_point.x = dist * math.cos(angle)
            last_scan_point.y = dist * math.sin(angle)
        # approximate wall_angle (degrees)
        self.right_wall_angle = normalize_angle(math.atan(coefficients[0]))*360./(2*math.pi)
        # approximate dist to wall (meters)
        self.distance_to_right_wall = coefficients[1]
        return coefficients, first_scan_point, last_scan_point
        
    def left_wall_param(self, y, start_index, end_index):
        coefficients = self.regression(y)
        
        first_scan_point = Point32()
        last_scan_point = Point32()
        dist = y[-1]
        angle_min = self.angle_min + 574 * self.angle_increment
        angle = angle_min + start_index * self.angle_increment
        if dist <= self.maximum_range:
            first_scan_point.x = dist * math.cos(angle)
            first_scan_point.y = dist * math.sin(angle)
        dist = y[end_index]
        angle = angle_min + end_index * self.angle_increment
        if dist <= self.maximum_range:
            last_scan_point.x = dist * math.cos(angle)
            last_scan_point.y = dist * math.sin(angle)
        # approximate wall_angle (degrees)
        self.left_wall_angle = normalize_angle(math.atan(coefficients[0]))*360./(2*math.pi)
        # approximate dist to wall (meters)
        self.distance_to_left_wall = coefficients[1]
        
    def display_callback(self, lines_msg):
        """
        Callback for extracted lines, displayed by publishing to
        vis_lines_topic and vis_scanpoints_topic.
        """
        display_lines.create_lines_marker(lines_msg, self)
        display_lines.create_scanpoints_marker(lines_msg, self)
        """
        The first thing to do is account for the fact that the lines are
        extracted in the 'odom' reference frame.  The laser scanner is installed
        upside-down.  We could use tf to account for the difference, but all that
        we need to adjust is to minus pi/2 all of the m angles for each extracted
        line.  The translation of the laser from the centre of the robot
        shouldn't have an impact on the algorithm here other than changing the
        interpretation of the 'follow-offset' parameter.
        """
        #pdb.set_trace()
        for l in lines_msg.lines:
            l.m = l.m - math.pi/2
        """
        Set 'line' to be the closest line to the robot.  If the closest line is
        not ahead of the robot or on it's right side then the 'line' variable will
        be reset to 'None' indicating there is no suitable line to follow.  We
        choose a range of m values to represent such lines.
        """
        line = None
        smallestR = float('inf')
        for l in lines_msg.lines:
            if l.c < smallestR:
                smallestR = l.c
                line = l
        """
        If this closest line is in the right angular range, we will use it below
        to generate a goal position.  Otherwise, we will ignore it and keep with
        any previously set velocity.
        """
        pdb.set_trace()
        
        if line is not None:
            if abs(line.m) > math.pi/2:
                line = None
        """
        Place the closest line into a new ExtractedLines message and publish it on
        topic 'selected_lines'.  This will allow us to see the line selected
        above in rviz.  Note that we create a new line and change the m back
        to its original value.  This is because rviz will display the line w.r.t.
        the 'odom' frame.
        """
        sel_lines = ExtractedLines()
        sel_lines.header.frame_id = lines_msg.header.frame_id
        if line != None:
            sel_line = ExtractedLine()
            sel_line.c = line.c
            sel_line.m = line.m + math.pi/2
            sel_lines.lines.append(sel_line)
        self.selected_lines_publisher.publish(sel_lines)
        """
        The position of the goal in the robot reference frame is specified by
        the values goalx and goaly, the x and y coordinates of the goal in the
        robot reference frame.  These are obtained by summing the following two
        vectors which the instructor will illustrate on the board.  Note that fo
        = follow-offset and fa = follow-advance):
        
        Vector from the origin to the closest point on the line, with a length
        of c (orthogonal distance of the line) - fo
               [(c-fo) cos(m), (c-fo) sin(m)]
        
        Vector along the line, oriented towards counter-clockwise with length fa
               [fa cos(m+pi/2), fa sin(m+pi/2)]
        """
        goalx = 0
        goaly = 0
        if line != None:
             fo = self.follow_offset
             fa = self.follow_advance
             goalx = (line.c - fo) * math.cos(line.m) + fa * math.cos(line.m + math.pi/2)
             goaly = (line.c - fo) * math.sin(line.m) + fa * math.sin(line.m + math.pi/2)
             self.adv_distance = math.sqrt(math.pow(goalx,2)+math.pow(goaly,2))
             self.adv_angle = math.atan(goaly/goalx)
        # Publish the twist message produced by the controller.
        if line != None:
            rospy.loginfo("Stopping the agent...")
            self.cmd_vel_publisher.publish(Twist())
            rospy.sleep(1)
            
        
    def extract_lines(self, y, status = 'R'):
        """
        Extracts lines from the given LaserScan and publishes to /extracted_lines.
        Publish these lines as an ExtractedLines object on the /extracted_lines
        topic.
        """
        #pdb.set_trace()
        # Create an ExtractedLines object and initialize some fields in the header.
        line = ExtractedLine()
        lines = ExtractedLines()
        #lines.header.frame_id = '/odom'
        lines.header.frame_id = '/odom'
        lines.header.stamp = rospy.Time.now()
        # Create our big list of index pairs.  Each pair gives the start and end
        # index which specifies a contiguous set of (scanned) data points in 'y'.
        n = len(y)
        start_index = 0
        end_index = n-1
        done_si = False
        for i in range(n):
            if y[i] < self.maximum_range and y[i] != 0.0:
                if not done_si:
                    start_index = i
                    done_si = True
                end_index = i
        if status == 'R':
            coefficients, first_scan_point, last_scan_point = self.right_wall_param(y, start_index, end_index)
        elif status == 'L':
            coefficients, first_scan_point, last_scan_point = self.left_wall_param(y, start_index, end_index)
        line.m = coefficients[0]
        line.c = coefficients[1]
        line.firstScanPoint = first_scan_point
        line.lastScanPoint = last_scan_point
        if line.c >= 4.99:
            line = None
    
        if line is not None:
            lines.lines.append(line)
            self.lines = lines.lines
        print "LINES: "
        print self.lines
        
        raw_input('Enter to continue')
        
        # Subscribe to 'lines_topic'
        rospy.Publisher('/extracted_lines', ExtractedLines, queue_size=10).publish(lines)
        #rospy.loginfo("Waiting for lines_topic...")
        #rospy.wait_for_message('/extracted_lines', ExtractedLines)
        rospy.Subscriber('/extracted_lines', ExtractedLines, self.display_callback, queue_size=10)
            
        rospy.sleep(1)
        self.lines.append(line)
        return line
            
    def laser_scan(self):
        #rospy.loginfo("Waiting for /base_scan topic...")
        rospy.wait_for_message('/base_scan', sensor_msgs.msg.LaserScan)
        # Subscribe the /base_scan topic to get the range readings  
        rospy.Subscriber('/base_scan', sensor_msgs.msg.LaserScan, self.scan_callback, queue_size = 10)
        rospy.sleep(0.1)
        self.distance_front = min(self.kinect_scan[300:338])
        if self.distance_front <= 1.7:
            self.driving_forward = False
        else:
            if self.Right1:
                self.adv_distance = self.distance_front - 1.7
            elif self.Right2 or self.Right3:
                self.adv_distance = 1.0
            else:
                self.adv_distance = 0.0
                
            self.driving_forward = True
        #rospy.loginfo("laser_scan done")
        
    def scan_callback(self, msg):
        self.kinect_scan = list(msg.ranges) # transformed to list since msg.ranges is a tuple
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        
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
        agent_rotation_angle = math.degrees(normalize_angle(quat_to_angle(self.agent_rotation)))
        # pdb.set_trace()
        angle_rad_rot = math.radians(agent_rotation_angle)
        if abs(agent_rotation_angle) < 45:
            turn = -angle_rad_rot
            print agent_rotation_angle, math.degrees(turn)
            (self.agent_position, self.agent_rotation) = advance(0.0, turn, da=True)
        if abs(agent_rotation_angle) < 90 and abs(agent_rotation_angle) >= 45:
            turn = self.sign(angle_rad_rot)*abs(math.pi/2 - abs(angle_rad_rot))
            print agent_rotation_angle, math.degrees(turn)
            (self.agent_position, self.agent_rotation) = advance(0.0, turn, da=True)
        if abs(agent_rotation_angle) < 135 and abs(agent_rotation_angle) >= 90:
            turn = -self.sign(angle_rad_rot)*abs(math.pi/2 - abs(angle_rad_rot))
            print agent_rotation_angle, math.degrees(turn)
            (self.agent_position, self.agent_rotation) = advance(0.0, turn, da=True)
        if abs(agent_rotation_angle) >= 135:
            turn = self.sign(angle_rad_rot)*abs(math.pi - abs(angle_rad_rot))
            print agent_rotation_angle, math.degrees(turn)
            (self.agent_position, self.agent_rotation) = advance(0.0, turn, da=True)
        agent_rotation_angle = math.degrees(normalize_angle(quat_to_angle(self.agent_rotation)))
        return agent_rotation_angle
        
    def move_adv(self):
        rospy.loginfo("Estoy en move advance")
        #pdb.set_trace()
        try:
            #raw_input("Press a key to continue...")
            if self.move_count == 0:
                print bcolors.OKGREEN + "PREPARING THINGS IN MY PLACE" + bcolors.ENDC
                # self.agent_pose is a tuple (position, rotation)
                (self.agent_position, self.agent_rotation) = advance(0.0, 0.0, da=True)
                self.print_position()
                self.waypoints.append((self.agent_position, self.agent_rotation))
                self.move_count += 1
                rospy.loginfo("move_adv done.")
                return 1
            
            self.laser_scan()
            if self.driving_forward or self.adv_distance == 0.0:
                if abs(self.adv_angle) < radians(2):    
                    (self.agent_position, self.agent_rotation) = advance(self.adv_distance, 0.0, da=True)
                else:
                    (self.agent_position, self.agent_rotation) = advance(self.adv_distance, self.adv_angle, da=True)
                self.adv_angle = 0.0
                self.print_position()
    #            raw_input("Press a key to continue...")
                self.waypoints.append((self.agent_position, self.agent_rotation))
                self.move_count += 1
                rospy.loginfo("move_adv done.")
                self.move_fail = False
            else:
                rospy.loginfo("move_adv failed.")
                (self.agent_position, self.agent_rotation) = advance(0.0, 0.0, da=True)
                self.move_fail = True
        except:
            rospy.loginfo("move_adv failed.")
            (self.agent_position, self.agent_rotation) = advance(0.0, 0.0, da=True)
            self.move_fail = True
            return 1
        return 1
    
    def right_status(self):
        rospy.loginfo("Estoy en right status")
        
        self.laser_scan()
        rospy.sleep(1)
        if self.distance_front < 2.5:
            (self.agent_position, self.agent_rotation) = advance(0.0, math.pi/2, da=True)
            self.laser_scan()
            rospy.sleep(1)
        y = list()
        y = self.kinect_scan
        
        #self.plotter(y[0:200], "Right")
        
        self.filtered_scan, singularities = self.moving_window_filtro(y[0:200], tolerance=0.1, n_neighbors=1)
        print singularities
#        if len(singularities) != 0:
#            self.plotter(self.filtered_scan, "Right-filtered")
        
        y1 = y[0:65]
        y2 = y[65:130]
        y3 = y[130:200]
        
        self.extract_lines(y1, status = 'R')
        
        agent_rotation_angle = self.arrange() 
        
        print "odom_angle: ", agent_rotation_angle
        print "wall_angle: ", self.right_wall_angle
        print bcolors.OKGREEN + "distance to right wall(coefficients[1]): " +  str(self.distance_to_right_wall) + bcolors.ENDC
        print bcolors.OKGREEN + "distance to front obstacle: " +  str(self.distance_front) + bcolors.ENDC
        #raw_input("Press ENTER to continue...")
        if self.distance_to_right_wall < 4.5:
            rospy.loginfo(bcolors.OKGREEN + "Right1 sensing" + bcolors.ENDC)
            self.Right1 = True
            self.adv_distance = self.distance_front - 1.7            
            #pdb.set_trace()
        else:
            rospy.loginfo(bcolors.OKGREEN + "Nothing on the right1" + bcolors.ENDC)
            self.adv_distance = 0.0
            self.Right1 = False
            #self.Right_Corner = False
            
        av_distance2 = sum(y2)/65.
        if av_distance2 < 4.5:
            rospy.loginfo(bcolors.OKGREEN + "Right2 sensing" + bcolors.ENDC)
            self.Right2 = True
        else:
            rospy.loginfo(bcolors.OKGREEN + "Nothing on the right2" + bcolors.ENDC)
            self.Right2 = False
            
        av_distance3 = sum(y3)/70.
        if av_distance3 < 4.5:
            rospy.loginfo(bcolors.OKGREEN + "Right3 sensing" + bcolors.ENDC)
            self.Right3 = True
        else:
            rospy.loginfo(bcolors.OKGREEN + "Nothing on the right3" + bcolors.ENDC)
            self.Right3 = False
        
        return 1
    
    def left_status(self):
        rospy.loginfo("Estoy en left status")
        self.laser_scan()
        rospy.sleep(2)
        
        y = list()
        y = self.kinect_scan
        #self.plotter(self.kinect_scan)
        
        y1 = y[574:639]
        y2 = y[508:574]
        y3 = y[438:508]
        
        self.extract_lines(y1, status = 'L')
        
        agent_rotation_angle = self.arrange()  
        print "odom_angle: ",  math.degrees(normalize_angle(agent_rotation_angle))
        print "wall_angle: ", self.left_wall_angle
        print "distance to left wall(coefficients[1]): ", self.distance_to_left_wall
        print bcolors.OKGREEN + "distance to front obstacle: " +  str(self.distance_front) + bcolors.ENDC
#        raw_input("Press a key to continue...")
        if self.distance_to_left_wall < 4.5:
            rospy.loginfo(bcolors.OKGREEN + "Left1 sensing" + bcolors.ENDC)
            self.Left1 = True
            if self.Left1:
                self.adv_distance = self.distance_front - 1.7
            else:
                self.adv_distance = 0.0
        else:
            rospy.loginfo(bcolors.OKGREEN + "Nothing on the left1" + bcolors.ENDC)
            self.Left1 = False

        av_distance2 = sum(y2)/65.
        if av_distance2 < 4.5:
            rospy.loginfo(bcolors.OKGREEN + "Left2 sensing" + bcolors.ENDC)
            self.Left2 = True
        else:
            rospy.loginfo(bcolors.OKGREEN + "Nothing on the left2" + bcolors.ENDC)
            self.Left2 = False
            
        av_distance3 = sum(y3)/70.
        if av_distance3 < 4.5:
            rospy.loginfo(bcolors.OKGREEN + "Left3 sensing" + bcolors.ENDC)
            self.Left3 = True
        else:
            rospy.loginfo(bcolors.OKGREEN + "Nothing on the left3" + bcolors.ENDC)
            self.Left3 = False
            
        return 1
    
    def sign(self, x):
        if x < 0.0:
            return -1
        elif x == 0.0:
            return 0
        elif x > 0.0:
            return 1
