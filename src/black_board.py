#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed May 10 20:18:58 2017

@author: juan
"""
import pdb

from black_board_class import black_board_object
import rospy
import numpy as np
import scipy.optimize as sci
import math
import pylab
from geometry_msgs.msg import Twist, Point32

from advance import advance
import sensor_msgs.msg
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle

import display_lines
from adaptor001.msg import ExtractedLine
from adaptor001.msg import ExtractedLines

from fancy_prompts import bcolors

def print_position():
        print black_board_object.move_count
        print black_board_object.agent_position
        print black_board_object.agent_rotation
    
def plotter(y, name):    
    """
    Uncomment these next few lines to display a constantly updating graph of data.
    Note: when the graph appears, you must close it first, and then the graph will 
    reopen and display the data as it comes in.
    """
    x = []
    x = range(len(y))
    pylab.figure(name)
    pylab.clf()
    pylab.plot(x,y, color = 'black')
    pylab.plot(x,y, 'bo')
    pylab.draw()
    pylab.show()
    #raw_input("Press a key to continue...")
    
def regression(y):
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
    
def right_wall_param(y, start_index, end_index):
    #pdb.set_trace()
    coefficients = regression(y)
    
    first_scan_point = Point32()
    last_scan_point = Point32()
    dist = y[0]
    angle = black_board_object.angle_min + start_index * black_board_object.angle_increment
    if dist <= black_board_object.maximum_range:
        first_scan_point.x = dist * math.cos(angle)
        first_scan_point.y = dist * math.sin(angle)
    dist = y[end_index]
    angle = black_board_object.angle_min + end_index * black_board_object.angle_increment
    if dist <= black_board_object.maximum_range:
        last_scan_point.x = dist * math.cos(angle)
        last_scan_point.y = dist * math.sin(angle)
    # approximate wall_angle (degrees)
    black_board_object.right_wall_angle = normalize_angle(math.atan(coefficients[0]))*360./(2*math.pi)
    # approximate dist to wall (meters)
    black_board_object.distance_to_right_wall = coefficients[1]
    return coefficients, first_scan_point, last_scan_point
    
def left_wall_param(y, start_index, end_index):
    coefficients = regression(y)
    
    first_scan_point = Point32()
    last_scan_point = Point32()
    dist = y[-1]
    black_board_object.angle_min = black_board_object.angle_min + 574 * black_board_object.angle_increment
    angle = black_board_object.angle_min + start_index * black_board_object.angle_increment
    if dist <= black_board_object.maximum_range:
        first_scan_point.x = dist * math.cos(angle)
        first_scan_point.y = dist * math.sin(angle)
    dist = y[end_index]
    angle = black_board_object.angle_min + end_index * black_board_object.angle_increment
    if dist <= black_board_object.maximum_range:
        last_scan_point.x = dist * math.cos(angle)
        last_scan_point.y = dist * math.sin(angle)
    # approximate wall_angle (degrees)
    black_board_object.left_wall_angle = normalize_angle(math.atan(coefficients[0]))*360./(2*math.pi)
    # approximate dist to wall (meters)
    black_board_object.distance_to_left_wall = coefficients[1]
    
def display_callback(lines_msg):
    """
    Callback for extracted lines, displayed by publishing to
    vis_lines_topic and vis_scanpoints_topic.
    """
    display_lines.create_lines_marker(lines_msg)
    display_lines.create_scanpoints_marker(lines_msg)
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
    black_board_object.selected_lines_publisher.publish(sel_lines)
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
         fo = black_board_object.follow_offset
         fa = black_board_object.follow_advance
         goalx = (line.c - fo) * math.cos(line.m) + fa * math.cos(line.m + math.pi/2)
         goaly = (line.c - fo) * math.sin(line.m) + fa * math.sin(line.m + math.pi/2)
         black_board_object.adv_distance = math.sqrt(math.pow(goalx,2)+math.pow(goaly,2))
         black_board_object.adv_angle = math.atan(goaly/goalx)
    # Publish the twist message produced by the controller.
    if line != None:
        rospy.loginfo("Stopping the agent...")
        black_board_object.cmd_vel_publisher.publish(Twist())
        rospy.sleep(1)
        
    
def extract_lines(y, status = 'R'):
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
        if y[i] < black_board_object.maximum_range and y[i] != 0.0:
            if not done_si:
                start_index = i
                done_si = True
            end_index = i
    if status == 'R':
        coefficients, first_scan_point, last_scan_point = right_wall_param(y, start_index, end_index)
    elif status == 'L':
        coefficients, first_scan_point, last_scan_point = left_wall_param(y, start_index, end_index)
    line.m = coefficients[0]
    line.c = coefficients[1]
    line.firstScanPoint = first_scan_point
    line.lastScanPoint = last_scan_point
    if line.c >= 4.99:
        line = None

    if line is not None:
        lines.lines.append(line)
        black_board_object.lines = lines.lines
    print "LINES: "
    print black_board_object.lines
    
    raw_input('Enter to continue')
    
    # Subscribe to 'lines_topic'
    rospy.Publisher('/extracted_lines', ExtractedLines, queue_size=10).publish(black_board_object.lines)
    #rospy.loginfo("Waiting for lines_topic...")
    #rospy.wait_for_message('/extracted_lines', ExtractedLines)
    rospy.Subscriber('/extracted_lines', ExtractedLines, display_callback, queue_size=10)
        
    rospy.sleep(1)
    black_board_object.lines.append(line)
    return line
        
def laser_scan():
    #rospy.loginfo("Waiting for /base_scan topic...")
    rospy.wait_for_message('/base_scan', sensor_msgs.msg.LaserScan)
    # Subscribe the /base_scan topic to get the range readings  
    rospy.Subscriber('/base_scan', sensor_msgs.msg.LaserScan, scan_callback, queue_size = 10)
    rospy.sleep(0.1)
    black_board_object.distance_front = min(black_board_object.kinect_scan[300:338])
    if black_board_object.distance_front <= 1.7:
        black_board_object.driving_forward = False
    else:
        if black_board_object.Right1:
            black_board_object.adv_distance = black_board_object.distance_front - 1.7
        elif black_board_object.Right2 or black_board_object.Right3:
            black_board_object.adv_distance = 1.0
        else:
            black_board_object.adv_distance = 0.0
            
        black_board_object.driving_forward = True
    #rospy.loginfo("laser_scan done")
    
def scan_callback(msg):
    black_board_object.kinect_scan = list(msg.ranges) # transformed to list since msg.ranges is a tuple
    black_board_object.angle_min = msg.angle_min
    black_board_object.angle_increment = msg.angle_increment
    
def formule(L, tolerance):
    f = 0
    f = L[1] - (L[0] + L[2])/2
    f = round_to_zero(f, tolerance)
    return f
    
def moving_window_filtro(x, tolerance=0.2, n_neighbors=1):
    n = len(x)
    width = n_neighbors*2 + 1
    x = [x[0]]*n_neighbors + x + [x[-1]]*n_neighbors
    # To complete the function,
    # return a list of the filtered values from i to i+width for all values i from 0 to n-1.
    filtro = []
    singularity = []
    for i in range(n):
        fi = abs(formule(x[i:i+width], tolerance))
        filtro.append(fi)
        if fi != 0.0:
            singularity.append(i)
        
    return filtro, singularity

def round_to_zero(val, tolerance):
    if -1 * tolerance < val < tolerance:
        return 0
    else:
        return val
    
def arrange():
    # makes the robot turn to adopt a rotation angle of 0, +-(pi/2) or pi 
    agent_rotation_angle = math.degrees(normalize_angle(quat_to_angle(black_board_object.agent_rotation)))
    # pdb.set_trace()
    angle_rad_rot = math.radians(agent_rotation_angle)
    if abs(agent_rotation_angle) < 45:
        turn = -angle_rad_rot
        print agent_rotation_angle, math.degrees(turn)
        (black_board_object.agent_position, black_board_object.agent_rotation) = advance(0.0, turn, da=True)
    if abs(agent_rotation_angle) < 90 and abs(agent_rotation_angle) >= 45:
        turn = sign(angle_rad_rot)*abs(math.pi/2 - abs(angle_rad_rot))
        print agent_rotation_angle, math.degrees(turn)
        (black_board_object.agent_position, black_board_object.agent_rotation) = advance(0.0, turn, da=True)
    if abs(agent_rotation_angle) < 135 and abs(agent_rotation_angle) >= 90:
        turn = -sign(angle_rad_rot)*abs(math.pi/2 - abs(angle_rad_rot))
        print agent_rotation_angle, math.degrees(turn)
        (black_board_object.agent_position, black_board_object.agent_rotation) = advance(0.0, turn, da=True)
    if abs(agent_rotation_angle) >= 135:
        turn = sign(angle_rad_rot)*abs(math.pi - abs(angle_rad_rot))
        print agent_rotation_angle, math.degrees(turn)
        (black_board_object.agent_position, black_board_object.agent_rotation) = advance(0.0, turn, da=True)
    agent_rotation_angle = math.degrees(normalize_angle(quat_to_angle(agent_rotation)))
    return agent_rotation_angle
    
def move_adv():
    rospy.loginfo("Estoy en move advance")
    #pdb.set_trace()
    try:
        #raw_input("Press a key to continue...")
        if black_board_object.move_count == 0:
            print bcolors.OKGREEN + "PREPARING THINGS IN MY PLACE" + bcolors.ENDC
            # agent_pose is a tuple (position, rotation)
            (black_board_object.agent_position, black_board_object.agent_rotation) = advance(0.0, 0.0, da=True)
            print_position()
            black_board_object.waypoints.append((black_board_object.agent_position, black_board_object.agent_rotation))
            black_board_object.move_count += 1
            rospy.loginfo("move_adv done.")
            return 1
        
        laser_scan()
        if black_board_object.driving_forward or black_board_object.adv_distance == 0.0:
            if abs(black_board_object.adv_angle) < math.radians(2):    
                (black_board_object.agent_position, black_board_object.agent_rotation) = advance(black_board_object.adv_distance, 0.0, da=True)
            else:
                (black_board_object.agent_position, black_board_object.agent_rotation) = advance(black_board_object.adv_distance, black_board_object.adv_angle, da=True)
            black_board_object.adv_angle = 0.0
            print_position()
#            raw_input("Press a key to continue...")
            black_board_object.waypoints.append((black_board_object.agent_position, black_board_object.agent_rotation))
            black_board_object.move_count += 1
            rospy.loginfo("move_adv done.")
            black_board_object.move_fail = False
        else:
            rospy.loginfo("move_adv failed.")
            (black_board_object.agent_position, black_board_object.agent_rotation) = advance(0.0, 0.0, da=True)
            black_board_object.move_fail = True
    except:
        rospy.loginfo("move_adv failed.")
        (black_board_object.agent_position, black_board_object.agent_rotation) = advance(0.0, 0.0, da=True)
        black_board_object.move_fail = True
        return 1
    return 1

def right_status():
    rospy.loginfo("Estoy en right status")
    
    laser_scan()
    rospy.sleep(1)
    if black_board_object.distance_front < 2.5:
        (black_board_object.agent_position, black_board_object.agent_rotation) = advance(0.0, math.pi/2, da=True)
        laser_scan()
        rospy.sleep(1)
    y = list()
    y = black_board_object.kinect_scan
    
    #plotter(y[0:200], "Right")
    
    filtered_scan, singularities = moving_window_filtro(y[0:200], tolerance=0.1, n_neighbors=1)
    print singularities
#        if len(singularities) != 0:
#            plotter(filtered_scan, "Right-filtered")
    
    y1 = y[0:65]
    y2 = y[65:130]
    y3 = y[130:200]
    
    extract_lines(y1, status = 'R')
    
    black_board_object.agent_rotation_angle = arrange() 
    
    print "odom_angle: ", black_board_object.agent_rotation_angle
    print "wall_angle: ", black_board_object.right_wall_angle
    print bcolors.OKGREEN + "distance to right wall(coefficients[1]): " +  str(black_board_object.distance_to_right_wall) + bcolors.ENDC
    print bcolors.OKGREEN + "distance to front obstacle: " +  str(black_board_object.distance_front) + bcolors.ENDC
    #raw_input("Press ENTER to continue...")
    if black_board_object.distance_to_right_wall < 4.5:
        rospy.loginfo(bcolors.OKGREEN + "Right1 sensing" + bcolors.ENDC)
        black_board_object.Right1 = True
        black_board_object.adv_distance = black_board_object.distance_front - 1.7            
        #pdb.set_trace()
    else:
        rospy.loginfo(bcolors.OKGREEN + "Nothing on the right1" + bcolors.ENDC)
        black_board_object.adv_distance = 0.0
        black_board_object.Right1 = False
        #Right_Corner = False
        
    av_distance2 = sum(y2)/65.
    if av_distance2 < 4.5:
        rospy.loginfo(bcolors.OKGREEN + "Right2 sensing" + bcolors.ENDC)
        black_board_object.Right2 = True
    else:
        rospy.loginfo(bcolors.OKGREEN + "Nothing on the right2" + bcolors.ENDC)
        black_board_object.Right2 = False
        
    av_distance3 = sum(y3)/70.
    if av_distance3 < 4.5:
        rospy.loginfo(bcolors.OKGREEN + "Right3 sensing" + bcolors.ENDC)
        black_board_object.Right3 = True
    else:
        rospy.loginfo(bcolors.OKGREEN + "Nothing on the right3" + bcolors.ENDC)
        black_board_object.Right3 = False
    
    return 1

def left_status():
    rospy.loginfo("Estoy en left status")
    laser_scan()
    rospy.sleep(2)
    
    y = list()
    y = black_board_object.kinect_scan
    #plotter(kinect_scan)
    
    y1 = y[574:639]
    y2 = y[508:574]
    y3 = y[438:508]
    
    extract_lines(y1, status = 'L')
    
    agent_rotation_angle = arrange()  
    print "odom_angle: ",  math.degrees(normalize_angle(agent_rotation_angle))
    print "wall_angle: ", black_board_object.left_wall_angle
    print "distance to left wall(coefficients[1]): ", black_board_object.distance_to_left_wall
    print bcolors.OKGREEN + "distance to front obstacle: " +  str(black_board_object.distance_front) + bcolors.ENDC
#        raw_input("Press a key to continue...")
    if black_board_object.distance_to_left_wall < 4.5:
        rospy.loginfo(bcolors.OKGREEN + "Left1 sensing" + bcolors.ENDC)
        black_board_object.Left1 = True
        if black_board_object.Left1:
            black_board_object.adv_distance = black_board_object.distance_front - 1.7
        else:
            black_board_object.adv_distance = 0.0
    else:
        rospy.loginfo(bcolors.OKGREEN + "Nothing on the left1" + bcolors.ENDC)
        black_board_object.Left1 = False

    av_distance2 = sum(y2)/65.
    if av_distance2 < 4.5:
        rospy.loginfo(bcolors.OKGREEN + "Left2 sensing" + bcolors.ENDC)
        black_board_object.Left2 = True
    else:
        rospy.loginfo(bcolors.OKGREEN + "Nothing on the left2" + bcolors.ENDC)
        black_board_object.Left2 = False
        
    av_distance3 = sum(y3)/70.
    if av_distance3 < 4.5:
        rospy.loginfo(bcolors.OKGREEN + "Left3 sensing" + bcolors.ENDC)
        black_board_object.Left3 = True
    else:
        rospy.loginfo(bcolors.OKGREEN + "Nothing on the left3" + bcolors.ENDC)
        black_board_object.Left3 = False
        
    return 1

def sign(x):
    if x < 0.0:
        return -1
    elif x == 0.0:
        return 0
    elif x > 0.0:
        return 1
