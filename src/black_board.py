#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed May 10 20:18:58 2017

@author: juan
"""
import pdb

from black_board_class import black_board_object
import rospy

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

from angles import constrain_angle

def print_position():
    print black_board_object.move_count
    print black_board_object.agent_position
    print black_board_object.agent_rotation
    
def plotter((x, y), name):    
    """
    Uncomment these next few lines to display a constantly updating graph of data.
    Note: when the graph appears, you must close it first, and then the graph will 
    reopen and display the data as it comes in.
    """
    pylab.figure(name)
    pylab.clf()
    pylab.plot(x, y, color = 'black')
    pylab.plot(x, y, 'bo')
    pylab.draw()
    pylab.show()
 
def display_callback(lines_msg):
    """
    Callback for extracted lines, displayed by publishing to
    vis_lines_topic and vis_scanpoints_topic.
    """
    print bcolors.OKBLUE + "Estoy en DISPLAY" + bcolors.ENDC
    rospy.sleep(2)
    display_lines.create_lines_marker(lines_msg)
    rospy.sleep(2)
    display_lines.create_scanpoints_marker(lines_msg)
    rospy.sleep(2)

def get_close():
    """
    The position of the goal in the robot reference frame is specified by
    the values goalx and goaly, the x and y coordinates of the goal in the
    robot reference frame.  These are obtained by summing the following two
    vectors which the instructor will illustrate on the board.  Note that fo
    = follow-offset and fa = follow-advance):
    
    Vector from the origin to the closest point on the line, with a length
    of c - fo (c: orthogonal distance of the line) 
           [(c-fo) cos(m), (c-fo) sin(m)]
    
    Vector along the line, oriented towards counter-clockwise with length fa
           [fa cos(m+pi/2), fa sin(m+pi/2)]
    
    The first thing to do is account for the fact that the lines are
    extracted in the 'base_link' reference frame. The translation of the laser from the centre of the robot
    shouldn't have an impact on the algorithm here other than changing the
    interpretation of the 'follow-offset' parameter.

    Set 'line' to be the closest line to the robot.  If the closest line is
    not ahead of the robot or on its right side then the 'line' variable will
    be reset to 'None' indicating there is no suitable line to follow.  We
    choose a range of m values to represent such lines.
    """
    print bcolors.OKGREEN + "Wall Following" + bcolors.ENDC
    line = None
    lines = black_board_object.lines
    smallestR = float('inf')
    for l in lines.lines:
        if l.r < smallestR:
            smallestR = l.r
            line = l

    print bcolors.OKBLUE + "Closest line: " + bcolors.ENDC
    print line
    #raw_input("Press ENTER to continue...")
    """
    If this closest line is in the right angular range, we will use it below
    to generate a goal position.  Otherwise, we will ignore it and keep with
    any previously set velocity.
    """
    #pdb.set_trace()
    
#    if line is not None:
#        if abs(line.alpha) > math.pi/2:
#            line = None
    """
    Place the closest line into a new ExtractedLines message and publish it on
    topic 'selected_lines'.  This will allow us to see the line selected
    above in rviz.  Note that we create a new line and change the r back
    to its original value.  This is because rviz will display the line w.r.t.
    the 'base_link' frame.
    """
    lines.header.frame_id = '/base_link'
    sel_lines = ExtractedLines()
    sel_lines.header.frame_id = lines.header.frame_id
    if line != None:
        sel_line = ExtractedLine()
        sel_line.r = line.r
        sel_line.alpha = line.alpha
        sel_lines.lines.append(sel_line)
        
        black_board_object.selected_lines_publisher.publish(sel_lines)

        fo = black_board_object.follow_offset
        #fa = black_board_object.follow_advance
        fa = black_board_object.distance_front
        dist = max(black_board_object.distance_to_right_wall, line.r) 
        black_board_object.adv_distance = math.sqrt(math.pow(dist - fo, 2) + math.pow(fa, 2))
        th = math.pi/2 - math.atan(fa/abs(dist - fo))
        black_board_object.adv_angle = th
        if (dist - fo) > 0:
            black_board_object.adv_angle = -th
        print bcolors.OKBLUE + "black_board_object.adv_distance: " +  str(black_board_object.adv_distance) + " m" + bcolors.ENDC
        print bcolors.OKBLUE + "black_board_object.adv_angle: " +  str(math.degrees(black_board_object.adv_angle)) + " deg" + bcolors.ENDC 
    
        # Publish the twist message produced by the controller.
        print bcolors.OKBLUE + "STOP the agent before wall following" + bcolors.ENDC
        black_board_object.cmd_vel_publisher.publish(Twist())
        rospy.sleep(2)
        #pdb.set_trace()
        #raw_input("Press ENTER to continue...")
        laser_scan()
        if black_board_object.driving_forward:
            (black_board_object.agent_position, black_board_object.agent_rotation) = advance(black_board_object.adv_distance, black_board_object.adv_angle, da=False)
        else:
           (black_board_object.agent_position, black_board_object.agent_rotation) = advance(0.0, black_board_object.adv_angle, da=False) 
            
        rospy.sleep(2)
        black_board_object.agent_rotation_angle = arrange()
        
def fit_line(scan, start_index, end_index, maximum_range, status):
    """ 
    Fit a line to the given LaserScan yielding an ExtractedLine.
    
    The method of weighted least squares is applied to yield a description of
    the line consisting of the tuple (r, alpha) where 'r' is the orthogonal
    distance of the line to the origin and 'alpha' is the angle that the ray
    perpendicular to the line makes with the origin.  Weighted least squares is
    applied as described in "Introduction to Autonomous Mobile Robots" by
    Siegwart, Nourbakhsh, and Scaramuzza.

    Arguments:
    start_index -- index of the first data point to be used in the line fitting
                   procedure.
    end_index -- index of the last data point to be used in the line fitting
                 procedure.
    maximum_range -- specifies the maximum range of data points which will be
                     incorporated into the fitted line.

    If the scan is empty or there are no points within 'maximum_range' then
    None is returned.
    """
    # First we must calculate alpha, using the formula presented in the
    # course notes (due to Arras, 1998).
    sumWeights = 0
    sumNumL = 0 # The summation term in the numerator on the left
    sumNumR = 0 # The summation term in the numerator on the right
    sumDenL = 0 # The summation term in the denominator on the left
    sumDenR = 0 # The summation term in the denominator on the right
    
    if status == "R":
        angle_min = black_board_object.angle_min
    if status == "L":
        angle_min = black_board_object.angle_min + 574 * black_board_object.angle_increment
    if status == "F":
        angle_min = black_board_object.angle_min + 270 * black_board_object.angle_increment
        
    for i in range(start_index, end_index+1):
        rho = scan[i]
        if rho == 0 or rho > maximum_range:
            continue
        theta = angle_min + i * black_board_object.angle_increment
        weight = 1 / rho**2
        #weight = 1

        sumWeights += weight

        factor1 = weight * rho * rho
        sumNumL += factor1 * math.sin(2 * theta)
        sumDenL += factor1 * math.cos(2 * theta)
        
        for j in range(start_index, end_index+1):
            rho_j = scan[j]
            if rho_j == 0 or rho_j > maximum_range:
                continue
            theta_j = angle_min + j * black_board_object.angle_increment
            weight_j = 1 / rho_j**2
            #weight_j = 1

            factor2 = weight * weight_j * rho * rho_j
            sumNumR += factor2 * math.cos(theta) * math.sin(theta_j)
            sumDenR += factor2 * math.cos(theta + theta_j)

    if sumWeights == 0:
        # There are either no scan points at all, or none within range.
        return None

    sumNumR *= 2.0 / sumWeights
    sumDenR /= sumWeights
    alpha = math.atan2(sumNumL - sumNumR, sumDenL - sumDenR) / 2.0 + math.pi/2
    
    # We now calculate r.
    sumNum = 0 # The summation term in the numerator
    for i in range(start_index, end_index+1):
        rho = scan[i]
        if rho == 0 or rho > maximum_range:
            continue
        theta = angle_min + i * black_board_object.angle_increment
        weight = 1 / rho**2
        #weight = 1

        sumNum += weight * rho * math.cos(theta - alpha)

    r = sumNum / sumWeights
    
#    print "scan: ", scan
#    print "r: ", r

    # Make sure that alpha is in the range (-pi, pi].
    alpha = constrain_angle(alpha - math.pi/2)
    
#    print "alpha: ", alpha
#    
#    raw_input('Enter to continue')
    
    # Determine the first and last points used to estimate this line's
    # parameters.  These two points do not define the line, but they are useful
    # for visualization to show the range of points involved.
    firstScanPoint = Point32()
    lastScanPoint = Point32()
    dist = scan[start_index]
    angle = angle_min + start_index * black_board_object.angle_increment
    if dist <= maximum_range:
        firstScanPoint.x = dist * math.cos(angle)
        firstScanPoint.y = dist * math.sin(angle)
    dist = scan[end_index]
    angle = angle_min + end_index * black_board_object.angle_increment
    if dist <= maximum_range:
        lastScanPoint.x = dist * math.cos(angle)
        lastScanPoint.y = dist * math.sin(angle)

    return ExtractedLine(-r, alpha, firstScanPoint, lastScanPoint)

def right_wall_param(r, start_index, end_index, maximum_range, status):
    #pdb.set_trace()
    line = ExtractedLine()
    line = fit_line(r, start_index, end_index, maximum_range, status)
    if line is not None:
        # approximate wall_angle (degrees)
        black_board_object.right_wall_angle = normalize_angle(line.alpha)*180./math.pi
        # approximate dist to wall (meters)
        dist = min(r)
        black_board_object.distance_to_right_wall = min(dist, abs(line.r))

    return line
    
def left_wall_param(r, start_index, end_index, maximum_range, status):
    line = ExtractedLine()
    line = fit_line(r, start_index, end_index, maximum_range, status)
    if line is not None:
        # approximate wall_angle (degrees)
        black_board_object.left_wall_angle = normalize_angle(line.alpha)*180./math.pi
        # approximate dist to wall (meters)
        dist = min(r)
        black_board_object.distance_to_left_wall = min(dist, abs(line.r))

    return line

def front_wall_param(r, start_index, end_index, maximum_range, status):
    #pdb.set_trace()
    line = ExtractedLine()
    line = fit_line(r, start_index, end_index, maximum_range, status)
    if line is not None:
        # approximate wall_angle (degrees)
        black_board_object.front_wall_angle = normalize_angle(line.alpha)*180./math.pi
        # approximate dist to wall (meters)
        dist = min(r)
        black_board_object.distance_to_front_wall = min(dist, abs(line.r))

    return line

def extract_lines(r, status = 'R'):
    """
    Extracts lines from the given LaserScan and publishes to /extracted_lines.
    Publish these lines as an ExtractedLines object on the /extracted_lines
    topic.
    """
    #pdb.set_trace()
    # Create an ExtractedLines object and initialize some fields in the header.
    line = ExtractedLine()
    # Create our big list of index pairs.  Each pair gives the start and end
    # index which specifies a contiguous set of (scanned) data points in 'y'.
    n = len(r)
    start_index = 0
    end_index = n-1
    done_si = False
    for i in range(n):
        if r[i] < black_board_object.maximum_range and r[i] != 0.0:
            if not done_si:
                start_index = i
                done_si = True
            end_index = i
#    print "status: ", status
#    print r
#    print "start index: ", start_index
#    print "end index: ", end_index
#    raw_input('Enter to continue')
    #pdb.set_trace()
    if status == 'R':
        line = right_wall_param(r, start_index, end_index, black_board_object.maximum_range, status)
        if line is None:
            return line
        if line.r > black_board_object.distance_to_right_wall:
            line = None
    elif status == 'L':
        line = left_wall_param(r, start_index, end_index, black_board_object.maximum_range, status)
        if line is None:
            return line
        if line.r > black_board_object.distance_to_left_wall:
            line = None
    elif status == 'F':
        line = front_wall_param(r, start_index, end_index, black_board_object.maximum_range, status)
        if line is None:
            return line
        if line.r > black_board_object.distance_to_front_wall:
            line = None
    if line is not None:
        if line.r > black_board_object.maximum_range or (abs(line.alpha) > math.radians(10) and abs(line.alpha) < math.radians(80)):
            line = None
    return line

def clear_lines():
    lines = black_board_object.lines
    for l in lines.lines:
        if l.r > black_board_object.distance_to_right_wall or l.r > black_board_object.distance_front:
            black_board_object.lines.lines.remove(l)
            
def line_storage(line):
    if line is not None and line not in black_board_object.lines.lines:
        black_board_object.lines.lines.append(line)
        if len(black_board_object.lines.lines) > 2:
            black_board_object.lines.lines.pop(0)
#    print len(black_board_object.lines.lines)
#    print bcolors.OKGREEN + "LINES: " + bcolors.ENDC
#    print black_board_object.lines
    
    #raw_input('Enter to continue')
    
def line_display():
    # Subscribe to 'lines_topic'
    black_board_object.extracted_publisher.publish(black_board_object.lines)
#    rospy.loginfo("Waiting for lines_topic...")
#    rospy.wait_for_message('/extracted_lines', ExtractedLines)
    rospy.Subscriber('/extracted_lines', ExtractedLines, display_callback, queue_size=10)

    rospy.sleep(1)
        
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
    # To complete the function, return a list of the filtered values  
    # from i to i+width for all values i from 0 to n-1.
    filtro = []
    singularity = []
    last_sing = 0
    for i in range(n):
        fi = abs(formule(x[i:i+width], tolerance))
        filtro.append(fi)
        if fi != 0.0 and (i - last_sing) > 20:
            singularity.append(i)
            last_sing = i
        
    return filtro, singularity

def round_to_zero(val, tolerance):
    if -1 * tolerance < val < tolerance:
        return 0
    else:
        return val
    
def clamp(self, val, minimum, maximum):
        if val < minimum:
            return minimum
        elif val > maximum:
            return maximum
        else:
            return val
    
def arrange():
    laser_scan()
    right_status()
    front_status()
    # makes the robot turn to adopt a rotation angle of 0, +-(pi/2) or pi 
    agent_rotation_angle = math.degrees(normalize_angle(quat_to_angle(black_board_object.agent_rotation)))
    #pdb.set_trace()
    angle_rad_rot = math.radians(agent_rotation_angle)
    if abs(agent_rotation_angle) < 45:
        turn = -angle_rad_rot
        print bcolors.OKBLUE + "agent_rotation_angle: ", str(agent_rotation_angle),  "\n", "degrees(turn): ", str(math.degrees(turn)) + bcolors.ENDC
        (black_board_object.agent_position, black_board_object.agent_rotation) = advance(0.0, turn, da=True)
    if abs(agent_rotation_angle) < 90 and abs(agent_rotation_angle) >= 45:
        turn = sign(angle_rad_rot)*abs(math.pi/2 - abs(angle_rad_rot))
        print bcolors.OKBLUE + "agent_rotation_angle: ", str(agent_rotation_angle),  "\n", "degrees(turn): ", str(math.degrees(turn)) + bcolors.ENDC
        (black_board_object.agent_position, black_board_object.agent_rotation) = advance(0.0, turn, da=True)
    if abs(agent_rotation_angle) < 135 and abs(agent_rotation_angle) >= 90:
        turn = -sign(angle_rad_rot)*abs(math.pi/2 - abs(angle_rad_rot))
        print bcolors.OKBLUE + "agent_rotation_angle: ", str(agent_rotation_angle),  "\n", "degrees(turn): ", str(math.degrees(turn)) + bcolors.ENDC
        (black_board_object.agent_position, black_board_object.agent_rotation) = advance(0.0, turn, da=True)
    if abs(agent_rotation_angle) >= 135:
        turn = sign(angle_rad_rot)*abs(math.pi - abs(angle_rad_rot))
        print bcolors.OKBLUE + "agent_rotation_angle: ", str(agent_rotation_angle),  "\n", "degrees(turn): ", str(math.degrees(turn)) + bcolors.ENDC
        (black_board_object.agent_position, black_board_object.agent_rotation) = advance(0.0, turn, da=True)
    if black_board_object.Right1 == False and black_board_object.Right2 == False and black_board_object.Right3 == False and black_board_object.move_count > 1:
        (black_board_object.agent_position, black_board_object.agent_rotation) = advance(0.0, -math.pi/2, da=True)
    if black_board_object.Right1 == True and black_board_object.Right2 == True and black_board_object.Right3 == True and black_board_object.driving_forward == False:
        (black_board_object.agent_position, black_board_object.agent_rotation) = advance(0.0, math.pi/2, da=True)
    agent_rotation_angle = math.degrees(normalize_angle(quat_to_angle(black_board_object.agent_rotation)))
    return agent_rotation_angle
    
def move_adv():
    print bcolors.OKBLUE + "Estoy en move advance" + bcolors.ENDC
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
            print bcolors.OKBLUE + "Preparation DONE" + bcolors.ENDC
            black_board_object.agent_rotation_angle = arrange()
            return 1
        
        laser_scan()
        rospy.sleep(2)
        if black_board_object.driving_forward or black_board_object.adv_distance == 0.0:
            get_close()
        laser_scan()
        rospy.sleep(2)
        if black_board_object.driving_forward or black_board_object.adv_distance == 0.0:    
            (black_board_object.agent_position, black_board_object.agent_rotation) = advance(black_board_object.adv_distance, 0.0, da=True)
            
            black_board_object.adv_angle = 0.0
            print_position()
#            raw_input("Press a key to continue...")
            black_board_object.waypoints.append((black_board_object.agent_position, black_board_object.agent_rotation))
            black_board_object.move_count += 1
            print bcolors.OKBLUE + "move_adv DONE" + bcolors.ENDC
            black_board_object.move_fail = False
        else:
            print bcolors.OKBLUE + "move_adv FAILED" + bcolors.ENDC
            (black_board_object.agent_position, black_board_object.agent_rotation) = advance(0.0, 0.0, da=True)
            
            black_board_object.move_fail = True
            
    except:
        print bcolors.OKBLUE + "move_adv FAILED" + bcolors.ENDC
        (black_board_object.agent_position, black_board_object.agent_rotation) = advance(0.0, 0.0, da=True)
        black_board_object.move_fail = True
        black_board_object.agent_rotation_angle = arrange()
        return 1
    black_board_object.agent_rotation_angle = arrange()
    return 1


def right_status():
    print bcolors.OKBLUE + "Estoy en right status" + bcolors.ENDC
    laser_scan()
    rospy.sleep(2)
    if black_board_object.move_count == 0:
        return 1
    r = list()
    r = black_board_object.kinect_scan
    black_board_object.lines.header.stamp = rospy.Time.now()
    #plotter(r[0:200], "Right")
    
    filtered_scan, singularities = moving_window_filtro(r[0:200], tolerance=0.1, n_neighbors=1)
    
    r1 = r[0:65]
    r2 = r[65:130]
    r3 = r[130:200]
    if len(singularities) != 0: 
        print bcolors.OKBLUE + "Right singularities: " + repr(singularities) + bcolors.ENDC
        r1 = r[0:singularities[0]]
        if len(singularities) > 1:
            r2 = r[singularities[0]:singularities[1]]
        if len(singularities) > 2:
            r3 = r[singularities[1]:singularities[2]]
#        if len(singularities) != 0:
#            plotter(filtered_scan, "Right-filtered")
    line = ExtractedLine()
    if min(r1) < black_board_object.maximum_range:
        print bcolors.OKBLUE + "Right1 sensing" + bcolors.ENDC
        
        line = extract_lines(r1, status = 'R')

        line_storage(line)
        

        print bcolors.OKBLUE + "odom_angle: " +  str(black_board_object.agent_rotation_angle) + bcolors.ENDC
        print bcolors.OKBLUE + "wall_angle: " +  str(black_board_object.right_wall_angle) + bcolors.ENDC
        print bcolors.OKBLUE + "distance to right wall(rho): " +  str(black_board_object.distance_to_right_wall) + bcolors.ENDC
        print bcolors.OKBLUE + "distance to front obstacle: " +  str(black_board_object.distance_front) + bcolors.ENDC
        black_board_object.Right1 = True
        black_board_object.adv_distance = black_board_object.distance_front - 1.7
    else:
        print bcolors.OKBLUE + "Nothing on the right1" + bcolors.ENDC
        black_board_object.adv_distance = 0.0
        black_board_object.Right1 = False
        
    av_distance2 = sum(r2)/65.
    if av_distance2 < black_board_object.maximum_range:
        print bcolors.OKBLUE + "Right2 sensing" + bcolors.ENDC
        if black_board_object.Right1 == False:
            line = extract_lines(r2, status = 'R')
            line_storage(line)
        
        black_board_object.Right2 = True
    else:
        print bcolors.OKBLUE + "Nothing on the right2" + bcolors.ENDC
        black_board_object.Right2 = False
    #raw_input("Press ENTER to continue...")    
    av_distance3 = sum(r3)/70.
    if av_distance3 < black_board_object.maximum_range:
        print bcolors.OKBLUE + "Right3 sensing" + bcolors.ENDC
        if black_board_object.Right1 == False and black_board_object.Right2 == False:
            line = extract_lines(r3, status = 'R')
            line_storage(line)
        
        black_board_object.Right3 = True
    else:
        print bcolors.OKBLUE + "Nothing on the right3" + bcolors.ENDC
        black_board_object.Right3 = False
    #raw_input("Press ENTER to continue...")
    if line is not None:
        line_display()
        
    return 1

def left_status():
    print bcolors.OKBLUE + "Estoy en left status" + bcolors.ENDC
    laser_scan()
    rospy.sleep(2)
    if black_board_object.move_count == 0:
        return 1
    r = list()
    r = black_board_object.kinect_scan
    #plotter(kinect_scan)
    filtered_scan, singularities = moving_window_filtro(r[438:639], tolerance=0.1, n_neighbors=1)
    if len(singularities) != 0: 
        print bcolors.OKBLUE + "Left singularities: " + repr(singularities) + bcolors.ENDC
#        if len(singularities) != 0:
#            plotter(filtered_scan, "Right-filtered")

    r1 = r[438:508]
    r2 = r[508:574]
    r3 = r[574:639]
    if len(singularities) != 0: 
        print bcolors.OKBLUE + "Left singularities: " + repr(singularities) + bcolors.ENDC
        r1 = r[438:singularities[0]]
        if len(singularities) > 1:
            r2 = r[singularities[0]:singularities[1]]
        if len(singularities) > 2:
            r3 = r[singularities[1]:singularities[2]]
            
    line = ExtractedLine()
    if min(r3) < black_board_object.maximum_range:
        print bcolors.OKGREEN + "Left3 sensing" + bcolors.ENDC
        
        line = extract_lines(r3, status = 'L')
        
        print "odom_angle: ",  black_board_object.agent_rotation_angle
        print "wall_angle: ", black_board_object.left_wall_angle
        print "distance to left wall(coefficients[1]): ", black_board_object.distance_to_left_wall
        print bcolors.OKBLUE + "distance to front obstacle: " +  str(black_board_object.distance_front) + bcolors.ENDC
        black_board_object.Left3 = True
        
        if black_board_object.Left3:
            black_board_object.adv_distance = black_board_object.distance_front - 1.7
        else:
            black_board_object.adv_distance = 0.0
    else:
        print bcolors.OKBLUE + "Nothing on the left3" + bcolors.ENDC
        black_board_object.Left3 = False
    
#        raw_input("Press a key to continue...")

    av_distance2 = sum(r2)/65.
    if av_distance2 < black_board_object.maximum_range:
        print bcolors.OKBLUE + "Left2 sensing" + bcolors.ENDC
        if black_board_object.Left1 == False:
            line = extract_lines(r2, status = 'L')
            line_storage(line)
        
        black_board_object.Left2 = True
    else:
        print bcolors.OKBLUE + "Nothing on the left2" + bcolors.ENDC
        black_board_object.Left2 = False
        
    av_distance3 = sum(r1)/70.
    if av_distance3 < black_board_object.maximum_range:
        print bcolors.OKBLUE + "Left1 sensing" + bcolors.ENDC
        if black_board_object.Left2 == False and black_board_object.Left3 == False:
            line = extract_lines(r1, status = 'L')
            line_storage(line)
        
        black_board_object.Left1 = True
    else:
        print bcolors.OKBLUE + "Nothing on the left1" + bcolors.ENDC
        black_board_object.Left1 = False
        
    if line is not None:
        line_display()
        
    return 1

def front_status():
    #pdb.set_trace()
    print bcolors.OKBLUE + "Estoy en front status" + bcolors.ENDC
    laser_scan()
    rospy.sleep(2)
    if black_board_object.move_count == 0:
        return 1
    
    r = list()
    r = black_board_object.kinect_scan

    black_board_object.lines.header.stamp = rospy.Time.now()
    #plotter(r[200:438], "Front")
    
    filtered_scan, singularities = moving_window_filtro(r[200:438], tolerance=0.1, n_neighbors=1)
    if len(singularities) != 0: 
        print bcolors.OKBLUE + "Front singularities: " + repr(singularities) + bcolors.ENDC
#        if len(singularities) != 0:
#            plotter(filtered_scan, "Right-filtered")
    r1 = r[200:270]
    r2 = r[270:370]
    r3 = r[370:438]
    if len(singularities) != 0: 
        print bcolors.OKBLUE + "Front singularities: " + repr(singularities) + bcolors.ENDC
        r1 = r[200:singularities[0]]
        if len(singularities) > 1:
            r2 = r[singularities[0]:singularities[1]]
        if len(singularities) > 2:
            r3 = r[singularities[1]:singularities[2]]
    line = ExtractedLine()
    if min(r2) < black_board_object.maximum_range:
        print bcolors.OKBLUE + "Front2 sensing" + bcolors.ENDC
        
        line = extract_lines(r2, status = 'F')
        line_storage(line)

        print "odom_angle: ", black_board_object.agent_rotation_angle
        print "wall_angle: ", black_board_object.front_wall_angle
        print bcolors.OKBLUE + "distance to front wall(rho): " +  str(black_board_object.distance_to_front_wall) + bcolors.ENDC
        print bcolors.OKBLUE + "distance to front obstacle: " +  str(black_board_object.distance_front) + bcolors.ENDC
        black_board_object.Front2 = True
        black_board_object.adv_distance = black_board_object.distance_front - 1.7
    else:
        print bcolors.OKBLUE + "Nothing on the front2" + bcolors.ENDC
        black_board_object.adv_distance = 0.0
        black_board_object.Front2 = False
        #Right_Corner = False

    #raw_input("Press ENTER to continue...")
              
    #pdb.set_trace()
    av_distance1 = sum(r1)/69.
    if av_distance1 < black_board_object.maximum_range:
        print bcolors.OKBLUE + "Front1 sensing" + bcolors.ENDC
        if black_board_object.Front2 == False:
            line = extract_lines(r1, status = 'F')
            line_storage(line)
        
        black_board_object.Front1 = True
    else:
        print bcolors.OKBLUE + "Nothing on the front1" + bcolors.ENDC
        black_board_object.Front1 = False
    #raw_input("Press ENTER to continue...")  
    
    av_distance3 = sum(r3)/67.
    if av_distance3 < black_board_object.maximum_range:
        print bcolors.OKBLUE + "Front3 sensing" + bcolors.ENDC
        if black_board_object.Front1 == False and black_board_object.Front2 == False:
            line = extract_lines(r3, status = 'F')
            line_storage(line)
        
        black_board_object.Front3 = True
    else:
        print bcolors.OKBLUE + "Nothing on the front3" + bcolors.ENDC
        black_board_object.Front3 = False
    
    if line is not None:
        line_display()
        
    #raw_input("Press ENTER to continue...")
    
    return 1

def sign(x):
    if x < 0.0:
        return -1
    elif x == 0.0:
        return 0
    elif x > 0.0:
        return 1
