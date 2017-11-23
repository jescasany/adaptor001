#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed May 10 20:18:58 2017

@author: juan
"""
import pdb

from black_board_class import bbo
import rospy

import math
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import Twist, Point32

from advance import advance
import sensor_msgs.msg
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle

import display_lines
from adaptor001.msg import ExtractedLine
from adaptor001.msg import ExtractedLines

from fancy_prompts import bcolors

from decode import Decode

from angles import constrain_angle

def print_position():
    print bbo.move_count
    print bbo.agent_position
    print bbo.agent_rotation

def print_interaction(interaction, text):
    bbo.interaction = interaction
    bbo.interaction1 = bbo.interaction
    decoded = Decode(str(interaction))
    translated = decoded.get_translation()
    print "\n"
    print bcolors.OKGREEN + text + translated + bcolors.ENDC
   
def plotter(y, name):    
    """
    Uncomment these next few lines to display a constantly updating graph of data.
    Note: when the graph appears, you must close it first, and then the graph will 
    reopen and display the data as it comes in.
    """
    x = []
    for i in range(106):
        x.append(i*bbo.angle_increment)
    for i in range(106-len(y)):
        y.append(0.0)
#    print x
#    print y
#    print math.degrees(bbo.angle_increment)
    ax = plt.subplot(111, projection='polar')
    ax.plot(x, y, 'ro', x, y, 'k')
    ax.set_ylim(min(y)-1, max(y)+1)
    ax.set_rmax(max(y)+1)
    ax.set_rticks(np.arange(min(y)-1,max(y)+1,1.0))  # less radial ticks
    ax.set_rlabel_position(-bbo.angle_increment)  # get radial labels away from plotted line
    ax.grid(True)
    ax.set_title(name, va='bottom')
    plt.show()

def display_callback(lines_msg):
    """
    Callback for extracted lines, displayed by publishing to
    vis_lines_topic and vis_scanpoints_topic.
    """
    print bcolors.OKYELLOW + "Estoy en DISPLAY" + bcolors.ENDC
    display_lines.create_lines_marker(lines_msg)
    rospy.sleep(2)
    display_lines.create_scanpoints_marker(lines_msg)
    rospy.sleep(2)
         
def fit_line(scan, start_index, end_index, maximum_range, first_index):
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
    sum_weights = 0
    sum_num_L = 0 # The summation term in the numerator on the left
    sum_num_R = 0 # The summation term in the numerator on the right
    sum_den_L = 0 # The summation term in the denominator on the left
    sum_den_R = 0 # The summation term in the denominator on the right
    
    angle_min = bbo.angle_min + first_index * bbo.angle_increment
        
    for i in range(start_index, end_index+1):
        rho = scan[i]
        if rho == 0.0 or rho > maximum_range:
            continue
        theta = angle_min + i * bbo.angle_increment
        weight = 1/rho**2
        #weight = 1

        sum_weights += weight

        factor1 = weight * rho * rho
        sum_num_L += factor1 * math.sin(2 * theta)
        sum_den_L += factor1 * math.cos(2 * theta)
        
        for j in range(start_index, end_index+1):
            rho_j = scan[j]
            if rho_j == 0 or rho_j > maximum_range:
                continue
            theta_j = angle_min + j * bbo.angle_increment
            weight_j = 1 / rho_j**2
            #weight_j = 1

            factor2 = weight * weight_j * rho * rho_j
            sum_num_R += factor2 * math.cos(theta) * math.sin(theta_j)
            sum_den_R += factor2 * math.cos(theta + theta_j)

    if sum_weights == 0:
        # There are either no scan points at all, or none within range.
        return None

    sum_num_R *= 2.0 / sum_weights
    sum_den_R /= sum_weights
    alpha = math.atan2(sum_num_L - sum_num_R, sum_den_L - sum_den_R) / 2.0 + math.pi/2
    
    # We now calculate r.
    sum_num = 0 # The summation term in the numerator
    for i in range(start_index, end_index+1):
        rho = scan[i]
        if rho == 0 or rho > maximum_range:
            continue
        theta = angle_min + i * bbo.angle_increment
        weight = 1 / rho**2
        #weight = 1

        sum_num += weight * rho * math.cos(theta - alpha)

    r = sum_num / sum_weights
    
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
    first_scan_point = Point32()
    last_scan_point = Point32()
    dist = scan[start_index]
    angle = angle_min + start_index * bbo.angle_increment
    if dist <= maximum_range:
        first_scan_point.x = dist * math.cos(angle)
        first_scan_point.y = dist * math.sin(angle)
    dist = scan[end_index]
    angle = angle_min + end_index * bbo.angle_increment
    if dist <= maximum_range:
        last_scan_point.x = dist * math.cos(angle)
        last_scan_point.y = dist * math.sin(angle)

    return ExtractedLine(-r, alpha, first_scan_point, last_scan_point)

def right_wall_param(r, start_index, end_index, maximum_range, first_index):
    #pdb.set_trace()
    line = ExtractedLine()
    line = fit_line(r, start_index, end_index, maximum_range, first_index)
    if line is not None:
        # approximate wall_angle (degrees)
        bbo.right_wall_angle = normalize_angle(line.alpha)*180./math.pi
        # approximate dist to wall (meters)
        dist = bbo.kinect_scan[0]
        bbo.distance_to_right_wall = min(dist, abs(line.r))

    return line
    
def left_wall_param(r, start_index, end_index, maximum_range, first_index):
    line = ExtractedLine()
    line = fit_line(r, start_index, end_index, maximum_range, first_index)
    if line is not None:
        # approximate wall_angle (degrees)
        bbo.left_wall_angle = normalize_angle(line.alpha)*180./math.pi
        # approximate dist to wall (meters)
        dist = bbo.kinect_scan[106]
        bbo.distance_to_left_wall = min(dist, abs(line.r))

    return line

def front_wall_param(r, start_index, end_index, maximum_range, first_index):
    #pdb.set_trace()
    line = ExtractedLine()
    line = fit_line(r, start_index, end_index, maximum_range, first_index)
    if line is not None:
        # approximate wall_angle (degrees)
        bbo.front_wall_angle = normalize_angle(line.alpha)*180./math.pi
        # approximate dist to wall (meters)
        dist = bbo.kinect_scan[53]
        bbo.distance_to_front_wall = min(dist, abs(line.r))

    return line

def extract_lines(r, status, first_index):
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
        if r[i] < bbo.maximum_range and r[i] != 0.0:
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
        line = right_wall_param(r, start_index, end_index, bbo.maximum_range, first_index)
        if line is None:
            return line
#        if line.r > bbo.distance_to_right_wall:
#            line = None
    elif status == 'L':
        line = left_wall_param(r, start_index, end_index, bbo.maximum_range, first_index)
        if line is None:
            return line
        if line.r > bbo.distance_to_left_wall:
            line = None
    elif status == 'F':
        line = front_wall_param(r, start_index, end_index, bbo.maximum_range, first_index)
        if line is None:
            return line
        if line.r > bbo.distance_to_front_wall:
            line = None
    if line is not None:
        if line.r > bbo.maximum_range or (abs(line.alpha) > math.radians(10) and abs(line.alpha) < math.radians(80)):
            line = None
    return line

def clear_lines():
    lines = bbo.lines
    for l in lines.lines:
        if l.r > bbo.distance_to_right_wall or l.r > bbo.distance_front:
            bbo.lines.lines.remove(l)
            
def line_storage(line):
    clear_lines()
    if line is not None and line not in bbo.lines.lines:
        bbo.lines.lines.append(line)
#        if len(bbo.lines.lines) > 1:
#            bbo.lines.lines.pop(0)
#    print len(bbo.lines.lines)
#    print bcolors.OKGREEN + "LINES: " + bcolors.ENDC
#    print bbo.lines
    
    #raw_input('Enter to continue')
    
def line_display():
    # Subscribe to 'lines_topic'
    bbo.extracted_publisher.publish(bbo.lines)
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
    bbo.kinect_scan = []
    bbo.distance_front = min(bbo.raw_kinect_scan[300:338])  # NOTE that is just raw_kinect_scan not transformed kinect_scan
    # transform the laser readings to one in six (0-639 to 0-106)
    for i in range(0, len(bbo.raw_kinect_scan), 6):
        if bbo.raw_kinect_scan[i] == 5.0:
            bbo.raw_kinect_scan[i] = 7.0
        bbo.kinect_scan.append(bbo.raw_kinect_scan[i])
#    print bbo.kinect_scan, len(bbo.kinect_scan)
#    raw_input('Enter to continue')
    
    if bbo.distance_front <= 1.7:
        bbo.driving_forward = False
    else:
        if True in bbo.Right:
            bbo.adv_distance = bbo.distance_front - 1.7
        else:
            bbo.adv_distance = 0.0
            
        bbo.driving_forward = True
    #rospy.loginfo("laser_scan done")
    return 1
    
def scan_callback(msg):
    bbo.raw_kinect_scan = list(msg.ranges) # transformed to list since msg.ranges is a tuple
    bbo.angle_min = msg.angle_min
    bbo.angle_increment = 6 * msg.angle_increment   #  6 is the number of readings we are jumping
    
def save_images():
    bbo.images.write(str(bbo.raw_kinect_scan) + " ")
    return 1
    
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
        # append a singularity at least separated 4 readings from previous
        # it's 4 for one reading in six 
        if fi != 0.0 and (i - last_sing) > 4:
            singularity.append(i)
            last_sing = i
            
    return filtro, singularity

def round_to_zero(val, tolerance):
    if -1 * tolerance < val < tolerance:
        return 0
    else:
        return val
    
def clamp(val, minimum, maximum):
        if val < minimum:
            return minimum
        elif val > maximum:
            return maximum
        else:
            return val
    
def arrange():
    print "Arrange status"
    bbo.arrange_status = True
    
    #pdb.set_trace()
    
    # makes the robot turn to adopt a rotation angle of 0, +-(pi/2) or pi 
    agent_rotation_angle = math.degrees(normalize_angle(quat_to_angle(bbo.agent_rotation)))
    
    angle_rad_rot = math.radians(agent_rotation_angle)
    if abs(agent_rotation_angle) < 45:
        turn = -angle_rad_rot
        print bcolors.OKBLUE + "agent_rotation_angle: ", str(agent_rotation_angle),  "\n", "degrees(turn): ", str(math.degrees(turn)) + bcolors.ENDC
        (bbo.agent_position, bbo.agent_rotation) = advance(0.0, turn, da=True)
    if abs(agent_rotation_angle) < 90 and abs(agent_rotation_angle) >= 45:
        turn = sign(angle_rad_rot)*abs(math.pi/2 - abs(angle_rad_rot))
        print bcolors.OKBLUE + "agent_rotation_angle: ", str(agent_rotation_angle),  "\n", "degrees(turn): ", str(math.degrees(turn)) + bcolors.ENDC
        (bbo.agent_position, bbo.agent_rotation) = advance(0.0, turn, da=True)
    if abs(agent_rotation_angle) < 135 and abs(agent_rotation_angle) >= 90:
        turn = -sign(angle_rad_rot)*abs(math.pi/2 - abs(angle_rad_rot))
        print bcolors.OKBLUE + "agent_rotation_angle: ", str(agent_rotation_angle),  "\n", "degrees(turn): ", str(math.degrees(turn)) + bcolors.ENDC
        (bbo.agent_position, bbo.agent_rotation) = advance(0.0, turn, da=True)
    if abs(agent_rotation_angle) >= 135:
        turn = sign(angle_rad_rot)*abs(math.pi - abs(angle_rad_rot))
        print bcolors.OKBLUE + "agent_rotation_angle: ", str(agent_rotation_angle),  "\n", "degrees(turn): ", str(math.degrees(turn)) + bcolors.ENDC
        (bbo.agent_position, bbo.agent_rotation) = advance(0.0, turn, da=True)
        
    bbo.agent_rotation_angle = math.degrees(normalize_angle(quat_to_angle(bbo.agent_rotation)))
    bbo.arrange_status = False
    return 1

def right_status():
    if bbo.arrange_status:
        print bcolors.OKBLUE + "Estoy en right status desde arrange" + bcolors.ENDC
    elif bbo.environment_status:
        print bcolors.OKBLUE + "Estoy en right status desde environment" + bcolors.ENDC
    laser_scan()
    rospy.sleep(2)
    
    #pdb.set_trace()
    
    if bbo.move_count == 0:
        print "Right status"
        return 1
    r = list()
    r = bbo.kinect_scan
    bbo.lines.header.stamp = rospy.Time.now()
    #plotter(r[0:36], "Right")
    
    filtered_scan, singularities = moving_window_filtro(r[bbo.laser_right_start:bbo.laser_right_end], bbo.tolerance, n_neighbors=1)
    
    tracks = list()
    r1 = r[bbo.laser_right_start:bbo.laser_right_end]
    tracks.append(r1)
    index = []
    index.append(bbo.laser_right_start)
    bbo.right_singularities = []
    bbo.right_distances = []
    
    if len(singularities) != 0:
        for i in range(len(singularities)):
            singularities[i] = singularities[i] + 1 # add 1 to make the track uniform
        singularities.insert(0, bbo.laser_right_start)
        if bbo.laser_right_end not in singularities:
            singularities.append(bbo.laser_right_end)
        print bcolors.OKBLUE + "Right singularities: " + repr(singularities) + bcolors.ENDC
        bbo.right_singularities = singularities
        tracks = []
        for i in range(len(singularities)-1):
            tracks.append(r[singularities[i]:singularities[i+1]])
            bbo.right_distances.append(r[singularities[i]])
        #print "Right tracks: ", tracks
    #pdb.set_trace()    
        #plotter(filtered_scan, "Right-filtered")
    line = ExtractedLine()
    track_count = 1
    index_count = 0
    bbo.Right = []
    
    for t in tracks:
        if min(t) < bbo.maximum_range:
            bbo.Right.append(True)
            print bcolors.OKBLUE + "Right" + str(track_count) + " sensing" + bcolors.ENDC
            if len(singularities) != 0:
                first_index = singularities[index_count]
                line = extract_lines(t, 'R', first_index)
                line_storage(line)
                index_count += 1
            else:
                first_index = index[0]
                line = extract_lines(t, 'R', first_index)
                line_storage(line)
            
            track_count += 1
            print bcolors.OKBLUE + "odom_angle: " +  str(bbo.agent_rotation_angle) + bcolors.ENDC
            print bcolors.OKBLUE + "wall_angle: " +  str(bbo.right_wall_angle) + bcolors.ENDC
            print bcolors.OKBLUE + "distance to right wall(rho): " +  str(bbo.distance_to_right_wall) + bcolors.ENDC
            print bcolors.OKBLUE + "distance to front obstacle: " +  str(bbo.distance_front) + bcolors.ENDC
            
            bbo.adv_distance = bbo.distance_front - 1.7
            if line is not None:
                line_display()
        else:
            print bcolors.OKBLUE + "Nothing on the right" + str(track_count) + bcolors.ENDC
            bbo.adv_distance = 0.0
            bbo.Right.append(False)
            track_count += 1
    
    n = len(bbo.Right)
    if n == 1:
        bbo.Right1 = bbo.Right[0]
    elif n == 2:
        bbo.Right1 = bbo.Right[0]
        bbo.Right2 = bbo.Right[1]
    elif n == 3:
        bbo.Right1 = bbo.Right[0]
        bbo.Right2 = bbo.Right[1]
        bbo.Right3 = bbo.Right[2]  
    
    if line is not None:
        line_display()
    #print tracks    
    #raw_input("Right Status * Press ENTER to continue...")    
    return 1

def left_status():
    if bbo.arrange_status:
        print bcolors.OKBLUE + "Estoy en left status desde arrange" + bcolors.ENDC
    elif bbo.environment_status:
        print bcolors.OKBLUE + "Estoy en left status desde environment" + bcolors.ENDC
        
    laser_scan()
    
    #pdb.set_trace()
    
    rospy.sleep(2)
    if bbo.move_count == 0:
        print "Left status"
        return 1
    
    r = list()
    r = bbo.kinect_scan
    #plotter(kinect_scan)
    filtered_scan, singularities = moving_window_filtro(r[bbo.laser_left_start:bbo.laser_left_end], bbo.tolerance, n_neighbors=1)
        
#        if len(singularities) != 0:
#            plotter(filtered_scan, "Right-filtered")
    tracks = list()
    r1 = r[bbo.laser_left_start:bbo.laser_left_end]
    tracks.append(r1)
    index = []
    index.append(0)
    bbo.left_singularities = []
    bbo.left_distances = []
    
    if len(singularities) != 0:
        for i in range(len(singularities)):
            singularities[i] = singularities[i] + bbo.laser_left_start + 1   # add 1 to make the track uniform
        singularities.insert(0, bbo.laser_left_start)
        if bbo.laser_left_end not in singularities:
            singularities.append(bbo.laser_left_end)
        print bcolors.OKBLUE + "Left singularities: " + repr(singularities) + bcolors.ENDC
        bbo.left_singularities = singularities
        tracks = []
        for i in range(len(singularities)-1):
            tracks.append(r[singularities[i]:singularities[i+1]])
            bbo.left_distances.append(r[singularities[i]])
        #print "Left tracks: ", tracks
        
    #pdb.set_trace()
    
    line = ExtractedLine()
    track_count = 1
    index_count = 0
    bbo.Left = []
    
    for t in tracks:
        if min(t) < bbo.maximum_range:
            print bcolors.OKBLUE + "Left" + str(track_count) + " sensing" + bcolors.ENDC
            if len(singularities) != 0:
                first_index = singularities[index_count]
                line = extract_lines(t, 'L', first_index)
                line_storage(line)
                index_count += 1
            else:
                first_index = index[0]
                line = extract_lines(t, 'L', first_index)
                line_storage(line)
            
            track_count += 1
            print bcolors.OKBLUE + "odom_angle: " +  str(bbo.agent_rotation_angle) + bcolors.ENDC
            print bcolors.OKBLUE + "wall_angle: " +  str(bbo.left_wall_angle) + bcolors.ENDC
            print bcolors.OKBLUE + "distance to left wall(rho): " +  str(bbo.distance_to_left_wall) + bcolors.ENDC
            print bcolors.OKBLUE + "distance to front obstacle: " +  str(bbo.distance_front) + bcolors.ENDC
            bbo.Left.append(True)
            bbo.adv_distance = bbo.distance_front - 1.7
                
        else:
            print bcolors.OKBLUE + "Nothing on the left" + str(track_count) + bcolors.ENDC
            bbo.Left.append(False)
            bbo.adv_distance = 0.0
            track_count += 1
    
    n = len(bbo.Left)
    if n == 1:
        bbo.Left1 = bbo.Left[0]
    elif n == 2:
        bbo.Left1 = bbo.Left[0]
        bbo.Left2 = bbo.Left[1]
    elif n == 3:
        bbo.Left1 = bbo.Left[0]
        bbo.Left2 = bbo.Left[1]
        bbo.Left3 = bbo.Left[2]
        
    if line is not None:
        line_display()
    #print tracks
    #raw_input("Left Status * Press ENTER to continue...")     
    return 1

def front_status():
    #pdb.set_trace()
    if bbo.arrange_status:
        print bcolors.OKBLUE + "Estoy en front status desde arrange" + bcolors.ENDC
    elif bbo.environment_status:
        print bcolors.OKBLUE + "Estoy en front status desde environment" + bcolors.ENDC
    
    laser_scan()
    
    #pdb.set_trace()
    
    rospy.sleep(2)
    if bbo.move_count == 0:
        print "Front status"
        return 1
    
    r = list()
    r = bbo.kinect_scan

    bbo.lines.header.stamp = rospy.Time.now()
    #plotter(r[200:438], "Front")
    
    filtered_scan, singularities = moving_window_filtro(r[bbo.laser_front_start:bbo.laser_front_end], bbo.tolerance, n_neighbors=1)
    
    tracks = list()
    r1 = r[bbo.laser_front_start:bbo.laser_front_end]
    tracks.append(r1)
    index = []
    index.append(0)
    bbo.front_singularities = []
    bbo.front_distances = []
    
    if len(singularities) != 0:
        for i in range(len(singularities)):
            singularities[i] = singularities[i] + bbo.laser_front_start + 1 # add 1 to make the track uniform
        singularities.insert(0, bbo.laser_front_start)
        if bbo.laser_front_end not in singularities: 
            singularities.append(bbo.laser_front_end)
        print bcolors.OKBLUE + "Front singularities: " + repr(singularities) + bcolors.ENDC
        bbo.front_singularities = singularities
        tracks = []
        for i in range(len(singularities)-1):
            tracks.append(r[singularities[i]:singularities[i+1]])
            bbo.front_distances.append(r[singularities[i]])
        #print "Front tracks: ", tracks
        
    #pdb.set_trace()
    
    line = ExtractedLine()
    track_count = 1
    index_count = 0
    bbo.Front = []
    
    for t in tracks:
        if min(t) < bbo.maximum_range:
            print bcolors.OKBLUE + "Front" + str(track_count) + " sensing" + bcolors.ENDC
            if len(singularities) != 0:
                first_index = singularities[index_count]
                line = extract_lines(t, 'F', first_index)
                line_storage(line)
                index_count += 1
            else:
                first_index = index[0]
                line = extract_lines(t, 'F', first_index)
                line_storage(line)
    
            track_count += 1
            print bcolors.OKBLUE + "odom_angle: " +  str(bbo.agent_rotation_angle) + bcolors.ENDC
            print bcolors.OKBLUE + "wall_angle: " +  str(bbo.front_wall_angle) + bcolors.ENDC
            print bcolors.OKBLUE + "distance to front wall(rho): " +  str(bbo.distance_to_right_wall) + bcolors.ENDC
            print bcolors.OKBLUE + "distance to front obstacle: " +  str(bbo.distance_front) + bcolors.ENDC
            bbo.Front.append(True)
            bbo.adv_distance = bbo.distance_front - 1.7
        else:
            print bcolors.OKBLUE + "Nothing on the front" + str(track_count) + bcolors.ENDC
            bbo.adv_distance = bbo.distance_front - 1.7
            bbo.Front.append(False)
            track_count += 1
            #Right_Corner = False
    
        #raw_input("Press ENTER to continue...")
    n = len(bbo.Front)
    if n == 1:
        bbo.Front1 = bbo.Front[0]
    elif n == 2:
        bbo.Front1 = bbo.Front[0]
        bbo.Front2 = bbo.Front[1]
    elif n == 3:
        bbo.Front1 = bbo.Front[0]
        bbo.Front2 = bbo.Front[1]
        bbo.Front3 = bbo.Front[2]
        
    if line is not None:
        line_display()
    #print tracks    
    #raw_input("Front Status * Press ENTER to continue...") 
    
    return 1

def sign(x):
    if x < 0.0:
        return -1
    elif x == 0.0:
        return 0
    elif x > 0.0:
        return 1
