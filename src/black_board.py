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
    print bbo.move_count
    print bbo.agent_position
    print bbo.agent_rotation
    
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
    print bcolors.OKGREEN + "Wall Following" + bcolors.ENDC
    if len(bbo.right_singularities) == 0:
        get_close_line()
    elif len(bbo.right_singularities) == 3:
        corner_index = bbo.right_singularities[1]
        corner_distance = bbo.right_distances[1]
        angle = bbo.angle_min + corner_index * bbo.angle_increment
        dist = corner_distance * math.cos(angle)
        
    bbo.adv_distance = dist + 2.0
    bbo.adv_angle = -math.pi/2
    
    print bcolors.OKBLUE + "bbo.adv_distance: " +  str(bbo.adv_distance) + " m" + bcolors.ENDC
    print bcolors.OKBLUE + "bbo.adv_angle: " +  str(math.degrees(bbo.adv_angle)) + " deg" + bcolors.ENDC 

    # Publish the twist message produced by the controller.
    print bcolors.OKBLUE + "STOP the agent before wall following" + bcolors.ENDC
    bbo.cmd_vel_publisher.publish(Twist())
    rospy.sleep(2)
    #pdb.set_trace()
    #raw_input("Press ENTER to continue...")
    laser_scan()
    if bbo.driving_forward:
        print "Moving due to singularity"
        (bbo.agent_position, bbo.agent_rotation) = advance(bbo.adv_distance, bbo.adv_angle, da=False)
    else:
        print "Just turning due to singularity"
        (bbo.agent_position, bbo.agent_rotation) = advance(0.0, bbo.adv_angle, da=False)
       
    rospy.sleep(2)
    bbo.agent_rotation_angle = arrange()

def get_close_line():
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
    lines = bbo.lines
    smallest_r = float('inf')
    for l in lines.lines:
        if l.r < smallest_r:
            smallest_r = l.r
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
        
        bbo.selected_lines_publisher.publish(sel_lines)

        fo = bbo.follow_offset
        #fa = bbo.follow_advance
        fa = bbo.distance_front
        #dist = min(bbo.distance_to_right_wall, line.r)
        dist = bbo.distance_to_right_wall
        bbo.adv_distance = math.sqrt(math.pow(dist - fo, 2) + math.pow(fa, 2))
        th = math.pi/2 - math.atan(fa/abs(dist - fo))
        bbo.adv_angle = th
        if (dist - fo) > 0:
            bbo.adv_angle = -th
        print bcolors.OKBLUE + "bbo.adv_distance: " +  str(bbo.adv_distance) + " m" + bcolors.ENDC
        print bcolors.OKBLUE + "bbo.adv_angle: " +  str(math.degrees(bbo.adv_angle)) + " deg" + bcolors.ENDC 
    
        # Publish the twist message produced by the controller.
        print bcolors.OKBLUE + "STOP the agent before wall following" + bcolors.ENDC
        bbo.cmd_vel_publisher.publish(Twist())
        rospy.sleep(2)
        #pdb.set_trace()
        #raw_input("Press ENTER to continue...")
        laser_scan()
        if bbo.driving_forward:
            print "Moving to closest line"
            (bbo.agent_position, bbo.agent_rotation) = advance(bbo.adv_distance, bbo.adv_angle, da=False)
        else:
            print "Just turning to closest line"
            (bbo.agent_position, bbo.agent_rotation) = advance(0.0, bbo.adv_angle, da=False) 
            
        rospy.sleep(2)
        bbo.agent_rotation_angle = arrange()
        
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
        dist = min(r)
        bbo.distance_to_right_wall = min(dist, abs(line.r))

    return line
    
def left_wall_param(r, start_index, end_index, maximum_range, first_index):
    line = ExtractedLine()
    line = fit_line(r, start_index, end_index, maximum_range, first_index)
    if line is not None:
        # approximate wall_angle (degrees)
        bbo.left_wall_angle = normalize_angle(line.alpha)*180./math.pi
        # approximate dist to wall (meters)
        dist = min(r)
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
        dist = min(r)
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
    if line is not None and line not in bbo.lines.lines:
        bbo.lines.lines.append(line)
        if len(bbo.lines.lines) > 2:
            bbo.lines.lines.pop(0)
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
    bbo.distance_front = min(bbo.kinect_scan[300:338])
    if bbo.distance_front <= 1.7:
        bbo.driving_forward = False
    else:
        if bbo.Right1:
            bbo.adv_distance = bbo.distance_front - 1.7
        elif bbo.Right2 or bbo.Right3:
            bbo.adv_distance = 1.0
        else:
            bbo.adv_distance = 0.0
            
        bbo.driving_forward = True
    #rospy.loginfo("laser_scan done")
    
def scan_callback(msg):
    bbo.kinect_scan = list(msg.ranges) # transformed to list since msg.ranges is a tuple
    bbo.angle_min = msg.angle_min
    bbo.angle_increment = msg.angle_increment
    
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
    bbo.arrange_status = True
    laser_scan()
    bbo.Right1 = False
    bbo.Right2 = False
    bbo.Right3 = False
    right_status()
    
    bbo.Front1 = False
    bbo.Front2 = False
    bbo.Front3 = False
    front_status()
    
    pdb.set_trace()
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
    if bbo.Right1 == False and bbo.Right2 == False and bbo.Right3 == False and bbo.move_count > 0:
        print "Turning to the right since nothing on the RIGHT"
        (bbo.agent_position, bbo.agent_rotation) = advance(0.0, -math.pi/2, da=True)
        
    if bbo.Right2 == True and bbo.driving_forward == False:
        print "Turning to the left since RIGHT and FRONT are busy"
        (bbo.agent_position, bbo.agent_rotation) = advance(0.0, math.pi/2, da=True)
    agent_rotation_angle = math.degrees(normalize_angle(quat_to_angle(bbo.agent_rotation)))
    bbo.arrange_status = False
    return agent_rotation_angle
    
def move_adv():
    print bcolors.OKBLUE + "Estoy en move advance" + bcolors.ENDC
    #pdb.set_trace()
    try:
        #raw_input("Press a key to continue...")
        if bbo.move_count == 0:
            print bcolors.OKGREEN + "PREPARING THINGS IN MY PLACE" + bcolors.ENDC
            # agent_pose is a tuple (position, rotation)
            (bbo.agent_position, bbo.agent_rotation) = advance(0.0, 0.0, da=True)
            print_position()
            bbo.waypoints.append((bbo.agent_position, bbo.agent_rotation))
            bbo.move_count += 1
            print bcolors.OKBLUE + "Preparation DONE" + bcolors.ENDC
            bbo.agent_rotation_angle = arrange()
            return 1
        
        laser_scan()
        rospy.sleep(2)
        if bbo.driving_forward or bbo.adv_distance == 0.0:
            get_close()
        laser_scan()
        rospy.sleep(2)
        if bbo.driving_forward or bbo.adv_distance == 0.0:    
            (bbo.agent_position, bbo.agent_rotation) = advance(bbo.adv_distance, 0.0, da=True)
            
            bbo.adv_angle = 0.0
            print_position()
#            raw_input("Press a key to continue...")
            bbo.waypoints.append((bbo.agent_position, bbo.agent_rotation))
            bbo.move_count += 1
            print bcolors.OKBLUE + "move_adv DONE" + bcolors.ENDC
            bbo.move_fail = False
        else:
            print bcolors.OKBLUE + "move_adv FAILED" + bcolors.ENDC
            (bbo.agent_position, bbo.agent_rotation) = advance(0.0, 0.0, da=True)
            
            bbo.move_fail = True
            
    except:
        print bcolors.OKBLUE + "move_adv FAILED" + bcolors.ENDC
        (bbo.agent_position, bbo.agent_rotation) = advance(0.0, 0.0, da=True)
        bbo.move_fail = True
        bbo.agent_rotation_angle = arrange()
        return 1
    bbo.agent_rotation_angle = arrange()
    return 1


def right_status():
    if bbo.arrange_status:
        print bcolors.OKBLUE + "Estoy en right status desde arrange" + bcolors.ENDC
    elif bbo.environment_status:
        print bcolors.OKBLUE + "Estoy en right status desde environment" + bcolors.ENDC
    laser_scan()
    
    #pdb.set_trace()
    
    rospy.sleep(2)
    if bbo.move_count == 0:
        print "Move count = 0"
        return 1
    r = list()
    r = bbo.kinect_scan
    bbo.lines.header.stamp = rospy.Time.now()
    #plotter(r[0:200], "Right")
    
    filtered_scan, singularities = moving_window_filtro(r[0:200], tolerance=0.1, n_neighbors=1)
    
    tracks = list()
    r1 = r[0:200]
    tracks.append(r1)
    index = list()
    index.append(0)
    
    if len(singularities) != 0:
        for i in range(len(singularities)):
            singularities[i] = singularities[i] + 1 # add 1 to make the track uniform
        singularities.insert(0, 0)
        singularities.append(200)
        print bcolors.OKBLUE + "Right singularities: " + repr(singularities) + bcolors.ENDC
        bbo.right_singularities = singularities
        tracks = []
        for i in range(len(singularities)-1):
            tracks.append(r[singularities[i]:singularities[i+1]])
            bbo.right_distances.append(r[singularities[i]])
        print "Right tracks: ", tracks
    pdb.set_trace()    
#        if len(singularities) != 0:
#            plotter(filtered_scan, "Right-filtered")
    line = ExtractedLine()
    track_count = 1
    index_count = 0
    for t in tracks:
        if min(t) < bbo.maximum_range:
            print bcolors.OKBLUE + "Right" + str(track_count) + " sensing" + bcolors.ENDC
            if len(singularities) != 0:
                first_index = singularities[index_count]
                line = extract_lines(t, 'R', first_index)
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
            bbo.Right.append(True)
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
        
    #raw_input("Press ENTER to continue...")
    if line is not None:
        line_display()
        
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
        print "Move count = 0"
        return 1
    
    r = list()
    r = bbo.kinect_scan
    #plotter(kinect_scan)
    filtered_scan, singularities = moving_window_filtro(r[438:639], tolerance=0.1, n_neighbors=1)
        
#        if len(singularities) != 0:
#            plotter(filtered_scan, "Right-filtered")
    tracks = list()
    r1 = r[438:639]
    tracks.append(r1)
    index = list()
    index.append(0)
    
    if len(singularities) != 0:
        for i in range(len(singularities)):
            singularities[i] = singularities[i] + 438 + 1 # add 1 to make the track uniform
        singularities.insert(0, 438)
        singularities.append(639)
        print bcolors.OKBLUE + "Left singularities: " + repr(singularities) + bcolors.ENDC
        bbo.left_singularities = singularities
        tracks = []
        for i in range(len(singularities)-1):
            tracks.append(r[singularities[i]:singularities[i+1]])
        print "Left tracks: ", tracks
        
    line = ExtractedLine()
    track_count = 1
    index_count = 0
    for t in tracks:
        if min(t) < bbo.maximum_range:
            print bcolors.OKGREEN + "Left" + str(track_count) + " sensing" + bcolors.ENDC
            if len(singularities) != 0:
                first_index = singularities[index_count]
                line = extract_lines(t, 'L', first_index)
                index_count += 1
            else:
                first_index = index[0]
                line = extract_lines(t, 'L', first_index)
            line_storage(line)
            
            track_count += 1
            print "odom_angle: ",  bbo.agent_rotation_angle
            print "wall_angle: ", bbo.left_wall_angle
            print "distance to left wall(coefficients[1]): ", bbo.distance_to_left_wall
            print bcolors.OKBLUE + "distance to front obstacle: " +  str(bbo.distance_front) + bcolors.ENDC
            bbo.Left.append(True)
            
            if bbo.Left.pop() == True:
                bbo.adv_distance = bbo.distance_front - 1.7
            else:
                bbo.adv_distance = 0.0
        else:
            print bcolors.OKBLUE + "Nothing on the left" + str(track_count) + bcolors.ENDC
            bbo.Left.append(False)
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
    
    #        raw_input("Press a key to continue...")
        
    if line is not None:
        line_display()
        
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
        print "Move count = 0"
        return 1
    
    r = list()
    r = bbo.kinect_scan

    bbo.lines.header.stamp = rospy.Time.now()
    #plotter(r[200:438], "Front")
    
    filtered_scan, singularities = moving_window_filtro(r[200:438], tolerance=0.1, n_neighbors=1)
    
    tracks = list()
    r1 = r[200:438]
    tracks.append(r1)
    index = list()
    index.append(0)
    
    if len(singularities) != 0:
        for i in range(len(singularities)):
            singularities[i] = singularities[i] + 200 + 1 # add 1 to make the track uniform
        singularities.insert(0, 200)
        singularities.append(438)
        print bcolors.OKBLUE + "Front singularities: " + repr(singularities) + bcolors.ENDC
        bbo.front_singularities = singularities
        tracks = []
        for i in range(len(singularities)-1):
            tracks.append(r[singularities[i]:singularities[i+1]])
        print "Front tracks: ", tracks

    line = ExtractedLine()
    track_count = 1
    index_count = 0
    for t in tracks:
        if min(t) < bbo.maximum_range:
            print bcolors.OKBLUE + "Front" + str(track_count) + " sensing" + bcolors.ENDC
            if len(singularities) != 0:
                first_index = singularities[index_count]
                line = extract_lines(t, 'F', first_index)
                index_count += 1
            else:
                first_index = index[0]
                line = extract_lines(t, 'F', first_index)

            line_storage(line)
    
            track_count += 1
            print "odom_angle: ", bbo.agent_rotation_angle
            print "wall_angle: ", bbo.front_wall_angle
            print bcolors.OKBLUE + "distance to front wall(rho): " +  str(bbo.distance_to_front_wall) + bcolors.ENDC
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
        
    #raw_input("Press ENTER to continue...")
    
    return 1

def sign(x):
    if x < 0.0:
        return -1
    elif x == 0.0:
        return 0
    elif x > 0.0:
        return 1
