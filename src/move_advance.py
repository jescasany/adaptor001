#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 13 12:45:33 2017

@author: juan
"""
import pdb

import rospy
from black_board_class import bbo
import black_board as bb
from advance import advance
from fancy_prompts import bcolors
from geometry_msgs.msg import Twist
import math
from adaptor001.msg import ExtractedLine
from adaptor001.msg import ExtractedLines

def singularities_selection():
    print 'bbo.right_singularities: ', bbo.right_singularities
    print 'bbo.front_singularities: ', bbo.front_singularities
    print 'bbo.left_singularities: ', bbo.left_singularities
    print 'bbo.Right: ', bbo.Right
    print 'bbo.Front: ', bbo.Front
    print 'bbo.Left: ', bbo.Left
    pdb.set_trace()
    if True not in bbo.Right and bbo.move_count > 0:
        print "Turning to the right since nothing on the RIGHT"
        bbo.adv_distance = 0.0
        bbo.adv_angle = -math.pi/2
        bbo.da = True
        
    elif True in bbo.Right and True in bbo.Front:
        print "Turning to the left since RIGHT and FRONT are busy"
        bbo.adv_distance = 0.0
        bbo.adv_angle = math.pi/2
        bbo.da = True
        
    elif len(bbo.front_singularities) >= 4:
        dist = bbo.front_distances[1]
        bbo.adv_distance = dist + 1.5
        bbo.adv_angle = 0.0
        bbo.da = True
        
    elif len(bbo.right_singularities) >= 3 and bbo.Right[0:2] == [True, False]:
        corner_index = bbo.right_singularities[1]
        corner_distance = bbo.right_distances[1]
        angle = bbo.angle_min + corner_index * bbo.angle_increment
        dist = corner_distance * math.cos(angle)
        bbo.adv_distance = dist + 1.2
        bbo.adv_angle = -math.pi/2
        bbo.da = True
        
    elif len(bbo.right_singularities) >= 3 and bbo.Right[0:2] == [False, True]:
        corner_index = bbo.right_singularities[1]
        corner_distance = bbo.right_distances[1]
        angle = bbo.angle_min + corner_index * bbo.angle_increment
        dist = corner_distance * math.sin(angle)
        bbo.adv_distance = dist + 1.2
        bbo.adv_angle = 0.0
        bbo.da = True
        
    elif len(bbo.right_singularities) == 0:
        get_close_line()
    
def get_close():
    print bcolors.OKGREEN + "Wall Following" + bcolors.ENDC
    singularities_selection()
    print bcolors.OKBLUE + "bbo.adv_distance: " +  str(bbo.adv_distance) + " m" + bcolors.ENDC
    print bcolors.OKBLUE + "bbo.adv_angle: " +  str(math.degrees(bbo.adv_angle)) + " deg" + bcolors.ENDC 

    # Publish the twist message produced by the controller.
    print bcolors.OKBLUE + "STOP the agent before wall following" + bcolors.ENDC
    bbo.cmd_vel_publisher.publish(Twist())
    rospy.sleep(2)
    bb.laser_scan()
    if bbo.driving_forward:
        print "Moving due to singularity"
        (bbo.agent_position, bbo.agent_rotation) = advance(bbo.adv_distance, bbo.adv_angle, bbo.da)
        rospy.sleep(2)
    else:
        print "Just turning due to singularity"
        (bbo.agent_position, bbo.agent_rotation) = advance(0.0, bbo.adv_angle, bbo.da)
        rospy.sleep(2)
            
    #bbo.agent_rotation_angle = arrange()

def get_close_line():
    """
    The position of the goal in the robot reference frame is specified by
    the values bbo.adv_distance and bbo.adv_angle in the
    robot coordinate frame /base_link.  These are obtained by summing the following two
    vectors.  Note that fo = follow-offset and fa = follow-advance):
    
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
    be reset to 'None' indicating there is no suitable line to follow.
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
        fa = bb.clamp(bbo.distance_front, 1.0, 3.5)  # bbo.distance_front is  min(bbo.kinect_scan[bbo.laser_front_start:bbo.laser_front_end])
        #dist = min(bbo.distance_to_right_wall, line.r)
        dist = bbo.distance_to_right_wall
        print "distance to wall: ", dist
        bbo.adv_distance = math.sqrt(math.pow(dist - fo, 2) + math.pow(fa, 2))
        th = math.pi/2 - math.atan(fa/abs(dist - fo))
        bbo.adv_angle = th
        if (dist - fo) > 0:
            bbo.adv_angle = -th
        bbo.da = False
            
def move_adv():
    print bcolors.OKBLUE + "Estoy en move advance" + bcolors.ENDC
    #pdb.set_trace()
    try:
        #raw_input("Press a key to continue...")
        if bbo.move_count == 0:
            print bcolors.OKGREEN + "PREPARING THINGS IN MY PLACE" + bcolors.ENDC
            # agent_pose is a tuple (position, rotation)
            (bbo.agent_position, bbo.agent_rotation) = advance(0.0, 0.0, bbo.da)
            bb.print_position()
            bbo.waypoints.append((bbo.agent_position, bbo.agent_rotation))
            bbo.move_count += 1
            print bcolors.OKBLUE + "Preparation DONE" + bcolors.ENDC
            #bbo.agent_rotation_angle = arrange()
            return 1
        
        bb.laser_scan()
        rospy.sleep(2)
        if bbo.driving_forward or bbo.adv_distance == 0.0:
            get_close()
            
            bbo.adv_angle = 0.0
            bb.print_position()
#            raw_input("Press a key to continue...")
            bbo.waypoints.append((bbo.agent_position, bbo.agent_rotation))
            
            print bcolors.OKBLUE + "move_adv DONE" + bcolors.ENDC
            bbo.move_fail = False
        else:
            print bcolors.OKBLUE + "move_adv FAILED" + bcolors.ENDC
            (bbo.agent_position, bbo.agent_rotation) = advance(0.0, 0.0, bbo.da)
            
            bbo.move_fail = True
            
    except:
        print bcolors.OKBLUE + "move_adv FAILED" + bcolors.ENDC
        (bbo.agent_position, bbo.agent_rotation) = advance(0.0, 0.0, bbo.da)
        bbo.move_fail = True
        return 1
    return 1

