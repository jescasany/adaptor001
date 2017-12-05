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
    #pdb.set_trace()
        
    if not (True in bbo.Right) and bbo.move_count > 0:
        print "Turning to the right since nothing on the RIGHT"
        bbo.adv_distance = 0.0
        bbo.adv_angle = -math.pi/2
        bbo.da = True
        bbo.singularity_selection = 1
        bbo.rf = True
    
    elif True in bbo.Right and len(bbo.right_singularities) == 0 and True in bbo.Front and bbo.corner1 == False:
        print "Turning to the left since RIGHT and FRONT are busy"
        bbo.adv_distance = 0.0
        bbo.adv_angle = math.pi/2
        bbo.da = True
        bbo.singularity_selection = 2
        
    elif len(bbo.right_singularities) >= 3 and bbo.Right[0:2] == [True, False]:
        if bbo.corner1 == True:
            bbo.corner1 = False
        corner_index = bbo.right_singularities[1]
        corner_distance = bbo.right_distances[1]
        angle = bbo.angle_min + corner_index * bbo.angle_increment
        dist = corner_distance * math.cos(angle)
        bbo.adv_distance = dist + 0.5
        bbo.adv_angle = 0.0
        bbo.da = True
        bbo.singularity_selection = 3
        bbo.corner1 = True
        
    elif len(bbo.right_singularities) >= 3 and bbo.Right[0:2] == [False, True]:
        corner_index = bbo.right_singularities[-2]
        corner_distance = bbo.right_distances[-2]
        angle = bbo.angle_min + corner_index * bbo.angle_increment
        dist = corner_distance * math.sin(angle)
        bbo.adv_distance = dist + 0.5
        if bbo.corner1 == True and bbo.rf == True:
            bbo.adv_angle = 0.0
            bbo.da = False
            bbo.corner1 = False
            bbo.rf = False
        elif bbo.corner1 == True:
            bbo.adv_angle = -math.pi/2
            bbo.da = False
        else:
            bbo.adv_angle = 0.0
            bbo.da = True
        bbo.singularity_selection = 4
        
    elif len(bbo.right_singularities) == 0:
        bbo.singularity_selection = 5
        bbo.rf = False
        get_close_line()
        
def get_close():
    print bcolors.OKGREEN + "Wall Following" + bcolors.ENDC
    singularities_selection()
    # Publish the twist message produced by the controller.
    print bcolors.OKBLUE + "STOP the agent before wall following" + bcolors.ENDC
    bbo.cmd_vel_publisher.publish(Twist())
    rospy.sleep(2)
    print bcolors.OKBLUE + "Agent STOPPED" + bcolors.ENDC
    bb.laser_scan()
    bbo.adv_distance = bb.clamp(bbo.adv_distance, 0.0, 4.0)
    print bcolors.OKBLUE + "bbo.adv_distance: " +  str(bbo.adv_distance) + " m" + bcolors.ENDC
    print bcolors.OKBLUE + "bbo.adv_angle: " +  str(math.degrees(bbo.adv_angle)) + " deg" + bcolors.ENDC 
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
    vectors.  Note that fo = follow-offset and fa = follow-advance:
    
    Vector from the origin to the closest point on the line, with a length
    of r - fo (r: orthogonal distance of the line) 
           [(r-fo) cos(m), (r-fo) sin(m)]
    
    We use dist (bbo.distance_to_right_wall) instead of r.
    
    Vector along the line, oriented towards counter-clockwise with length fa
           [fa cos(m+pi/2), fa sin(m+pi/2)]
    """
    print bcolors.OKGREEN + "Wall Following" + bcolors.ENDC


    fo = bbo.follow_offset
    fo = 1.5
    #fa = bbo.follow_advance
    fa = bbo.distance_front  # bbo.distance_front is  min(bbo.raw_kinect_scan[bbo.laser_front_start:bbo.laser_front_end])
    #dist = min(bbo.distance_to_right_wall, line.r)
    dist = bbo.distance_to_right_wall
    print "distance to wall: ", dist
    print "distance to front: ", bbo.distance_front
    if dist < 1.0 or dist > 2.0:
        bbo.adv_distance = math.sqrt(math.pow(dist - fo, 2) + math.pow(fa, 2))
        th = math.pi/2 - math.atan(fa/abs(dist - fo))
        bbo.adv_angle = th
        if (dist - fo) > 0:
            bbo.adv_angle = -th
        bbo.da = False
    else:
        bbo.adv_distance = bbo.distance_front - 1.7
        bbo.adv_angle = 0.0
        bbo.da = True
            
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
            bbo.da = False
            (bbo.agent_position, bbo.agent_rotation) = advance(0.5, math.pi, bbo.da)
            rospy.sleep(2)
            (bbo.agent_position, bbo.agent_rotation) = advance(0.0, -(math.pi), bbo.da)
            bbo.move_fail = True
            
    except:
        print bcolors.OKBLUE + "move_adv FAILED" + bcolors.ENDC
        (bbo.agent_position, bbo.agent_rotation) = advance(0.0, 0.0, bbo.da)
        bbo.move_fail = True
        return 1
    return 1

