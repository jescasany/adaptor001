#!/usr/bin/env python

""" advance.py - Version 1.0 2017-03-25

    A basic demo of using the /odom topic to move a robot a given distance
    or rotate through a given angle.

    Based on odom_out_and_back.py from rbx1 apps
"""
import pdb

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, sqrt, pow, degrees

from fancy_prompts import bcolors

def advance(distance, angle, da):
    """ Publisher to control the robot's speed """
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    # How fast will we update the robot's movement?
    rate = 10
    # Set the equivalent ROS rate variable
    r = rospy.Rate(rate)
    # Set the forward linear speed to 0.2 meters per second
    if distance >= 0.0:
        linear_speed = 0.5
    else:
        linear_speed = -0.5
    # Set the travel distance in meters
    goal_distance = abs(distance)
    # Set the rotation speed in radians per second
    if angle < 0.0:
        angular_speed = -0.5
    else:
        angular_speed = 0.5
    # Set the angular tolerance in degrees converted to radians
    angular_tolerance = radians(0.5)
    # Set the rotation angle to angle in radians 
    goal_angle = angle
    # Initialize the tf listener
    tf_listener = tf.TransformListener()
    # Give tf some time to fill its buffer
    rospy.sleep(2)
    # Set the map frame
    map_frame = '/map'
    # Set the odom frame
    odom_frame = '/odom'
    """ Find out if the robot uses /map->/odom transform """
    try:
        tf_listener.waitForTransform(map_frame, odom_frame, rospy.Time(), rospy.Duration(1.0))
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("Cannot find transform between /map and /odom")
        rospy.signal_shutdown("tf Exception")  
    # Find out if the robot uses /base_link or /base_footprint
    try:
        tf_listener.waitForTransform(odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
        base_frame = '/base_footprint'
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        try:
            tf_listener.waitForTransform(odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
            base_frame = '/base_link'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
            rospy.signal_shutdown("tf Exception")  
    # Initialize the position variable as a Point type
    position = Point()   
    # Initialize the movement command
    move_cmd = Twist()
    

    # Get the starting position values     
    (position, rotation) = get_odom(tf_listener, odom_frame, base_frame)
                
    x_start = position.x
    y_start = position.y
    
    # Keep track of the distance traveled
    dist = 0.0
    #pdb.set_trace()
    if da:
        print bcolors.OKGREEN + "da True" + bcolors.ENDC
        print bcolors.OKGREEN + "Empieza distancia" + bcolors.ENDC
        # Set the movement command to forward motion
        move_cmd.linear.x = linear_speed
        bump_count = 0
        # Enter the loop to move along
        while dist < goal_distance and not rospy.is_shutdown():
            #pdb.set_trace()
            last_dist = dist
            # Publish the Twist message and sleep 1 cycle         
            cmd_vel_pub.publish(move_cmd)
            r.sleep()
            # Get the current position
            (position, rotation) = get_odom(tf_listener, odom_frame, base_frame)
            # Compute the Euclidean distance from the start
            dist = sqrt(pow((position.x - x_start), 2) + pow((position.y - y_start), 2))
            
            if dist == last_dist and dist != 0.0:
                bump_count += 1
                print "dist, goal_distance", dist, goal_distance
                print "BUMP"+str(bump_count)
                if bump_count > 10:
                    # Move forward for a time to go the desired distance
                    linear_duration = 1.5/abs(linear_speed) 
                    ticks = int(linear_duration * rate)
                    move_cmd.linear.x *= -1
                    for t in range(ticks):
                        cmd_vel_pub.publish(move_cmd)
                        r.sleep()
                    continue
        # Stop the robot before the rotation
        move_cmd = Twist()
        cmd_vel_pub.publish(move_cmd)
        rospy.sleep(1)
        
        # Set the movement command to a rotation
        move_cmd.angular.z = angular_speed
        # Track the last angle measured
        last_angle = quat_to_angle(rotation)
        print bcolors.OKGREEN + "Empieza angle" + bcolors.ENDC
        # Track how far we have turned
        turn_angle = 0
        done = False
        while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle         
            cmd_vel_pub.publish(move_cmd)
            r.sleep()
            # Get the current rotation
            (position, rotation) = get_odom(tf_listener, odom_frame, base_frame)
            # Compute the amount of rotation since the last loop
            delta_angle = normalize_angle(quat_to_angle(rotation) - last_angle)
            # Add to the running total
            turn_angle += delta_angle
            last_angle = quat_to_angle(rotation)

            if (abs(turn_angle + angular_tolerance) > abs(goal_angle*4/5) or abs(goal_angle) < radians(2)) and not done:
                #pdb.set_trace()
                # Stop the robot before the next leg
                move_cmd = Twist()
                cmd_vel_pub.publish(move_cmd)
                rospy.sleep(1)
                if angle < 0.0:
                    angular_speed = -0.05
                else:
                    angular_speed = 0.05
                # Set the movement command to a rotation
                move_cmd.angular.z = angular_speed
                done = True
                
        # Stop the robot before the next leg
        move_cmd = Twist()
        cmd_vel_pub.publish(move_cmd)
        rospy.sleep(1)
    else:
        print bcolors.OKGREEN + "da False" + bcolors.ENDC
        #pdb.set_trace()
        # Set the movement command to a rotation
        move_cmd.angular.z = angular_speed
        # Track the last angle measured
        last_angle = quat_to_angle(rotation)
        print bcolors.OKGREEN + "Empieza angle" + bcolors.ENDC
        # Track how far we have turned
        turn_angle = 0
        done = False
        while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle         
            cmd_vel_pub.publish(move_cmd)
            r.sleep()
            # Get the current rotation
            (position, rotation) = get_odom(tf_listener, odom_frame, base_frame)
            # Compute the amount of rotation since the last loop
            delta_angle = normalize_angle(quat_to_angle(rotation) - last_angle)
            # Add to the running total
            turn_angle += delta_angle
            last_angle = quat_to_angle(rotation)
#            print "x", position.x
#            print "y", position.y
#            print "la", last_angle
#            print "ta", degrees(turn_angle)
#            print "\n"
            #raw_input("Press ENTER to continue ...")
            if (abs(turn_angle + angular_tolerance) > abs(goal_angle*4/5) or abs(goal_angle) < radians(2)) and not done:
                #pdb.set_trace()
                # Stop the robot before the next leg
                move_cmd = Twist()
                cmd_vel_pub.publish(move_cmd)
                rospy.sleep(1)
                if angle < 0.0:
                    angular_speed = -0.05
                else:
                    angular_speed = 0.05
                # Set the movement command to a rotation
                move_cmd.angular.z = angular_speed
                done = True
                
        # Stop the robot before the next movement
        move_cmd = Twist()
        cmd_vel_pub.publish(move_cmd)
        rospy.sleep(1)
        print bcolors.OKGREEN + "Empieza distancia" + bcolors.ENDC 
        #pdb.set_trace()
        # Get the starting position values     
        (position, rotation) = get_odom(tf_listener, odom_frame, base_frame)
                    
        x_start = position.x
        y_start = position.y
        
        move_cmd.linear.x = linear_speed
        # Keep track of the distance traveled
        dist = 0.0
        bump_count = 0
        # Enter the loop to move along
        while dist < goal_distance and not rospy.is_shutdown():
            last_dist = dist
            # Publish the Twist message and sleep 1 cycle         
            cmd_vel_pub.publish(move_cmd)
            r.sleep()
            # Get the current position
            (position, rotation) = get_odom(tf_listener, odom_frame, base_frame)
            # Compute the Euclidean distance from the start
            dist = sqrt(pow((position.x - x_start), 2) + pow((position.y - y_start), 2))
            
            if dist == last_dist and dist != 0.0:
                bump_count += 1
                print "dist, goal_distance", dist, goal_distance
                print "BUMP"+str(bump_count)
                if bump_count > 10:
                    # Move forward for a time to go the desired distance
                    linear_duration = 1.5/abs(linear_speed) 
                    ticks = int(linear_duration * rate)
                    move_cmd.linear.x *= -1
                    for t in range(ticks):
                        cmd_vel_pub.publish(move_cmd)
                        r.sleep()
                    continue
        # Stop the robot before the rotation
        move_cmd = Twist()
        cmd_vel_pub.publish(move_cmd)
        rospy.sleep(1)

    # Stop the robot for good
    cmd_vel_pub.publish(Twist())
    rospy.sleep(1)

    # Get the current rotation
    (position, rotation) = get_odom(tf_listener, odom_frame, base_frame)
    
    return (position, rotation)

def get_odom(tf_listener, odom_frame, base_frame):
    """ Get the current transform between the odom and base frames """
    try:
        (trans, rot)  = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return

    return (Point(*trans), Quaternion(*rot))


 

