#!/usr/bin/env python

""" go_to.py - Version 1.0 2017-01-27

    A basic demo of using the /odom topic to move a robot a given distance
    or rotate through a given angle.

    Based on odom_out_and_back.py from rbx1 apps
"""

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, sqrt, pow, degrees

class GoTo():
    def __init__(self, distance, angle):
        # Give the node a name
        rospy.init_node('go_to', anonymous=False)

        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        
        # How fast will we update the robot's movement?
        rate = 20
        
        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)
        
        # Set the forward linear speed to 0.15 meters per second 
        linear_speed = 0.2
        
        # Set the travel distance in meters
        goal_distance = distance

        # Set the rotation speed in radians per second
        angular_speed = 0.5
        
        # Set the angular tolerance in degrees converted to radians
        angular_tolerance = radians(1.0)
        
        # Set the rotation angle to angle in radians 
        goal_angle = angle

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Set the odom frame
        self.odom_frame = '/odom'
        
        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")  
        
        # Initialize the position variable as a Point type
        position = Point()
            
        # Initialize the movement command
        move_cmd = Twist()
        
        # Set the movement command to forward motion
        move_cmd.linear.x = linear_speed
        
        # Get the starting position values     
        (position, rotation) = self.get_odom()
                    
        x_start = position.x
        y_start = position.y
        
        rot_deg = degrees(rotation)
        
        print position
        print rot_deg
        raw_input("Press a key to continue...")
        
        # Keep track of the distance traveled
        dist = 0
        
        # Enter the loop to move along a side
        while dist < goal_distance and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle         
            self.cmd_vel.publish(move_cmd)
            
            r.sleep()
    
            # Get the current position
            (position, rotation) = self.get_odom()
            
            # Compute the Euclidean distance from the start
            dist = sqrt(pow((position.x - x_start), 2) + pow((position.y - y_start), 2))

        # Stop the robot before the rotation
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)
        
        # Set the movement command to a rotation
        move_cmd.angular.z = angular_speed
        
        # Track the last angle measured
        last_angle = rotation
        
        # Track how far we have turned
        turn_angle = 0
        
        while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle         
            self.cmd_vel.publish(move_cmd)
            r.sleep()
            
            # Get the current rotation
            (position, rotation) = self.get_odom()
            
            # Compute the amount of rotation since the last loop
            delta_angle = normalize_angle(rotation - last_angle)
            
            # Add to the running total
            turn_angle += delta_angle
            last_angle = rotation
            
        # Stop the robot before the next leg
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)
        
        # Stop the robot for good
        self.cmd_vel.publish(Twist())
        
    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))
        
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        GoTo(3.0, 0.0)
    except:
        rospy.loginfo("go-to node terminated.")

