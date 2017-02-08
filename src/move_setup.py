#!/usr/bin/env python

""" move_setup.py - Version 1.0 2017-02-07

    Set up a charging station for use with simulated tasks using
    behavior trees.

    Based on task_setup.py from rbx2_tasks/src/rbx2_tasks/.
      
"""

import rospy
import actionlib
from actionlib import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import  pi
from collections import OrderedDict

def setup_task_environment(self):
    # How big is the square we want the robot to patrol?
    self.square_size = rospy.get_param("~move_distance", 3.0) # meters
    
    # Set the low battery threshold (between 0 and 100)
    self.low_battery_threshold = rospy.get_param('~low_battery_threshold', 30)
    
    # How long do we have to get to each waypoint?
    self.move_base_timeout = rospy.get_param("~move_base_timeout", 10) #seconds
    
    # Initialize the patrol counter
    self.patrol_count = 0
    
    # Subscribe to the move_base action server
    self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    
    rospy.loginfo("Waiting for move_base action server...")
    
    # Wait up to 60 seconds for the action server to become available
    self.move_base.wait_for_server(rospy.Duration(60))    
    
    rospy.loginfo("Connected to move_base action server")
    
    # Create a variable to hold the target quaternion (orientation)
    quaternion = Quaternion(quaternion_from_euler(0, 0, pi, axes='sxyz'))
    
    # Create a list to hold the waypoint poses
    self.waypoints = list()
            
    # Append each of the four waypoints to the list.  Each waypoint
    # is a pose consisting of a position and orientation in the map frame.
    self.waypoints.append(Pose(Point(15.0, -3.5, 0.0), quaternion))
    
    # Where is the docking station?
    self.docking_station_pose = (Pose(Point(18.0, -7.5, 0.0), Quaternion(0.0, 0.0, pi, 1.0)))            
        
    # Set a marker for the docking station
    init_docking_station_marker(self)
        
    # Publisher to manually control the robot (e.g. to stop it)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
    
    rospy.loginfo("Starting Moves")
    
    # Publish the docking station marker
    self.docking_station_marker_pub.publish(self.docking_station_marker)
    rospy.sleep(1)

def init_docking_station_marker(self):
    # Define a marker for the charging station
    marker_scale = 0.3
    marker_lifetime = 0 # 0 is forever
    marker_ns = 'waypoints'
    marker_id = 0
    marker_color = {'r': 0.7, 'g': 0.7, 'b': 0.0, 'a': 1.0}
    
    self.docking_station_marker_pub = rospy.Publisher('docking_station_marker', Marker, queue_size=5)
    
    self.docking_station_marker = Marker()
    self.docking_station_marker.ns = marker_ns
    self.docking_station_marker.id = marker_id
    self.docking_station_marker.type = Marker.CYLINDER
    self.docking_station_marker.action = Marker.ADD
    self.docking_station_marker.lifetime = rospy.Duration(marker_lifetime)
    self.docking_station_marker.scale.x = marker_scale
    self.docking_station_marker.scale.y = marker_scale
    self.docking_station_marker.scale.z = 0.02
    self.docking_station_marker.color.r = marker_color['r']
    self.docking_station_marker.color.g = marker_color['g']
    self.docking_station_marker.color.b = marker_color['b']
    self.docking_station_marker.color.a = marker_color['a']
    
    self.docking_station_marker.header.frame_id = 'odom'
    self.docking_station_marker.header.stamp = rospy.Time.now()
    self.docking_station_marker.pose = self.docking_station_pose
