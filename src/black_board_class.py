#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed May 10 11:14:29 2017

@author: juan
"""

import rospy

from geometry_msgs.msg import Twist, Point, Quaternion

from adaptor001.msg import ExtractedLine
from adaptor001.msg import ExtractedLines
from visualization_msgs.msg import Marker

""" A class to track black_board.variables """
class BlackBoard:
    
    def __init__(self):
        
        # Initialize the move counter
        self.move_count = 0
        # Initialize the line counter
        self.line_count = 0
        # initialize one simulation step (that might consist of several primitive steps)
        self.sim_step = 1
        # Initialize a number of variables for the blackboard
        self.kinect_scan = list()
        self.filtered_scan = list()
        # Initialize distance and angle to advance
        self.adv_distance = 1.0      # meters
        self.adv_angle = 0.0     # radians
        
        self.driving_forward = True      # is True if there is no obstacle ahead 
        self.move_fail = False       #  whether move_adv fails or not
        
        self.limit_range = 5.1
        self.distance_to_right_wall = 1.5
        self.last_distance = 1.5
        self.distance_to_left_wall = 5.0
        self.right_wall_angle = 0.0
        self.left_wall_angle = 0.0
        self.front_wall_angle = 0.0
        self.distance_to_front_wall = 7.0
        self.Left = list()
        self.Left1 = False
        self.Left2 = False
        self.Left3 = False
        self.Right = list()
        self.Right1 = False
        self.Right2 = False
        self.Right3 = False
        self.Front = list()
        self.Front1 = False
        self.Front2 = False
        self.Front3 = False
        self.right_singularities = list()
        self.left_singularities = list()
        self.front_singularities = list()
        self.environment_status = False
        self.arrange_status = False
        # The agent's current position on the map
        self.agent_position = Point()
        self.agent_rotation = Quaternion()
        self.agent_pose = (Point(), Quaternion())
        self.agent_rotation_angle = 0.0
        # Create a dictionary to hold navigation waypoints
        self.waypoints = list()
        (self.agent_position, self.agent_rotation) = self.agent_pose 
        self.odom_angle = 0.0
        self.agent_mechanism = ''    # to choose among simple, recursive and constructive mechanisms
        self.boredom = False
        
        self.line = ExtractedLine()
        self.lines = ExtractedLines()
        self.lines.header.frame_id = '/base_link'
    
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
        self.extracted_publisher = rospy.Publisher('/extracted_lines', ExtractedLines, queue_size=100)
        # We will publish to vis_lines_topic which will show the lines (as
        # line segments) in RViz.  We will also publish the first and last scan
        # points to topic vis_scanpoints_topic in the same colour so that it
        # is clear from what range of scan points the lines are generated.
        self.lines_publisher = rospy.Publisher(self.vis_lines_topic, Marker, queue_size=100)
        self.scanpoints_publisher = rospy.Publisher(self.vis_scanpoints_topic, Marker, queue_size=100)
        self.selected_lines_publisher = rospy.Publisher('/extracted_lines_wf', \
                               ExtractedLines, queue_size=100)

        
# Initialize the blackboard
bbo = BlackBoard()
   