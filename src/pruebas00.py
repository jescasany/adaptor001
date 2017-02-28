#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 16 11:59:11 2017

@author: juan
"""
import pdb
import rospy
import numpy as np
import pylab
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rbx2_msgs.srv import *
from pi_trees_ros.pi_trees_ros import *
from rbx2_tasks.task_setup import *
from advance import *
import sensor_msgs.msg

# A class to track global variables
class BlackBoard():
    def __init__(self):
        
        # The robot's current position on the map
        self.robot_pose = Pose()
        
        # Create a dictionary to hold navigation waypoints
        self.waypoints = list()
        
        # Initialize the patrol counter
        self.move_count = 0
        
    def print_position(self):
        print self.move_count
        print self.robot_pose
    
    def plotter(self, y):    
    # Uncomment these next few lines to display a constantly updating graph of data.
    # Note: when the graph appears, you must close it first, and then the graph will reopen
    # and display the data as it comes in.
        x = []
        x = range(len(y))
        pylab.figure(1)
        pylab.clf()
        pylab.plot(x,y)
        pylab.draw()
        pylab.show()
        
        raw_input("Press a key to continue...") 
        

# Initialize the black board
black_board = BlackBoard()
# Initialize a list to hold the waypoint poses
black_board.robot_pose = (0.0, 0.0, 0.0, 0.0)
black_board.kinect_scan = list()
black_board.filtered_scan = list()


class Pruebas():
    def __init__(self):
        pdb.set_trace()
        rospy.init_node("pruebas_tree")

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # Create the root node
        BEHAVE = Sequence("BEHAVE")
        
        WALL_FOLLOW = Sequence("WALL_FOLLOW")

        MOVE_3 = CallbackTask("move_3", self.move_3)
        
        IS_VISITED = CallbackTask("is_visited", self.is_visited)
        
        IS_POSSIBLE = CallbackTask("is_possible", self.is_possible)
        
        BEHAVE.add_child(WALL_FOLLOW)
        
        WALL_FOLLOW.add_child(MOVE_3)
        WALL_FOLLOW.add_child(IS_VISITED)
        WALL_FOLLOW.add_child(IS_POSSIBLE)
        
        # Display the tree before beginning execution
        print "Patrol Behavior Tree"
        print_tree(BEHAVE)
        
        # Run the tree
        while not rospy.is_shutdown():
            BEHAVE.run()
            rospy.sleep(0.1)
        
    def move_3(self):
        try:
            raw_input("Press a key to continue...")
            if black_board.move_count == 0:
                black_board.robot_pose = advance(0.0, 0.0)
                black_board.print_position()
                black_board.waypoints.append(black_board.robot_pose)
                black_board.move_count += 1
                rospy.loginfo("move_3 done.")
            black_board.robot_pose = advance(3.0, 0.0)
            black_board.print_position()
#          raw_input("Press a key to continue...")
            black_board.waypoints.append(black_board.robot_pose)
            black_board.move_count += 1
            rospy.loginfo("move_3 done.")
        except:
            rospy.loginfo("move_3 failed.")
            
        return True
    
    def is_visited(self):
        if black_board.robot_pose in black_board.waypoints:
            return True
        rospy.loginfo("is not visited.")
        return False
    
    def formule(self, L):
        f = 0
        f = L[1] - (L[0] + L[2]) / 2
        return f
        
    def moving_window_filtro(self, x, n_neighbors=1):
        n = len(x)
        width = n_neighbors*2 + 1
        x = [x[0]]*n_neighbors + x + [x[-1]]*n_neighbors
        # To complete the function,
        # return a list of the filtered values from i to i+width for all values i from 0 to n-1.
        filtro = []
        for i in range(n):
            fi = self.formule(x[i:i+width])
            filtro.append(fi)
            
        return filtro
    
    def is_possible(self):
        self.laser_scan()
        rospy.sleep(2)
        y = list()
        y = black_board.kinect_scan
        black_board.plotter(black_board.kinect_scan)
        black_board.filtered_scan = self.moving_window_filtro(y, 1)
        black_board.plotter(black_board.filtered_scan)
        #update distanceToWall
        distance_to_wall = sum(y) / float(len(y))
        print "distanceToWall: ", distance_to_wall
        
        #raw_input("Press a key to continue...")
        
        return True
    
    def laser_scan(self):
        rospy.loginfo("Waiting for /base_scan topic...")
        rospy.wait_for_message('/base_scan', sensor_msgs.msg.LaserScan)
        # Subscribe the /base_scan topic to get the range readings  
        rospy.Subscriber('/base_scan', sensor_msgs.msg.LaserScan, self.scan_callback, queue_size = 10)
        rospy.loginfo("laser_scan done")
        
    
    def scan_callback(self, msg):
        black_board.kinect_scan = list(msg.ranges)
    
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    tree = Pruebas()