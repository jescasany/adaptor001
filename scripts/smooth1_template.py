#!/usr/bin/env python  
"""
Implements smooth controller 1 described in the notes for COMP 4766/6778.

The function set_goal sets the position of the goal in the robot reference
frame.  When get_twist is called a Twist message is returned which is intended
to drive the robot towards this goal.

Andrew Vardy
"""

from geometry_msgs.msg import Twist

class SmoothController1:
    def set_goal(self, goalx, goaly):
        """Set the position of the goal in the robot reference frame."""
        self.goalx = goalx
        self.goaly = goaly

    def get_twist(self):
        # Apply the control law
        twist = Twist()
        twist.linear.x = ??? * self.goalx
        twist.angular.z = ??? * self.goaly
        return twist
