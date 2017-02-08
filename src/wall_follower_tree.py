#!/usr/bin/env python

"""
    wall_follower_tree.py - Version 1.0 2017-02-07
    
    Navigate a series of waypoints while monitoring battery levels.
    Uses the pi_trees package to implement a behavior tree task manager.
    
    Based on patrol_tree.py and clean_house_tree.py from rbx2_tasks/nodes
"""

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from rbx2_msgs.srv import *
from pi_trees_ros.pi_trees_ros import *
from wall_follower_setup import *
from collections import OrderedDict
from math import pi, sqrt
import time

# A class to track global variables
class BlackBoard():
    def __init__(self):
        # A list to store the mapping of rooms to tasks
        self.move_list = list()
        
        # The robot's current position on the map
        self.robot_position = Point()

# Initialize the black board
black_board = BlackBoard()

# Create a move list mapping positions to moves.
black_board.move_list = OrderedDict()

class UpdateMoveList:
    def __init__(self, move):
        self.move = move

    def run(self):
        try:
            black_board.move_list.append(self.move)
        except:
            pass
        
        return True

class CheckLocation(Task):
    def __init__(self, waypoints, *args, **kwargs):
        name = "CHECK_LOCATION_" + self.patrol_count
        super(CheckLocation, self).__init__(name)    
        self.name = name
        self.waypoints = waypoints

    def run(self):
        wp = self.waypoints[self.patrol_count].position
        cp = black_board.robot_position
        
        distance = sqrt((wp.x - cp.x) * (wp.x - cp.x) +
                        (wp.y - cp.y) * (wp.y - cp.y) +
                        (wp.z - cp.z) * (wp.z - cp.z))
                                
        if distance < 0.15:
            status = TaskStatus.SUCCESS
        else:
            status = TaskStatus.FAILURE
            
        return status

class WallFollower():
    def __init__(self):
        rospy.init_node("wall_follower", anonymous=False)

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # Initialize a number of parameters and variables
        setup_task_environment(self)
        
        # Create a dictionary to hold navigation tasks for getting to each room
        MOVE_BASE = {}
        
        waypoints = list()
        
        # Create simple action navigation task for each waypoint
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.waypoints[self.patrol_count]
        MOVE_BASE[self.patrol_count] = SimpleActionTask("MOVE_BASE_" + str(self.patrol_count), "move_base", MoveBaseAction, goal, reset_after=True, feedback_cb=self.update_robot_position)
        
        # Set the docking station pose
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.docking_station_pose
        
        # Assign the docking station pose to a move_base action task
        MOVE_BASE['dock'] = SimpleActionTask("MOVE_BASE_DOCK", "move_base", MoveBaseAction, goal, reset_after=True, feedback_cb=self.update_robot_position)
        
        # Create the root node
        BEHAVE = Sequence("BEHAVE")
        
        # Create the "stay healthy" selector
        STAY_HEALTHY = Selector("STAY_HEALTHY")
        
        # Create the patrol loop decorator
        WALL_FOLLOWER = Selector("WALL_FOLLOWER")
        
        # Add the two subtrees to the root node in order of priority
        BEHAVE.add_child(STAY_HEALTHY)
        BEHAVE.add_child(WALL_FOLLOWER)
        
        
        # Add the battery check and recharge tasks to the "stay healthy" task
        with STAY_HEALTHY:
            # The check battery condition (uses MonitorTask)
            CHECK_BATTERY = MonitorTask("CHECK_BATTERY", "battery_level", Float32, self.check_battery)
            
            # The charge robot task (uses ServiceTask)
            CHARGE_ROBOT = ServiceTask("CHARGE_ROBOT", "battery_simulator/set_battery_level", SetBatteryLevel, 100, result_cb = self.recharge_cb)
      
            # Build the recharge sequence using inline syntax
            RECHARGE = Sequence("RECHARGE", [MOVE_BASE['dock'], CHARGE_ROBOT])
                
            # Add the check battery and recharge tasks to the stay healthy selector
            STAY_HEALTHY.add_child(CHECK_BATTERY)
            STAY_HEALTHY.add_child(RECHARGE)
                
        # Display the tree before beginning execution
        print "Wall Follower Tree"
        print_tree(BEHAVE)
        
        # Run the tree
        while not rospy.is_shutdown():
            BEHAVE.run()
            rospy.sleep(0.1)
            
    def check_battery(self, msg):
        if msg.data is None:
            return TaskStatus.RUNNING
        else:
            if msg.data < self.low_battery_threshold:
                rospy.loginfo("LOW BATTERY - level: " + str(int(msg.data)))
                return TaskStatus.FAILURE
            else:
                return TaskStatus.SUCCESS
    
    def recharge_cb(self, result):
        rospy.loginfo("BATTERY CHARGED!")
        return TaskStatus.SUCCESS
  
    def update_robot_position(self, msg):
        black_board.robot_position = msg.base_position.pose.position
          
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_all_goals()
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    tree = WallFollower()

