#!/usr/bin/env python

"""
    wall_follower_tree.py - Version 1.0 2017-02-07
    
    Navigate a series of waypoints while monitoring battery levels.
    Uses the pi_trees package to implement a behavior tree task manager.
    
    Based on patrol_tree.py and clean_house_tree.py from rbx2_tasks/nodes
"""

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Point, Pose2D, Quaternion, Pose
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from rbx2_msgs.srv import *
from pi_trees_ros.pi_trees_ros import *
from wall_follower_setup import *
from collections import OrderedDict
from math import sqrt, sin, cos
import time

# A class to track global variables
class BlackBoard():
    def __init__(self):
        # The robot's current position on the map
        self.robot_pose2d = Pose2D()
        
        # The robot's current position on the map
        self.robot_position = Point()
        
        # Create a dictionary to hold navigation tasks for getting to each waypoint
        self.MOVE_BASE = {}
        
        # Create a list to hold the waypoint poses
        self.waypoints = list()
        
        # Initialize the patrol counter
        self.patrol_count = 0

# Initialize the black board
black_board = BlackBoard()

# Create a move list mapping positions to moves.
black_board.MOVE_BASE = OrderedDict()

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
    def __init__(self, *args, **kwargs):
        name = "CHECK_LOCATION_" + str(black_board.patrol_count)
        super(CheckLocation, self).__init__(name)    
        self.name = name

    def run(self):
        wp = black_board.waypoints[black_board.patrol_count].position
        cp = black_board.robot_position
        
        distance = sqrt((wp.x - cp.x) * (wp.x - cp.x) +
                        (wp.y - cp.y) * (wp.y - cp.y) +
                        (wp.z - cp.z) * (wp.z - cp.z))
                                
        if distance < 0.15:
            status = TaskStatus.SUCCESS
        else:
            status = TaskStatus.FAILURE
            
        return status
    
class NextWaypoint(Task):
    def __init__(self, *args, **kwargs):
        name = "NEXT_WAYPOINT_" + str(black_board.patrol_count + 1)
        super(NextWaypoint, self).__init__(name)
        self.name = name
        
    def run(self):
        black_board.patrol_count += 1
        
        cp = black_board.robot_pose2d
        co = black_board.robot_pose2d.theta
        quaternion = Quaternion(*quaternion_from_euler(0, 0, co, axes='sxyz'))
            
        # Append the next waypoint to the list.  Each waypoint
        # is a pose consisting of a position and orientation in the map frame.
        black_board.waypoints.append(Pose(Point(cp.x * (1 + cos(co)) , cp.y * (1 + sin(co)), 0.0), quaternion))
        
        # Create simple action navigation task for each waypoint
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = black_board.waypoints[black_board.patrol_count]
        black_board.MOVE_BASE[black_board.patrol_count] = SimpleActionTask("MOVE_BASE_" + str(black_board.patrol_count), "move_base", MoveBaseAction, goal, reset_after=True, feedback_cb=tree.update_robot_position)
        
        return TaskStatus.SUCCESS

class WallFollower():
    def __init__(self):
        rospy.init_node("wall_follower", anonymous=False)

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # Initialize a number of parameters and variables
        setup_task_environment(self)
        
        # Set the docking station pose
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.docking_station_pose
        
        # Assign the docking station pose to a move_base action task
        NAV_DOCK_TASK = SimpleActionTask("NAV_DOCK_TASK", "move_base", MoveBaseAction, goal, reset_after=True, feedback_cb=self.update_robot_position)
        
        # Create the root node
        BEHAVE = Sequence("BEHAVE")
        
        # Create the "stay healthy" selector
        STAY_HEALTHY = Selector("STAY_HEALTHY")
        
        NEXT_WAYPOINT = NextWaypoint()
        
        # Create the patrol
        WALL_FOLLOWER = Sequence("WALL_FOLLOWER")
        
        # Add the two subtrees to the root node in order of priority
        BEHAVE.add_child(STAY_HEALTHY)
        BEHAVE.add_child(NEXT_WAYPOINT)
        BEHAVE.add_child(WALL_FOLLOWER)
        
        print "black_board.patrol_count: ", black_board.patrol_count
        raw_input("Press a key to continue...")
        
        # Add the move_base tasks to the wall_follower task
        WALL_FOLLOWER.add_child(black_board.MOVE_BASE[black_board.patrol_count])
        
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
        print "Behavior Tree\n"
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
#        self.patrol_count += 1
#        angle = msg.base_position.pose.orientation
#        print angle.x
#        print angle.y
#        print angle.z
#        print angle.w
#        raw_input("Press key to continue...")
#        quaternion = Quaternion(*quaternion_from_euler(0, 0, pi, axes='sxyz'))
#        self.waypoints.append(Pose(Point(15.0, -3.5, 0.0), quaternion))
          
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_all_goals()
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        tree = WallFollower()
    except rospy.ROSInterruptException:
        rospy.loginfo("Wall following finished.")
