#!/usr/bin/env python

"""
    wall_follow_vel_tree.py - Version 1.0 2017-02-16
    
    Navigate a series of waypoints while monitoring battery levels.
    Uses the pi_trees package to implement a behavior tree task manager.
    
    Based on patrol_tree.py and clean_house_tree.py from rbx2_tasks/nodes insofar as it is related to Behavior trees.
"""
#import pdb

import rospy
import tf
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Point, Pose2D, Quaternion, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from rbx2_msgs.srv import *
from pi_trees_ros.pi_trees_ros import *
#import wall_follower_setup as wfs
from wall_follower_setup import *
from collections import OrderedDict
from math import sqrt, pow, sin, cos, pi, radians, degrees

# A class to track global variables
class BlackBoard():
    def __init__(self):
        # The robot's current position on the map
        self.robot_pose2d = Pose2D()
        
        # The robot's current position on the map
        self.robot_pose = Pose()
        
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
    
class NextWaypoint(Task):
    def __init__(self, *args, **kwargs):
        name = "MOVE_BASE_" + str(black_board.patrol_count)
        super(NextWaypoint, self).__init__(name)
        self.name = name
        
        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Set the odom frame
        self.odom_frame = '/odom'
        
        # Set the map frame
        self.map_frame = '/map'
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait up to 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))    
        
        rospy.loginfo("Connected to move_base action server")
        
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
        
    def run(self):
        
        # Get the starting position values     
        (cp, co) = self.get_odom()
        
        print cp
        print co, degrees(co)
        #raw_input("Press a key to continue...")
        
        quaternion = Quaternion(*quaternion_from_euler(0, 0, co, axes='sxyz'))
        
        d = 3   # distance to move forward
        
        # Append the next waypoint to the list.  Each waypoint
        # is a pose consisting of a position and orientation in the map frame.
        black_board.waypoints.append(Pose(Point(cp.x + d * cos(abs(co)) , cp.y + d *  sin(abs(co)), 0.0), quaternion))
        
        # Create simple action navigation task for each waypoint
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = '/map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = black_board.waypoints[black_board.patrol_count]
        
        print black_board.patrol_count
        print black_board.waypoints[black_board.patrol_count]
        raw_input("Press a key to continue...")
        
        self.move_base.send_goal(goal)
        
        self.move_base.wait_for_result()
        
        result = self.move_base.get_result()
        
        self.update_robot_position()
        
        return TaskStatus.SUCCESS
        
    def update_robot_position(self):
        black_board.robot_pose = self.get_odom()
        black_board.patrol_count += 1
        
        
        
    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.map_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

class WallFollower():
    def __init__(self):
        
        #pdb.set_trace()
        rospy.init_node("wall_follower", anonymous=False)

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # Initialize a number of parameters and variables
        setup_task_environment(self)
        
        # Create simple action navigation task for the first waypoint
#        if black_board.patrol_count == 0:
#            goal = MoveBaseGoal()
#            goal.target_pose.header.frame_id = 'map'
#            goal.target_pose.header.stamp = rospy.Time.now()
#            # Append the first waypoint to the list.
#            black_board.waypoints.append(Pose(Point(18.0, -3.5, 0.0), Quaternion(0.0, 0.0, pi, 1.0)))
#            goal.target_pose.pose = black_board.waypoints[black_board.patrol_count]
#            black_board.MOVE_BASE[black_board.patrol_count] = SimpleActionTask("MOVE_BASE_" + str(black_board.patrol_count), "move_base", MoveBaseAction, goal, reset_after=True, feedback_cb=self.update_robot_position)
        
        # Set the docking station pose
#        goal = MoveBaseGoal()
#        goal.target_pose.header.frame_id = 'map'
#        goal.target_pose.header.stamp = rospy.Time.now()
#        goal.target_pose.pose = self.docking_station_pose
#        
#        # Assign the docking station pose to a move_base action task
#        NAV_DOCK_TASK = SimpleActionTask("NAV_DOCK_TASK", "move_base", MoveBaseAction, goal, reset_after=True, feedback_cb=self.update_robot_position)
        
        # Create the root node
        BEHAVE = Sequence("BEHAVE")
        
        # Create the "stay healthy" selector
        #STAY_HEALTHY = Selector("STAY_HEALTHY")
        
        NEXT_WAYPOINT = NextWaypoint()
        
        # Add the two subtrees to the root node in order of priority
        #BEHAVE.add_child(STAY_HEALTHY)
        BEHAVE.add_child(NEXT_WAYPOINT)
        
        # Add the move_base tasks to the wall_follower task
        #WALL_FOLLOWER.add_child(black_board.MOVE_BASE[black_board.patrol_count])
        
        # Add the battery check and recharge tasks to the "stay healthy" task
#        with STAY_HEALTHY:
#            # The check battery condition (uses MonitorTask)
#            CHECK_BATTERY = MonitorTask("CHECK_BATTERY", "battery_level", Float32, self.check_battery)
#            
#            # The charge robot task (uses ServiceTask)
#            CHARGE_ROBOT = ServiceTask("CHARGE_ROBOT", "battery_simulator/set_battery_level", SetBatteryLevel, 100, result_cb = self.recharge_cb)
#      
#            # Build the recharge sequence using inline syntax
#            RECHARGE = Sequence("RECHARGE", [NAV_DOCK_TASK, CHARGE_ROBOT])
#                
#            # Add the check battery and recharge tasks to the stay healthy selector
#            STAY_HEALTHY.add_child(CHECK_BATTERY)
#            STAY_HEALTHY.add_child(RECHARGE)
                
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
