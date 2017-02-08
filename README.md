# adaptor001
27/01/2017
Now, I have changed to Stage + behavior trees.
I have installed:
ROS Indigo in Ubuntu 14.04 with Python 2.7.4, Gazebo 2.2 in iMac computer that has an Intel processor with 2 cores making a total of 2 CPU,s. (I don't know, at this time, how Gazebo uses the processor. However Gazebo runs very slow in this computer.)
Ros by example 1 and 2 have provided RBX1 and RBX2 software that are installed in catkin_ws catkin package. I have made a new directory named turtlebot where my software is installed. These notebooks are saved in Jupyter_notebooks directory in Owncloud.
Transform the original maze into a new image of a corridor with rooms like autolab.
Transform the original turtlebot_in_stage.launch including battery simulator.
Transform the original task_setup.py to set the positions of the approximate center of the rooms and the docking station.
The following files in the turtlebot_stage folder are all the preset launch and configuration files:
$ launch/turtlebot_in_stage.launch
$ maps/maze.png
$ maps/maze.yaml
$ maps/stage/maze.world
$ maps/stage/turtlebot.inc
$ rviz/robot_navigation.rviz
The rviz file is basically a big dump of rviz settings and won't be touched in this tutorial. "turtlebot.inc" defines the layout of the turtlebot and its sensor and will just be mentioned at the end.
We'll use the preset config files and modify these. For this make a copy of the files "world", "yaml" ".and "png" file to a folder of your choice. In the example below we use "~/adaptor001/
There are some typing errors to correct:

sudo gedit  /opt/ros/indigo/share/turtlebot_stage/launch/turtlebot_in_stage.launch

to change "$(find turtlebot_navigation)/launch/includes/amcl.launch.xml"

to "$(find turtlebot_navigation)/launch/includes/amcl/amcl.launch.xml"
cp bitmaps/autolab.png ~/adaptor001/autolab.png
cp bitmaps/autolab.yaml ~/adaptor001/autolab.yaml
cp bitmaps/stage/autolab.world ~/adaptor001/autolab.world
Write on the terminal the following:
roslaunch turtlebot_stage turtlebot_in_stage.launch map_file:="/home/juan/catkin_ws/src/adaptor001/worlds/autolab.yaml" world_file:="/home/juan/catkin_ws/src/adaptor001/worlds/autolab.world" initial_pose_x:=0.8 initial_pose_x:=18.0 initial_pose_y:=-4.5 initial_pose_a:=165.0
And then run:
rosrun rbx2_tasks patrol_tree.py
or,
rosrun rbx2_tasks clean_house_tree.py
Now I have to write code for getting the waypoints to the several rooms and corridors from the graph and map as moving autonomously the robot (turtlebot).
First, copy patrol_tree.py to adaptor001/scripts as wall_follower.py.
Second, change turtlebot_stage/turtlebot_in_stage.launch to turtlebot_stage/adaptor001.launch to add the battery simulator and its params and args.
Then, once we have tested this changes to the code, we copy task_setup.py from rbx2 to wall_follower_setup.py in adaptor001/src.
Copy odom_out_and_back.py from rbx1_nav/nodes to go_to.py in adaptor001/src.
Copy class BlackBoard, class UpdateTaskList and class CheckLocation from clean_house_tree.py from rbx2_tasks/nodes into wall_follower.py in adaptor001/src.
/scan is the topic where listening ranges.
This is the sensor_msgs/LaserScan message definition:
header: 
  seq: 1065
  stamp: 
    secs: 106
    nsecs: 600000000
  frame_id: base_laser_link
angle_min: -0.506145477295
angle_max: 0.506145477295
angle_increment: 0.00158417993225
time_increment: 0.0
scan_time: 0.0
range_min: 0.0
range_max: 5.0
ranges: [...]
intensities: [...]
angle_min <> -30 degrees
angle_max <> +30 degrees
angle_increment <> 0.091 degrees
We have, therefore, 0...639 readings to analyze.
I've done the scan_reading.py in adaptor001/src.
Now I've to write down a behavior tree with all the tasks and, then code each branch and leaf. There is two Python scripts patrol_tree.py and clean_house_tree.py to get inspiration from. Our behavior tree now looks like this:
BEHAVE(sequence)
STAY_HEALTHY(selector)
CHECK BATTERY(condition)
RECHARGE(sequence)
NAV_DOCK
CHARGE
PATROL(selector)
IS_VISITED(condition)
IS_POSSIBLE(condition)
IGNORE_FAILURE
MOVE_3(sequence)
CURRENT_POSITION(action)
MOVE(action)
UPDATE_GRAPH(action)
ADD_MOVE_LIST(action)
IS_ROOM
UPDATE_GRAPH(action)
ROOM_DATA(action)
NAVIGATE(sequence)(loop)
ALLOWED(condition)
CHECK_LOCATION(action)
ASK_GOAL(condition)
WANDER...
The Python scripts that implement the behavior tree above has to be coded with the following ideas that are quoted from IDEAL MOOC Course:
Do not consider the agent's input data as the agent's perception of its environment.
The agent is not a passive observer of reality, but rather constructs a perception of reality through active interaction. The term embodied means that the agent must be a part of reality for this active interaction to happen.
In the embodied view, we design the agent's input (called result r in Figure 1/right) as a result of an experiment initiated by the agent. In simulated environments, we implement r as a function of the experiment and of the state (r = f (e,s) in Figure 1/right).
Focus on sensorimotor interactions rather than separating perception from action.
The expression "to intend to enact" interaction ⟨e,r⟩ means that the agent performs experiment e while expecting result r. As a result of this intention, the agent may "actually enact" interaction ⟨e,r'⟩ if it receives result r' instead of r.
Since sensorimotor agents are not programmed to seek predefined goals, we do not assess their learning by measuring their performance in reaching predefined goals, but by demonstrating the emergence of cognitive behaviors through behavioral analysis.
Cognitive agents must discover, learn, and exploit regularities of interaction.
Regularities of interaction (in short, regularities) are patterns of interaction that occur consistently. Regularities depend on the coupling between the agent and the environment. That is, they depend both on the structure of the environment, and on the possibilities of interaction that the agent has at its disposal.
We define a self-programming agent as an agent that can autonomously acquire executable code and re-execute this code appropriately.
Similar to other machine learning agents, self-programming agents record data in memory as they learn. Self-programming agents also run a predefined program, but this program can control the execution of learned data as sequences of instructions.
To understand the full implication of this definition, it is important to take a cognitive science perspective rather than a software development perspective. A natural cognitive system (an animal) does not have a compiler or an interpreter to exploit a programming language. The only thing at its disposal that remotely resembles an instruction set is the set of interaction possibilities it has with the world around it. The only thing at its disposal that remotely resembles an execution engine is its cognitive system which allows it to execute and learn sequences of interactions with the world.
Self-programming consists of the re-enaction of regularities of interaction.
. . .
Use sensorimotor interactions as primitives of the model.
Spatio-temporal regularities of interaction lead to ontological knowledge of the world.
To design agents that can construct ontological knowledge from spatio-sequential regularities of interaction, we draw inspiration from natural organisms. Natural organisms generally have inborn brain structures that encode space, preparing them to detect and learn spatio-sequential regularities of interaction. We design the policy coupling of our agents by pulling lessons from these natural brain structures, which leads us to a biologically-inspired developmental cognitive architecture.
Design a cognitive coupling capable of generating an increasingly intelligent stream of activity as it develops.
This research endeavor entails two difficulties: 1) designing an increasingly intelligent developmental cognitive coupling, and 2) analyzing the stream of data that it generates to assess its level of intelligence.
Experiments and results can be implemented mainly in the action nodes.
In STAY_HEALTHY node there is no room for experiments and results.
Type Markdown and LaTeX: α2α2
