
# Use ROS from IPython

15/03/2016

I have installed:

ROS Hydro in Ubuntu 12.04 with Python 2.7.3, Gazebo 1.9.6 in Tosh computer that has an Intel processor with 4 cores x 2 threads each making a total of 8 CPU,s. (I don't know, at this time, how Gazebo uses the processor.)

Ros by example 1 and 2 have provided RBX1 and RBX2 software that are installed in hydro_ws catkin package. I have made a new directory named adaptor where my software is installed. These notebooks are saved in adaptor_notebooks directory, for example.




Run  roscore in tosh

Change to the launch directory of your project by doing:

roscd adaptor_gazebo/launch

The next launch file will just execute a launch file provided 
by RBX2, and tells it to load our world file (simple world), our adaptor robot (pi robot with kinect) and show the Gazebo 
client. You can launch it by doing:

roslaunch adaptor_gazebo adaptor_world.launch

Here look for code for pi robot or turtlebot in gazebo https://github.com/jihoonl/mine/tree/master/visualizer/resources/turtlebot_description/urdf or here

https://github.com/Shokoofeh/pi_tracker or

https://thelittlerobotblogs.wordpress.com/2014/02/24/populating-gazebo-worlds/ or

https://github.com/JoshMarino/gazebo_and_ros_control



====================================================================

====================================================================


__27/01/2017__

__Now, I have changed to Stage + behavior trees.__

I have installed:

ROS Indigo in Ubuntu 14.04 with Python 2.7.4, Gazebo 2.2 in iMac computer that has an Intel processor with 2 cores making a total of 2 CPU,s. (I don't know, at this time, how Gazebo uses the processor. However Gazebo runs very slow in this computer.)

Ros by example 1 and 2 have provided RBX1 and RBX2 software that are installed in catkin_ws catkin package. I have made a new directory named turtlebot where my software is installed. These notebooks are saved in Jupyter_notebooks directory in Owncloud.

Transform the original maze into a new image of a corridor with rooms like autolab.

Transform the original turtlebot_in_stage.launch including battery simulator.

Transform the original task_setup.py to set the positions of the approximate center of the rooms and the docking station.

The following files in the turtlebot_stage folder are all the preset launch and configuration files:

```
$ launch/turtlebot_in_stage.launch
$ maps/maze.png
$ maps/maze.yaml
$ maps/stage/maze.world
$ maps/stage/turtlebot.inc
$ rviz/robot_navigation.rviz
```

The rviz file is basically a big dump of rviz settings and won't be touched in this tutorial. "turtlebot.inc" defines the layout of the turtlebot and its sensor and will just be mentioned at the end.

We'll use the preset config files and modify these. For this make a copy of the files "world", "yaml" ".and "png" file to a folder of your choice. In the example below we use "~/adaptor001/


```
There are some typing errors to correct:

sudo gedit  /opt/ros/indigo/share/turtlebot_stage/launch/turtlebot_in_stage.launch

to change "$(find turtlebot_navigation)/launch/includes/amcl.launch.xml"

to "$(find turtlebot_navigation)/launch/includes/amcl/amcl.launch.xml"
```



cp bitmaps/autolab.png  ~/adaptor001/autolab.png

cp bitmaps/autolab.yaml  ~/adaptor001/autolab.yaml

cp bitmaps/stage/autolab.world  ~/adaptor001/autolab.world

Write on the terminal the following:

roslaunch turtlebot_stage adaptor001.launch map_file:="/home/juan/catkin_ws/src/adaptor001/worlds/autolab.yaml" world_file:="/home/juan/catkin_ws/src/adaptor001/worlds/autolab.world" initial_pose_x:=0.8 initial_pose_x:=18.0 initial_pose_y:=-3.5 initial_pose_a:=3.141592

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

====================================================================

====================================================================


__16/02/2017__

The approach above uses move_base and navigation 2D tasks so that you have few possibilities to intervene in details of code.

Furthermore, both navigation 2D tasks and behavior tree approaches are mainly used when you know the locations previously. And this is not the situation with our approach that is trying to navigate without knowing previously the locations where you are moving.

I've done the scan_reading.py in adaptor001/src.

Now I've to write down a behavior tree with all the tasks and, then code each branch and leaf. There is two Python scripts patrol_tree.py and clean_house_tree.py to get inspiration from. Our __behavior tree__ now
looks like this:

* BEHAVE(sequence)
  * STAY_HEALTHY(selector)
    * CHECK BATTERY(condition)
    * RECHARGE(sequence)
      * NAV_DOCK
      * CHARGE
  * WALL_FOLLOW(selector)
    * IS_VISITED(condition)
    * IS_POSSIBLE(condition)
    * IGNORE_FAILURE
      * MOVE_3(sequence)
        * CURRENT_POSITION(action)
        * MOVE(action)
    * UPDATE_GRAPH(action)
    * ADD_MOVE_LIST(action)
  * IS_ROOM
    * UPDATE_GRAPH(action)
    * ROOM_DATA(action)
  * NAVIGATE(sequence)(loop)
    * ALLOWED(condition)
    * CHECK_LOCATION(action)
    * ASK_GOAL(condition)
    * WANDER...

The Python scripts that implement the __behavior tree__ above has to be coded with the following ideas that are quoted from IDEAL MOOC Course:
> **Do not consider the agent's input data as the agent's perception of its environment.**

> The agent is not a passive observer of reality, but rather constructs a perception of reality through active interaction. The term embodied means that the agent must be a part of reality for this active interaction to happen.

> In the embodied view, we design the agent's input (called result r in Figure 1/right) as a result of an experiment initiated by the agent. In simulated environments, we implement r as a function of the experiment and of the state (r = f (e,s) in Figure 1/right).

> **Focus on sensorimotor interactions rather than separating perception from action.**

> The expression "to intend to enact" interaction ⟨e,r⟩ means that the agent performs experiment e while expecting result r. As a result of this intention, the agent may "actually enact" interaction ⟨e,r'⟩ if it receives result r' instead of r.

> Since sensorimotor agents are not programmed to seek predefined goals, we do not assess their learning by measuring their performance in reaching predefined goals, but by demonstrating the emergence of cognitive behaviors through behavioral analysis.

> **Cognitive agents must discover, learn, and exploit regularities of interaction.**

> Regularities of interaction (in short, regularities) are patterns of interaction that occur consistently. Regularities depend on the coupling between the agent and the environment. That is, they depend both on the structure of the environment, and on the possibilities of interaction that the agent has at its disposal.

> **We define a self-programming agent as an agent that can autonomously acquire executable code and re-execute this code appropriately.**

> Similar to other machine learning agents, self-programming agents record data in memory as they learn. Self-programming agents also run a predefined program, but this program can control the execution of learned data as _sequences of instructions_.

> To understand the full implication of this definition, it is important to take a cognitive science perspective rather than a software development perspective. A natural cognitive system (an animal) does not have a compiler or an interpreter to exploit a programming language. The only thing at its disposal that remotely resembles an instruction set is the set of interaction possibilities it has with the world around it. The only thing at its disposal that remotely resembles an execution engine is its cognitive system which allows it to execute and learn _sequences of interactions_ with the world.

> **Self-programming consists of the re-enaction of regularities of interaction.**

> . . .

> **Use sensorimotor interactions as primitives of the model.**

> **Spatio-temporal regularities of interaction lead to ontological knowledge of the world.**

> To design agents that can construct ontological knowledge from spatio-sequential regularities of interaction, we draw inspiration from natural organisms. Natural organisms generally have inborn brain structures that encode space, preparing them to detect and learn spatio-sequential regularities of interaction. We design the policy coupling of our agents by pulling lessons from these natural brain structures, which leads us to a biologically-inspired developmental cognitive architecture.

> **Design a cognitive coupling capable of generating an increasingly intelligent stream of activity as it develops.**

> This research endeavor entails two difficulties: 1) designing an increasingly intelligent developmental cognitive coupling, and 2) analyzing the stream of data that it generates to assess its level of intelligence.

/base_scan is the topic where listening ranges.

The following is the sensor_msgs/LaserScan message definition as defined in turtlebot.inc under kinect ranger:

```
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
```

angle_min <> -30 degrees

angle_max <> +30 degrees

angle_increment <> 0.091 degrees

We have, therefore, 0...639 readings to analyze.

When robot is parallel to wall at a 1 m of distance of it, then readings from 0 to 200 are less than 5.0 since they are measuring actual distances. This sector is equivalent to 18 degrees from angle_max on the right.



![](img/ideal/experimentsresults.png)

__Figure 1:__ Embodied model (right) compared to the traditional model (left). In the traditional model, the cycle conceptually starts with observing the environment (black circle on the environment) and ends by acting on the environment (black arrow on the environment). In the embodied model, the cycle conceptually starts with the agent performing an experiment (black circle on the agent), and ends by the agent receiving the result of the experiment (black arrow on the agent).

Experiments and results can be implemented mainly in the action nodes.

In STAY_HEALTHY node there is no room for experiments and results, since the dock_location is ready known.

Set the Stage simulator for cmd_vel that is more convenient for our approach in the following way:

roscore & rosrun stage_ros stageros $(rospack find stage_ros)/world/autolab.world or you can launch it all together in this way:

roslaunch adaptor001 vel_stage.launch map_file:="/home/juan/catkin_ws/src/adaptor001/worlds/autolab.yaml" world_file:="/home/juan/catkin_ws/src/adaptor001/worlds/autolab.world" initial_pose_x:=18.0 initial_pose_y:=-3.5 initial_pose_a:=3.141592


Since I've made a launch file for this setup altogether with a rviz config that is the following:

vel_stage.launch in adaptor001/launch folder.

![](img/tiposcontexto.png)

__Figure 2:__ Types of singularities the robot can find as it is right-wall following.

====================================================================

====================================================================

# __18/05/2017__

We have integrated our code with the code of Katja Abramova called enactiveAgents.

We have called our code eca_agent00.py, eca_agent01.py, eca_agent02.py and eca_agent03.py.

We have made the last version --eca_agent03.py-- as modular as possible in order to make it more readable, modifiable and maintainable.

This code is working, however we have to tweak valences so that it runs properly.


We have endowed the robot with the ability of feel walls through line extraction. And we manage this lines are represented in rviz as well as their segments, their intersections(corners) and their open discontinuities(doors).

We get the raw laser scanned readings and we can pass a simple filter over them to get singularities from the raw set of singularities. Furthermore, we can fit straight lines to each segment defined by each pair of singularities.

Singularity extraction is not too artificial since it is very similar to what is made by each biological sensor (vision, hearing).

We have to realize that fitting lines to raw scans is somewhat artificial, however it is very useful for our approach of right wall following.

# Line Extraction

* Assume we have a sensor (e.g. laser rangefinder) that returns a sequence of (range, angle) points.

* For the moment, we assume that all of these points come from a single line in the environment (e.g. a wall).

* If we have more than two points then it is unlikely that we can find a single line which passes through all of them.

    * The system is overdetermined (too many equations).
    * We apply least squares optimization to find a line of best fit.
    * In fact, we apply weighted least squares using the uncertainty of data points to weight their impact on the solution.
    
    ![](img/regression.png)

# Segmentation

* We made the assumption above that all data points came from the
same line → this is generally a poor assumption.
* It is necessary to separate a set of measurements into subsets that all
belong to the same feature; This is known as __segmentation__.
* The _Split-and-Merge_ algorithm described below is relatively simple
and was found to be the fastest, and among the most accurate in a
recent study [Nguyen et al., 2005].
* The idea:
    * Fit a line using all points.
    * Determine the point at maximum distance to this line.
    * If the distance of this worst fit point is greater than a threshold, split
the line in two.
    * Continue fitting and splitting until no splittable segments remain.
    * Merge similar lines together.

# Split-and-Merge

1. Initialize a list of lists L. Create list of all points and place it into L.
2. Get the next list, li from L. Fit a line to li .
3. Detect the worst fit point p, which has the largest distance dp to the line.
4. If dp is less than a threshold, go to step 2.
5. Otherwise, split li at p into li1 and li2 . Replace li in L with li1 and li2 .
   Go to step 2.
6. When all lists in L have been checked, merge similar lines.

More details are required for steps 3 and 6...

__Step 3:__
We introduce a parameter called minPointsPerLine to prevent fitting
lines to small sets of points to reduce the impact of noise. Therefore, the worst fit point should not be one of the first or last minPointsPerLine/2 points of the list.

__Step 6:__
"When all lists in L have been checked, merge similar lines."

We can measure the similarity of two lines A = (r1, α1) and B = (r2, α2) by comparing the two differences |r1 − r2| and |α1 − α2| to threshold values.  Note that some care must be taken in computing 
|α1 − α2|. e.g. the difference between 170◦ and −170◦ is 20◦, not 340◦.

Similar adjacent lines are merged by taking all of their source points and fitting a new line.

(The method described above is similar, but slightly simplified from that described in [Nguyen et al., 2005]).

Once we have lines, we can use them directly or else determine __line segments__; the projection of the start and end data points onto the line yield a line segment.

![](img/line_segments.png)

__Corners__ are points of intersection between two lines. We may require that the angle between the lines is large. A corner can be further classified as convex or concave depending upon the orientation of the lines w.r.t. the robot.[Tomatis et al., 2003].

![](img/corners.png)

We can use the same general approach for extracting __circles__ as we used for extracting lines.

![](img/circles.png)

# References

* Arras, K. (1998).
    An introduction to error propagation.
    Technical Report EPFL-ASL-TR-98-01 R3, EPFL.
* Nguyen, V., Martinelli, A., Tomatis, N., and Siegwart, R. (2005).
    A comparison of line extraction algorithms using 2d laser rangefinder for indoor mobile robotics.
    In Proceedings of the IEEE/RSJ Intenational Conference on Intelligent Robots and Systems.
* Tomatis, N., Nourbakhsh, I., and Siegwart, R. (2003).
    Hybrid simultaneous localization and map building: a natural integration of topological and metric.
    Robotics and Autonomous Systems, 44:3–14.


__20/06/2017__

We have integrated our code with the code of Katja Abramova called enactiveAgents.

We have called our new code version eca_agent04.py.
We made our last version --eca_agent03.py-- as modular as possible in order to make it more readable, modifiable and maintainable.

This new code is working, however we have to tweak experiments an their valences so that it runs properly.

Here we have endowed the robot with the ability of feel walls through the use of singularities. And we manage this singularities that are segments of walls, their intersections(corners) and their open discontinuities(doors).

We get the raw laser scanned readings, then we transform them to one reading in six and we can pass a simple filter over them to get singularities from the tramsformed set of singularities.

Singularity extraction is not too artificial since it is very similar to what is made by each biological sensor (vision, hearing).

We have to use now the __is_visited__ behavior. The agent knows that it has been already in this position, so it can handle this situation to navigate.

Also, we have implemented a way to handle the bump situation in Stage, since Stage now doesn't include any driver to bumpers.

