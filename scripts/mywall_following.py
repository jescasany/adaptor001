#!/usr/bin/env python

""" 
    mywall_following.py - Version 1.0 2015-05-21
    
    Juan Escasany for make mybot following walls      
"""
import roslib
import rospy

from std_msgs.msg import String

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import cv_bridge
import cv

import numpy as np
import sensor_msgs.msg as sm
import pylab
import scipy.optimize as sci
import math
import time
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle

import laser_bueno01

# We want these values to be global, but we haven't yet given them a value.
# NOTE: Python throws a warning as a result of running things in this fashion, but you'll
# be fine.
global distance_to_wall
global Front
global Back 
global Left
global Right
global odom_angle

numIncrement = 5
fitfunc = lambda p, x: p[0] * x + p[1]
errfunc = lambda p, x, y: fitfunc(p, x) - y
full = 200
wantedAngle = -22
findWall = 0
followWall = 1

# IMPORTANT NOTE:  All functions that take in a laserScan are actually 
# taking in a filtered laserScan where all NaN data are removed from the list.

PreviousDirection = "none"
CurrentLaserScan = np.asarray([])
Intersection = True       # default is False.  Set True for testing
distanceToWall = 0 
wallAngle = 0
depthsum = 0

def HandleDepthData(data1):
    '''
    HandleDepthData takes in the depth image and updates 
    the global CurrentLaserScan, the current wallAngle, 
    and the average distanceToWall
    '''
    
    global CurrentLaserScan
    
    global image
    global wallAngle
    global distanceToWall
    global on
    global wallFollowMode
    
    # gets an image 240 x 320 and one channel
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv(data1, "16UC1")
    
    # gets the pixel x location (because we're only taking the valid points
    x = []
    # gets the distances
    y = []
    
    for i in range(0, 319, numIncrement):
        # add all the valid points and their corresponding locations on the
        # image.
        
        if not math.isnan(image[120, i]):
            x += [i]
            y += [image[120,i]]
    x = np.asarray(x)
    y = np.asarray(y)

    #update distanceToWall
    distance_to_wall = y.sum() / y.size
    
    print "distanceToWall", distanceToWall
    
    raw_input("Press a key to continue...")

# Uncomment these next few lines to display a constantly updating graph of data.
# Note: when the graph appears, you must close it first, and then the graph will reopen
# and display the data as it comes in.
    
    #pylab.figure(1)
    #pylab.clf()
    #pylab.plot(x,y)
    #pylab.draw()
    #pylab.show()
    
    #raw_input("Press a key to continue...") 
    
    x = x * 3.26 / 320.
    
    if y.size < 2:
        print "not enough data.  Too many NaN values thrown out."
        distance_to_wall = 0
        wallAngle = 0
        return
        
    guess = (0, 0)
    coefficients, success = sci.leastsq(errfunc, guess[:], args = (x, y), maxfev = 10000)
    # update wallAngle
    wallAngle= math.atan(coefficients[0])*360./(2*math.pi)
    #update the CurrentLaserScan
    CurrentLaserScan = y
    
    print "y", y
    
    raw_input("Press...")
    
    
def intersection_prompt():
    '''takes in a filtered laser scan, angle, and degrees from the displayplot.  
    '''
    global Intersection
    global Right
    global CurrentLaserScan
    
    # if we detect an opening to our right:
    if RightSensing(CurrentLaserScan) == True:
        print "Right Open"
        Right = True
        Intersection = True
    else:
        Right = False
        Intersection = False

        print "no intersection"
        print
        print

def Front_Detector():
    ''' updates the global Front boolean if the front is open
    '''
    global Intersection
    global Front
    global CurrentLaserScan
    global odom_angle

    if Intersection == True:
        print "looking for front"
        # look to the left
        tank(0, 0.2)
        initial_angle = odom_angle
        max_angle = 0.5
        odometry_reading = 0
        avg_dist = 0

        while odometry_reading < max_angle:
            # update odometry_reading
            odometry_reading = math.fabs(odom_angle - initial_angle)
            
            ThreeWay = FrontandRight(CurrentLaserScan)
            # look for an open passage ahead
            if ThreeWay == True:
                print "Front Open"
                Front = True
                return
            
        tank(0, 0)
        print "no front detected after scanning"
        Front = False
        print "sleeping for 1"
        time.sleep(1)
        
    else:
        Front = False
        return

def Left_Detector():
    ''' takes an initial scan of the global CurrentLaserScan data while looking forward,
        rotates a set amount, and looks at the change in distance.
        It then uses this amount to determine the left-part of the environment.'''
    global distanceToWall
    global Intersection
    global odom_angle
    global Left
        # hard-coded temporary left-hallway and corner distance values
        # these values are what we'd LIKE to see from the kinect.
    corner = 2.6
    left_hallway = 4.4
    simpleWall = 3.6

    if Intersection == True:
        # start moving around to scan for distances
        tank(0,0.2)
        # enter a scan loop
        # we know the value of the corner and left hallway, so we'll continue scanning
        # and looping until we find a reading close to that value.
        initial_angle = odom_angle  # The old odom_angle, because odom_angle is constantly
                                    # getting updated
        odometry_reading = 0
        max_angle = 1.5 # radians
        # if we've rotated farther than 90, then we can't detect what we're looking for
        # anymore
        
        while odometry_reading < max_angle:
            # take two readings and do math on them
            current_distance = distanceToWall
            # update the odometry reading
            odometry_reading = math.fabs((odom_angle - initial_angle))
            print "odometry_reading is "+str(odometry_reading)
            
            #compute errors
            noleft_error = (math.fabs((distanceToWall - corner)))/corner # corner
            noleft_error2 = (math.fabs((distanceToWall - simpleWall)))/simpleWall # wall
            left_error = (math.fabs((distanceToWall - left_hallway)))/left_hallway

#  The conditions here must be changed.  These conditions are unreliable.

            if noleft_error < 0.05:
                print "no left passage"
                Left = False
                tank(0,0)
                print "sleeping for 10"
                time.sleep(10)
                return
            if left_error < 0.05:
                print "Found left passage!"
                Left = True
                tank(0,0)
                print "sleeping for 10"
                time.sleep(10)
                return
            if noleft_error2 < 0.05:
                print "Found simple wall passage!"
                Left = False
                tank(0,0)
                print "sleeping for 10"
                time.sleep(10)
                return
            time.sleep(0.01)
        tank(0,0)
        print "we couldn't identify anything in the left of this intersection."
        time.sleep(5)
        return
        
def RightSensing(laserScan):
    '''
    given a filtered laserScan, this function interprets the data 
    and returns whether or not the robot has an opening to its right
    '''
    #time.sleep(2)   # for debugging
    
    signalLength = len(laserScan)
    GoodDataThreshold = 80

    # Good Data?
    if signalLength < GoodDataThreshold:
        print "not enough data"
        return False
    
# Three important shapes of this signal exist.  The levels of the nested loops
# detect this shape

   # Begin Processing the Signal
    average_signal_point = (laserScan[0]
                      + laserScan[1]
                      + laserScan[2])/3
    error = 0.05 * average_signal_point

    # Wrap it all up in a try-except. Index errors means that the laserScan
    # doesn't conform to the desired signal shape
    try:
        # Shape 1
        for index in range(2, signalLength):
            # detect the initial rise in the signal
            if (laserScan[index] + error) > average_signal_point:
                # reset average and error
                average_signal_point = (laserScan[index]
                                    + laserScan[index-1]
                                    + laserScan[index-2]) / 3
                error = 0.05 * average_signal_point # possibly adjust 0.05 later.
            
            else:
                # We need to see an initial rising trend. 25% is the cutoff.
                if index > signalLength / 4:
                    #print "Shape 1 Passes"
                    # we move on to look for the second shape in the signal:
                    # the BIG DROP
                    SmallCliff = laserScan[index] - laserScan[0]

                    SignalDropThreshold = 2.5
                    
                    # Shape 2
                    for index2 in range(index + 1, index + 3):
                        # check the first three values (since the initial rise)
                        # if we see a huge difference in height from
                        # the beginning and peak and the peak and drop,
                        # then we've detected the BIGDROP in the signal
                        BigSignalDrop = laserScan[index] - laserScan[index2]

                        if BigSignalDrop > (SignalDropThreshold * SmallCliff):
                            # We've detected the big drop in the signal!
                            # leave this bigdrop-detecting for-loop!
                            #print "Shape 2 passes"
                            break
                        else:
                            if index2 == index + 2:
                                # if we've checked all values in this for-loop 
                                # and haven't found the big drop, no 
                                # intersection exists!
                                # print "Shape 2 failed"
                                return False
                    # Now, we need to detect the remaining decrease in the signal.
                    average_signal_point = (laserScan[index]
                          + laserScan[index - 1]
                          + laserScan[index - 2]) / 3
                    error = 0.05 * average_signal_point

                    for index3 in range(index + 3, signalLength):
                        if laserScan[index3] < average_signal_point + error:
                            # reset the average and error
                            average_signal_point = (laserScan[index]
                              + laserScan[index - 1]
                              + laserScan[index - 2]) / 3
                            error = 0.05 * average_signal_point
                        
                        else:
                            # our last drop in signal doesn't exist.
                            #print "Shape 3 failed"
                            return False

                    #print "Shape 3 passes"
                    print "WE FOUND AN INTERSECTION !"
                    return True
    
                else:
                    #print "Shape 1 failed"
                    return False
        
    except IndexError:
        #print "We had an index error"
        return False

def FrontandRight(laserScan):
    '''
    given a filtered laserScan, this function interprets the data 
    and returns whether or not a path lies both in front and 
    to the right of the robot.
    '''
    
    signalLength = len(laserScan)
    GoodDataThreshold = 80

    # Good Data?
    if signalLength < GoodDataThreshold:
        print "not enough data to determine if front is open."
        time.sleep(10)
        return False
    
# Four important shapes of this signal exist.  The levels of the nested loops
# detect this shape

   # Begin Processing the Signal
    average_signal_point = (laserScan[0]
                      + laserScan[1]
                      + laserScan[2]) / 3
    error = 0.05 * average_signal_point

    # Wrap it all up in a try-except. Index errors means that the laserScan
    # doesn't conform to the desired signal shape
    try:
        # Shape 1
        for index in range(2,signalLength):
            # detect the initial rise in the signal
            if (laserScan[index] + error) < average_signal_point:
                # reset average and error
                average_signal_point = (laserScan[index]
                                    + laserScan[index-1]
                                    + laserScan[index-2])/3
                error = 0.05*average_signal_point # possibly adjust 0.05 later.
            
            else:
                # We need to see an initial falling trend. 20% is the cutoff.
                if index > signalLength/20:
                    # print "Shape 1 Passes"
                    # we move on to look for the second shape in the signal:
                    # the small rise
                    # but first, we'll remember the value of this point for our
                    # Shape3 detector
                    SmallCliff = laserScan[index] - laserScan[0]
                    SignalDropThreshold = 2.5
                    
                    # Shape 2: rising line
                    average_signal_point = (laserScan[index]
                                    + laserScan[index-1]
                                    + laserScan[index-2])/3
                    error = 0.05*average_signal_point # possibly adjust 0.05 later.
                    
                    for index2 in range(index,signalLength):
                        if (laserScan[index2] + error) > average_signal_point:
                            # reset average and error
                            average_signal_point = (laserScan[index2]
                                    + laserScan[index2-1]
                                    + laserScan[index2-2])/3
                            error = 0.05*average_signal_point
                            # possibly adjust 0.05 later.
                        else:
                            # Now we want to see the big drop
                            if index2 > (2/5)*signalLength:
                                # We move on to find the BIGDROP
                                # Shape 3: the big drop
                                for index3 in range(index2+1,index2+3):
                                    # check the first three values (since the initial rise)
                                    # if we see a huge difference in height from
                                    #   the beginning and peak   and
                                    #   the peak and drop,
                                    # then we've detected the BIGDROP in the signal
                                    BigSignalDrop = laserScan[index2] - laserScan[index3]

                                    if BigSignalDrop > (SignalDropThreshold*SmallCliff):
                                    # We've detected the big drop in the signal!
                                    # leave this bigdrop-detecting for-loop!
                                    # print "BigDrop passes"
                                        break
                                    else:
                                        if index3 == index2+2:
                                        # if we've checked all values in this for-loop and
                                        # haven't found the big drop, no intersection exists!
                                        # print "BigDropfailed"
                                            return False
                                        # Now, we need to detect the remaining decrease
                                        # in the signal.
                                average_signal_point = (laserScan[index3]
                                                  + laserScan[index3 - 1]
                                                  + laserScan[index3 - 2]) / 3
                                error = 0.05 * average_signal_point
                                
                                for index4 in range(index3 + 3, signalLength):
                                    if laserScan[index4] < average_signal_point + error:
                                    # reset the average and error
                                        average_signal_point = (laserScan[index4]
                                          + laserScan[index4-1]
                                          + laserScan[index4-2])/3
                                        error = 0.05 * average_signal_point
                        
                                    else:
                                    # our last drop in signal doesn't exist.
                                    # print "Shape 4 failed"
                                        return False
                                
                                    # print "Shape 4 passes"
                                    # print
                                print "Front and Right are Open !!"
                                return True

                            else:
                            #    print "Shape 2 failed"
                                return False
                else:
                  # print "Shape 1 failed"
                    return False
        
    except IndexError:
     #  print "We had an index error"
        return False    

        
class WallFollowing():
    i = 0
    
    def __init__(self):
        # Give the node a name
        rospy.init_node('wall_following', anonymous=False)
        
        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)
        
        # How fast will we check the odometry values?
        rate = 20
        
        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)
        
        # Set the parameters for wall following
        goal_distance = rospy.get_param("~goal_distance", 1.0)      # meters
        goal_angle = rospy.get_param("~goal_angle", math.radians(-90))    # degrees converted to radians
        linear_speed = rospy.get_param("~linear_speed", 0.5)        # meters per second
        angular_speed = rospy.get_param("~angular_speed", 3.5)      # radians per second
        angular_tolerance = rospy.get_param("~angular_tolerance", math.radians(2)) # degrees to radians
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/mybot/cmd_vel', Twist)
        
        # The base frame is base_footprint for the TurtleBot but base_link for Pi Robot and in mybot case is footprint
        self.base_frame = rospy.get_param('~base_frame', '/footprint')
        
        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        
        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2.0)
        
        # Set the odom frame
        self.odom_frame = '/odom'
        
        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/footprint'
        except(tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/chassis', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/chassis'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /chassis or /footprint")
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
        
        # Keep track of the distance traveled and save it altogether with images, etc.
        distance = 0
        
        while distance < goal_distance and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle         
            self.cmd_vel.publish(move_cmd)
            
            r.sleep()
            
            # Get the current position
            (position, rotation) = self.get_odom()
            
            # Compute the distance to the wall...
            
            
        # Stop the robot before rotating
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(2.0)
            
        # Set the movement command to a rotation
        move_cmd.angular.z = angular_speed
            
        # Track the last angle measured
        last_angle = rotation
            
        # Track how far we have turned
        turn_angle = 0
            
        # Begin the rotation
        
        
        
        
        # Stop the robot when we are done
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
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(2.0)

def OdometryNode():
    ''' 
    OdometryNode creates a node that subscribes to the odometry information 
    and then uses the callback function called HandleOdomData to store
    the angle to a global variable for access in this Python Script 
    '''
    #rospy.init_node('OdometryAccessor')
    rospy.Subscriber('odom', Odometry, HandleOdomData)
    print "OdometryNode activated"

def HandleOdomData(msg):
    # print "Handling Odometry"
    global odom_angle
    Quat_angle = msg.pose.pose.orientation
    quaternion = []
    quaternion.append(Quat_angle.x)
    quaternion.append(Quat_angle.y)
    quaternion.append(Quat_angle.z)
    quaternion.append(Quat_angle.w)
    
    odom_angle = euler_from_quaternion(quaternion)[2]
    # print "odom_angle is" + str(odom_angle) + "from HandleOdomData"

def LaserScanNode():
    ''' 
    LaserScanNode creates a Node that subscribes to the depth information from
    the Kinect and sends it to the callback function HandleDepthData 
    to produce a global FilteredLaserScan 
    ''' 
    global on
    global f
    global wallFollowMode
    global timeToStop
    #cv.NamedWindow('disp', cv.WINDOW_AUTOSIZE)
    rospy.init_node('GlobalMaker')
    rospy.Subscriber('/camera/depth/image_raw',sm.Image,HandleDepthData)

    print "LaserScanNode Activated"
    
    on = True
    wallFollowMode = findWall
    timeToStop =-1
    '''if on:
        print "Waiting for tank"
        rospy.wait_for_service('tank')
        f = rospy.ServiceProxy('tank', Tank)
        f(False, 0, 0) 
        print "Found tank"
    '''

def clear_odometry():
    ''' resets the angle and distance traveled, and stops the robot from moving'''
    rospy.wait_for_service('tank')
    try:
        f = rospy.ServiceProxy('tank', Tank)
        f(True,0,0)
    except rospy.ServiceException, e:
        print "could not reset odometry."
        
def tank(left, right, time=-1):
    """
    Drives the robot at the speed specified for each side.
    
    left and right are between -1 and 1.
    """
    rospy.wait_for_service('tank')
    maxSpeed = 500
    tolerance = 50
    left = clamp(left * maxSpeed, -1*maxSpeed, maxSpeed)
    right = clamp(right * maxSpeed, -1*maxSpeed, maxSpeed)
    left = roundToZero(left, tolerance)
    right = roundToZero(right, tolerance)

    try:
        # This creates a function that uses the tank service.
        # the irobot_create_2_1 services are described here
        # http://www.ros.org/wiki/irobot_create_2_1#Messages_and_Services
        f = rospy.ServiceProxy('tank', Tank)
        f(False, left, right)
        if time != -1:
            rospy.sleep(time)
            f(False, 0, 0)
        return 
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def clamp(val, minimum, maximum):
    if val < minimum:
        return minimum
    elif val > maximum:
        return maximum
    else:
        return val

def roundToZero(val, tolerance):
    if -1 * tolerance < val < tolerance:
        return 0
    else:
        return val
        
if __name__ == '__main__':
    global Front
    global Left
    global Right
    global Back
    global distanceToWall

    Back = True
    Left = False
    Front = False
    Right = False

    LaserScanNode() # calls HandleDepth to constantly update the global 
                    # CurrentLaserScan
    
    OdometryNode()  # creates a node that subscribes to the odometry information 
                    # and stores the total angle to a global variable.
    print
    print "defining variables while we wait"
    time.sleep(5)
    # We need to wait while the two nodes fire up; otherwise, they'll throw back
    # 'distanceToWall is not defined' and other such errors.

    while not rospy.is_shutdown():
        #print distanceToWall
        intersection_prompt()   # Tells us whether or not we're at
                                # intersection  and sets intersection to
                                # True if we are indeed at an intersection.
        if Right == True:
            print "Front Detecting!"
                                                   
        Front_Detector()   # If we're at an intersection, FrontDetector() aligns itself
                           # slightly left to look for an open path directly in front.
                           # If such a path exists, the intersection options are updated.
        if Front == True:
            print "Left Detecting!"
            
        Left_Detector() # If we're at an intersection, this function assumes we're facing
                        # down the hall, and it continues rotating looking left
                        # for a path to the left
      
        print "Front, Back, Left, Right  is ..."
        print (Front,Back,Left,Right)

        if Left == True:
            time.sleep(10)
            
        time.sleep(0.01) # Allow some computation time per cycle.
        
    rospy.spin()
