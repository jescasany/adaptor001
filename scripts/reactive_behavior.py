# -*- coding: utf-8 -*-
"""
Created on Tue Mar  1 12:12:09 2016

@author: Daniel Williams - 13458204
"""

#!/usr/bin/env python

import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from random import randint
import random

class image_converter:

    def __init__(self):

		#SETS AN INITIAL VALUE FOR THE MINIMUM LASER SCAN VALUE SO THAT IF A CALLBACK REQUIRES A VALUE
		#BEFORE ONE IS SET BY THE LASER SCAN CALLBACK THERE WILL BE NOT ERRORS
        self.minimum = 1000   
        
		#INITIALISES THE DISPLAY WINDOWS THAT WILL BE USED TO DISPLAY IMAGES
        cv2.namedWindow("colour window", 1)
        cv2.namedWindow("left image", 1)
        cv2.namedWindow("right image", 1)
        
        cv2.startWindowThread()
        self.bridge = CvBridge()    
        
		#INITIALISES A GLOBALLY ACCESSIBLE PUBLISHER THAT CAN PUBLISH TWIST MESSAGES TO THE cmd_vel TOPIC
        self.geoPub = rospy.Publisher("/turtlebot_1/cmd_vel", Twist)
        
		#INITIALISES A SUBSCRIBER TO THE 'camera/rgb/image_raw' TOPIC THAT WILL RECIEVE AN Image AND CALL THE callback FUNCTION WHEN NEW DATA IS PUBLISHED
        self.image_sub = rospy.Subscriber("/turtlebot_1/camera/rgb/image_raw", Image, self.callback)
        
		#INITIALISES A SUBSCRIBER TO THE 'scan' TOPIC THAT WILL RECIEVE A LaserScan AND CALL THE scanCallback FUNCTION WHEN NEW DATA IS PUBLISHED
        self.laser_sub = rospy.Subscriber("/turtlebot_1/scan", LaserScan, self.scanCallback)
        
		
		#Real Robot use
		#self.geoPub = rospy.Publisher("/cmd_vel", Twist)
		#self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.callback)
		#self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.scanCallback)
        
        
    #METHOD THAT HANDLES ROBOT MOVEMENT WHEN TARGET LIGHTSOURCE IS VISIBLE    
    def targetFound(self, lWhite, rWhite):
        
        #THRESHOLDING THAT SETS THE SPEED OF THE LINEAR MOVEMENT SO THAT THE CLOSER TO THE LIGHTSOURCE THE SLOWER THE LINEAR MOMENTUM,
        if ((lWhite + rWhite) <= 10000): 
            speed = 0.2 
        elif ((lWhite + rWhite) > 10000 and (lWhite + rWhite) <= 20000): 
            speed = 0.4 
        elif ((lWhite + rWhite) > 20000 and (lWhite + rWhite) <= 30000): 
            speed = 0.2
        elif ((lWhite + rWhite) > 30000 and (lWhite + rWhite) <= 36300): 
            speed = 0.1
        elif ((lWhite + rWhite) > 363000):  
            speed = 0.0
            print "Light Source Found"

        #MOVE LEFT
        #CHECKS IF THE LEFT HALF OF THE IMAGE CONTAINS A GREATER NUMBER OF HIGH INTENSITY PIXELS THAT THE RIGHT
        #IF IT DOES PUBLISHES A GEOMETERY TWIST MESSAGE TO TURN THE ROBOT TO THE LEFT WHILST MAINTAINING THE LINEAR MOMENTUM
        if (lWhite < rWhite): 
            message = Twist()
            message.linear.x = speed
            message.angular.z = -0.5
            self.geoPub.publish(message)
            
        #MOVE RIGHT  
        #CHECKS IF THE RIGHT HALF OF THE IMAGE CONTAINS A GREATER NUMBER OF HIGH INTENSITY PIXELS THAT THE LEFT
        #IF IT DOES PUBLISHES A GEOMETERY TWIST MESSAGE TO TURN THE ROBOT TO THE RIGHT WHILST MAINTAINING THE LINEAR MOMENTUM
        if (rWhite < lWhite):
            message = Twist()
            message.linear.x = speed
            message.angular.z = 0.5
            self.geoPub.publish(message)
        
        #MOVE FORWARD
        #CHECKS IF THE DIFFERENCE BETWEEN THE LEFT AND RIGHT HALFS OF THE IMAGE ARE WITHIN A RANGE
        #IF IT IS WITHIN THE RANGE PUBLISHES A GEOMETERY TWIST MESSAGE MAINTAINING THE LINEAR MOMENTUM
        if ((lWhite/rWhite > 0.8) & (lWhite/rWhite < 1.2)):
            message = Twist()
            message.linear.x = speed
            self.geoPub.publish(message)
        
    #METHOD THAT HANDLES ROBOT MOVEMENT WHEN TARGET LIGHTSOURCE IS NOT VISIBLE       
    def Wander(self):
        
        #PUBLISHES A GEOMETERY TWIST MESSAGE TO MAINTAIN A LINEAR MOMENTUM
        message = Twist()
        message.linear.x = 0.3
        self.geoPub.publish(message)             
        
    #METHOD THAT HANDLES ROBOT MOVEMENT WHEN AN OBSTACLE IS DETECTED IN THE VISION OF THE ROBOTS LASER SCANNER    
    def obstacleAvoidance(self):

        #CALCULATES A RANDOM WITHIN THE SPECIFIED RANGES TO USE AS THE ANGULAR Z VALUE 
        ang = randint(1, 9)/10.0        
        #ang = random.uniform(0.1, 3.4)        
        
        #PUBLISHES A GEOMETERY TWIST MESSAGE TO ROTATE THE ROBOT BY THE ANGULAR Z CALCULATED ABOVE
        message = Twist()
        message.linear.x = 0.0
        message.angular.z = ang
        self.geoPub.publish(message)   
   
   #CALLBACK FUNCTION FOR THE LASER SCANNER SUBSCRIBER
   #TAKES IN THE LASER SCAN DATA SO THAT THE DISTANCE TO OBSTICLES CAN BE DETERMINED
    def scanCallback(self, data):

        #TAKES THE FLOAT ARRAY CONTAINING THE SCAN VALUES FROM THE RETURNED MESSAGE DATA AND STORES THEM IN AN ARRAY 
        scan = data.ranges      
        
        #FINDS THE MINIMUM VALUE WITHIN THE RETURNED LASER SCAN AND THE INDEX WITHIN THE SCAN THAT THE VALUE IS LOCATED
        self.minIndex = scan.index(min(scan))        
        #self.minimum = min(scan)  
        self.minimum = numpy.nanmin(scan)

    #MAIN CALLBACK FUNCTION THAT IS EXECUTED WHEN A IMAGE IS RETURNED FROM THE camera/rgb/image_raw TOPIC SUBSCRIBER
    def callback(self, data):
        
        #GETS THE WIDTH MIDPOINT AND HEIGHT OF THE IMAGE RETURNED IN THE MESSAGE
        width = data.width
        midPoint = (width/2)
        
        #STORED THE MIMIMUM VALUE FROM THE LASER SCAN IN A LOCAL VARIABLE
        minVal = self.minimum
        
        try:
		
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            #SPLITS THE INPUT IMAGE INTO 2 SEPRTATE IMAGES                        
            leftImage = cv_image[:, :midPoint, :] #ALL ELEMENTS FROM 0 TO THE MIDPOINT WIDTH OF THE IMAGE
            rightImage = cv_image[:, midPoint:, :] #ALL ELEMENTS FROM THE MIDPOINT TO THE END OF THE IMAGE
            
            #CONVERT IMAGE TO GRAY SCALE FOR SIMPLE INTENSITY DETECTION
            left_hsv_image = cv2.cvtColor(leftImage, cv2.COLOR_BGR2GRAY)
            right_hsv_image = cv2.cvtColor(rightImage, cv2.COLOR_BGR2GRAY)
            
		#SUMS THE NUMBER OF HIGHT INTENSITY PIXELS THAT ARE GREATER THAN THE GIVEN THRESHOLD '200'
            leftWhite = (left_hsv_image > 200).sum()
            rightWhite = (right_hsv_image > 200).sum()
            
			#CALCULATES THE TOTAL NUMBER OF HIGH INTENSITY PIXELS IN THE WHOLE IMAGE
            Value = int(int(leftWhite) + int(rightWhite))
            
            #print minVal
 
			#CHECKS FOR OBSTACLES WITHIN CLOSE PROIMITY OF THE ROBOT USING THE MINIMUM VALUE RETURNED FROM THE LASER SCAN
            if (minVal > 1.0):
                
                #print Value
                
                #TARGET LIGHTSOURCE FOUND
				#CHECKS THAT THERE ARE 1000 OR MORE HIGH INTENSITY PIXELS WITHIN THE IMAGE AS A WHOLE
                if (Value >= 1000 ): 
					#CALLS targetFound FUNCTION PASSING THE HIGH INTENSITY VALUES OF THE LEFT AND RIGHT IMAGES
                    self.targetFound(leftWhite, rightWhite)
                    print "Light Source Targeted"
                
                #WANDER
				#CHECKS IF THERE ARE LESS THAN 1000 HIGH INTENSITY PIXELS WITHIN THE IMAGE AS A WHOLE
                if (Value < 1000):  #VALUE FOR SIMULATION USE
                    #CALLS Wander FUNCTION
                    self.Wander()
                    print "Wandering"
                
            #OBSTACLE DETECTED    
			#CHECKS IF THE MIMIMUM VALUE RETURNED BY THE LASER SCAN SHOWS AN OBSTACLE IN CLOSE PROXIMITY 
            elif (minVal <= 1.0 or str(minVal) == "nan"):
                #CALLS obstacleAvoidence FUNCTION
                self.obstacleAvoidance()
                print "object detected"
				
        #ERROR HANDLING    
        except CvBridgeError, e:
            print e

        #OUTPUT IMAGES FOR VISUALISATION OF ROBOTS VIEW
        cv2.imshow("colour window", cv_image)
        cv2.imshow("left image", left_hsv_image)
        cv2.imshow("right image", right_hsv_image)   
    
    
rospy.init_node('image_converter', anonymous=True)
image_converter()
rospy.spin()
cv2.destroyAllWindows()