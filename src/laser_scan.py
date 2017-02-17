#!/usr/bin/env python

""" 
    laser_scan.py - Version 1.0 2017-02-01
    
    Juan Escasany 
    
    To read and analyze scans     
"""
import rospy
import sensor_msgs.msg
        
def laser_scan():
        
#    rospy.init_node('reader', anonymous=False)
    
    rospy.loginfo("Waiting for /scan topic...")
    rospy.wait_for_message('/base_scan', sensor_msgs.msg.LaserScan)
    
    # Subscribe the /scan topic to get the range readings  
    result = rospy.Subscriber('/base_scan', sensor_msgs.msg.LaserScan, scan_callback, queue_size = 10)
    rospy.loginfo("laser_scan done")
    
    return result

def scan_callback(msg):
    y = list()
    y = msg.ranges
    if sum(y[0:200])/200 < 3.0 and sum(y[200:300])/100 == 5.0:
        return "right wall"
    elif sum(y[200:300])/100 < 4.0:
        return "obstacle"
    else:
        return "right door"