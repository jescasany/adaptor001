#!/usr/bin/env python

""" 
    scan_reading.py - Version 1.0 2017-02-01
    
    Juan Escasany 
    
    To read and analyze scans     
"""
import rospy
import sensor_msgs.msg

class LaserScanReader:
    def __init__(self):
        self.x = list()
        self.y = list()
        
    def get_ranges(self):
        return self.y
              
    def scan_callback(self, msg):
        self.y = msg.ranges
        self.x = range(len(msg.ranges))
        if sum(self.y[0:200])/200 < 3.0:
            print "pared a la derecha"
        elif sum(self.y[0:300])/300 == 5.0:
            print "nada delante"
        else:
            print "puerta a la derecha"
        
    def reader(self):
        
        rospy.init_node('reader', anonymous=False)
        
        rospy.loginfo("Waiting for /scan topic...")
        rospy.wait_for_message('/scan', sensor_msgs.msg.LaserScan)
        
        # Subscribe the /scan topic to get the range readings  
        rospy.Subscriber('/scan', sensor_msgs.msg.LaserScan, self.scan_callback, queue_size = 10)
            
if __name__ == '__main__':
    l = LaserScanReader()
    l.reader()     # calls reader to constantly update the /scan
    print l.get_ranges()
    rospy.spin()