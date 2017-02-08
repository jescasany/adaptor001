#!/usr/bin/env python

import roslib
import rospy

import sensor_msgs.msg

class Mybot:
    
    def scan_callback(self, scan):
        scan_pub = rospy.Publisher('mybot/laser/scan', sensor_msgs.msg.LaserScan)
        ranges_size = len(scan.ranges)
        
        # populate the LaserScan message
        mybot = sensor_msgs.msg.LaserScan()
        mybot.header.stamp = scan.header.stamp
        mybot.header.frame_id = scan.header.frame_id
        mybot.angle_min = scan.angle_min
        mybot.angle_max = scan.angle_max
        mybot.angle_increment = scan.angle_increment
        mybot.time_increment = scan.time_increment
        mybot.range_min = 0.0
        mybot.range_max = 100.0
        mybot.ranges = list() 
        
        for i in range(ranges_size):
            mybot.ranges.append(scan.ranges[i] + 1)
        
        scan_pub.publish(mybot) 
        return
           
    def reader(self):
        scan_sub = rospy.Subscriber('/mybot/laser/scan', sensor_msgs.msg.LaserScan, self.scan_callback)
        return

if __name__ == '__main__':
    try:
        rospy.init_node('Mybot', anonymous = False)
        mybot = Mybot()
        mybot.reader()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Mybot finished.")

    