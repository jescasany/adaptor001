#!/usr/bin/env python  
"""
ROS Node which displays lines extracted from laser range scans.  The node
subscribes to 'lines_topic' and publishes the data needed to display these
lines to 'vis_lines_topic' and correspondingly coloured first and last scan points used to generate each line to 'vis_scanpoints_topic'.  RViz must be
configured appropriately to display the messages posted to these topics.
"""

import pdb

import rospy
from math import cos, sin, pi

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from adaptor001.msg import ExtractedLine
from adaptor001.msg import ExtractedLines

from black_board_class import BlackBoard, black_board_object

def create_lines_marker(lines):
    marker = Marker()
    #pdb.set_trace()
    marker_lifetime = 0 # 0 is forever
    # Get the pairs of points that comprise the endpoints for all lines
    marker.points = generate_endpoints(lines)
#    print 'Marker points: '
#    print marker.points

    # Initialize basic fields of the marker
    marker.header.frame_id = lines.header.frame_id
    marker.header.stamp = rospy.Time()
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    #marker.lifetime = rospy.Duration(marker_lifetime)

    # The marker's pose is at (0,0,0) with no rotation.
    marker.pose.orientation.w = 1

    # Set line width
    marker.scale.x = 0.02

    n = len(lines.lines)
    #print "Numero de lineas(lines)", n
    if n == 1:
        # There is only one line.  Just set the color field.
        marker.color.g = 1.0
        marker.color.a = 1.0
    if n > 1:
        # Set per-vertex colours
        for i in range(n):
            color = ColorRGBA()
            # A very simplistic colour spectrum.
            color.r = 1.0 - i/float(n-1)
            color.g = 1.0
            color.b = i/float(n-1)
            color.a = 1.0
            marker.colors.append(color)
            marker.colors.append(color)
            
    #print "Marker", marker

    black_board_object.lines_publisher.publish(marker)
    rospy.sleep(5)
    #raw_input("Press ENTER to continue...")

def create_scanpoints_marker(lines):
    marker = Marker()
    #pdb.set_trace()
    marker_lifetime = 0 # 0 is forever
    # Initialize basic fields of the marker
    marker.header.frame_id = lines.header.frame_id
    marker.header.stamp = rospy.Time()
    marker.id = 0
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    #marker.lifetime = rospy.Duration(marker_lifetime)
    
    # The marker's pose is at (0,0,0) with no rotation.
    marker.pose.orientation.w = 1

    # Set point width
    marker.scale.x = 0.08
    marker.scale.y = 0.08

    # Add the scan points
    marker.points = []
    n = len(lines.lines)
    #print "Numero de linea(points)s", n
    for i in range(n):
        marker.points.append(lines.lines[i].firstScanPoint)
        marker.points.append(lines.lines[i].lastScanPoint)

    if n == 1:
        # There is only one line.  Just set the color field.
        marker.color.g = 1.0
        marker.color.a = 1.0
    if n > 1:
        # Set per-vertex colours
        for i in range(n):
            color = ColorRGBA()
            # A very simplistic colour spectrum.
            color.r = 1.0 - i/float(n-1)
            color.g = 1.0
            color.b = i/float(n-1)
            color.a = 1.0
            marker.colors.append(color)
            marker.colors.append(color)
    
    #print "Marker", marker
    
    black_board_object.scanpoints_publisher.publish(marker)
    rospy.sleep(5)
    #raw_input("Press ENTER to continue...")

def generate_endpoints(lines):
    """
    Generate two endpoints for each given line.

    Returns a list of point pairs to represent each of the given lines.  
    The points are positioned at a fixed distance 'q' from the point on the
    line closest to the origin (where the orthogonal ray hits the line).
    """
    # Distance from the closest point on the line to the origin at which to
    # place the endpoints.
    q = 5

    # The list of points
    points = []

    n = len(lines.lines)
    #print "Numero de lineas(gen_markers)", n
    for i in range(n):
        r = lines.lines[i].r
        alpha = lines.lines[i].alpha - pi/2

        # Each point is a vector sum from the origin to the point on the line
        # closest to the origin + a vector along the line in either direction.
        point1 = Point(r*cos(alpha) + q*cos(alpha+pi/2), \
                       r*sin(alpha) + q*sin(alpha+pi/2), \
                       0)
        point2 = Point(r*cos(alpha) + q*cos(alpha-pi/2), \
                       r*sin(alpha) + q*sin(alpha-pi/2), \
                       0)
        points.append(point1)
        points.append(point2)

    return points
