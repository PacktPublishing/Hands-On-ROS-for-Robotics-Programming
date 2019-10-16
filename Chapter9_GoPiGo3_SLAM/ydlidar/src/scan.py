#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    print "\nNumber of points =", len(msg.ranges)
    print "------------------"
    print "Range (m) at    0 deg = ", round(msg.ranges[360] , 1)
    print "Range (m) at   90 deg = ", round(msg.ranges[540] , 1)
    print "Range (m) at  180 deg = ", round(msg.ranges[719] , 1)
    print "Range (m) at  -90 deg = ", round(msg.ranges[180] , 1), " or 270 deg"
    #print "Range (m) at -180 deg = ", round(msg.ranges[0] , 1), " or 360 deg"

rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()