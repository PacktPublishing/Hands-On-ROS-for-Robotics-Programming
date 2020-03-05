#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy 
import sys
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Range

speed_factor= 0.3 # Reduce wander speeds by this factor

FORWARD_SPEED = 3.8 * speed_factor
SPIRAL_SPEED = 2.5 * speed_factor
BACKWARD_SPEED = -0.8 * speed_factor
ROTATION_SPEED = 1.5 * speed_factor
laser_BACKWARD_SPEED_multiplier = 1
laser_ROTATION_SPEED_multiplier = 2

class MoveForward():
    def __init__(self):
        self.distanceLaser=1.6 # =1.6 meters (<1.13m, the matching distance to distanceRange=0.8m)
        self.distanceRange=2 # =2 meters
        self.msg=Twist()    
        self.msg=Twist()
        
        rospy.init_node("move_forward",anonymous=False)
        rospy.on_shutdown(self.shutdown)
        
        self.cmd_vel = rospy.Publisher("/cmd_vel",Twist)
        rospy.Subscriber("/gopigo/scan",LaserScan,self.laser_callback)
        rospy.Subscriber("/gopigo/distance",Range,self.range_callback)
        
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.cmd_vel.publish(self.msg)
            r.sleep()
    
    def range_callback(self, scan):
        closest = scan.range
        rospy.loginfo("FRONT distance %s m", closest)
        if closest>self.distanceRange:
            self.msg.linear.x = FORWARD_SPEED # FORWARD_SPEED m/s
            self.msg.angular.z = SPIRAL_SPEED
        elif closest<self.distanceRange:
            self.msg.linear.x = BACKWARD_SPEED  # BACKWARD_SPEED m/s
            self.msg.angular.z = -ROTATION_SPEED # ROTATION_SPEED rad/s
            rospy.loginfo("Within DISTANCE threshold -> ROTATE the robot")
                 
    
    def laser_callback(self, scan):
        closest = min(scan.ranges)
        furthest = max(scan.ranges)
        rospy.loginfo("LASER distance %s  --  %s", closest, furthest)
        if closest<self.distanceLaser:
            self.msg.linear.x = laser_BACKWARD_SPEED_multiplier *BACKWARD_SPEED
            self.msg.angular.z = laser_ROTATION_SPEED_multiplier *ROTATION_SPEED
            rospy.loginfo("Within LASER threshold --> ROTATE FASTER the robot")
               
    def shutdown(self):
        self.msg.linear.x= 0
        self.msg.angular.z = 0
        self.cmd_vel.publish(self.msg)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        MoveForward()
        rospy.spin()
    except KeyboardInterrupt:
        print "Ending MoveForward"