#!/usr/bin/env python

# BEGIN ALL
import rospy
from sensor_msgs.msg import Range

# BEGIN CALLBACK
def callback(msg):
    print "---------------------------"
    print msg
    print "---------------------------"
    rospy.loginfo(rospy.get_caller_id() + ' GoPiGo3 measures distance %.1f mm', msg.range*1000)
# END CALLBACK


rospy.init_node('distance_subscriber')

# BEGIN SUBSCRIBER
sub = rospy.Subscriber('distance_sensor/distance', Range, callback)
# END SUBSCRIBER

rospy.spin()
# END ALL
