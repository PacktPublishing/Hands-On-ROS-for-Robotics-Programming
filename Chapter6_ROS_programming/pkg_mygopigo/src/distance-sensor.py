#!/usr/bin/env python

# Import DI sensors module
from di_sensors.easy_distance_sensor import EasyDistanceSensor
# ROS specific modules
import rospy
from sensor_msgs.msg import Range

# Main loop

def main():
	# instantiate the distance object
    #sensor = DistanceSensor()
    my_sensor = EasyDistanceSensor()
    #sensor.start_continuous()

    rospy.init_node("distance_sensor")
    
    pub_distance = rospy.Publisher("~distance", Range, queue_size=10)

    msg_range = Range()
    msg_range.header.frame_id = "distance"
    msg_range.radiation_type = Range.INFRARED
    msg_range.min_range = 0.02
    msg_range.max_range = 3.0

    rate = rospy.Rate(rospy.get_param('~hz', 1))
    while not rospy.is_shutdown():
        # read distance in meters
        read_distance = my_sensor.read()/100.0
        msg_range.range = read_distance
        # Add timestamp
        msg_range.header.stamp = rospy.Time.now()

        # Print current distance
        print msg_range.range*1000," mm"
        # Publish distance message
        pub_distance.publish(msg_range)

        rate.sleep()

if __name__ == '__main__':
    main()
