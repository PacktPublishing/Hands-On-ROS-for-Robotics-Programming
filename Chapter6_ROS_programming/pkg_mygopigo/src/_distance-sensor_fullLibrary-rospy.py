#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from di_sensors.distance_sensor import DistanceSensor
from sensor_msgs.msg import Range


def main():
    sensor = DistanceSensor()
    sensor.start_continuous()

    rospy.init_node("distance_sensor")
    pub_distance = rospy.Publisher("~distance", Range, queue_size=10)

    msg_range = Range()
    msg_range.header.frame_id = "distance"
    msg_range.radiation_type = Range.INFRARED
    msg_range.min_range = 0.005
    msg_range.max_range = 8.0

    rate = rospy.Rate(rospy.get_param('~hz', 30))
    while not rospy.is_shutdown():
        # read distance in meter
        msg_range.range = sensor.read_range_continuous() / 1000.0
        msg_range.header.stamp = rospy.Time.now()

        pub_distance.publish(msg_range)

        rate.sleep()


if __name__ == '__main__':
    main()