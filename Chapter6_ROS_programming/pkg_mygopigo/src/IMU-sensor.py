#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from di_sensors.inertial_measurement_unit import InertialMeasurementUnit
from sensor_msgs.msg import Imu, Temperature, MagneticField
from std_msgs.msg import Header
from tf.broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math


def main():
    rospy.init_node("imu")

    port = rospy.get_param("~port", "GPG3_AD1")
    sensor = InertialMeasurementUnit(bus=port)

    pub_imu = rospy.Publisher("~imu", Imu, queue_size=10)
    pub_temp = rospy.Publisher("~temp", Temperature, queue_size=10)
    pub_magn = rospy.Publisher("~magnetometer", MagneticField, queue_size=10)

    br = TransformBroadcaster()

    msg_imu = Imu()
    msg_temp = Temperature()
    msg_magn = MagneticField()
    hdr = Header(stamp=rospy.Time.now(), frame_id="IMU")

    rate = rospy.Rate(rospy.get_param('~hz', 30))
    while not rospy.is_shutdown():
        q = sensor.read_quaternion()        # x,y,z,w
        mag = sensor.read_magnetometer()    # micro Tesla (µT)
        gyro = sensor.read_gyroscope()      # deg/second
        accel = sensor.read_accelerometer() # m/s²
        temp = sensor.read_temperature()    # °C

        msg_imu.header = hdr

        hdr.stamp = rospy.Time.now()

        msg_temp.header = hdr
        msg_temp.temperature = temp
        pub_temp.publish(msg_temp)

        msg_imu.header = hdr
        msg_imu.linear_acceleration.x = accel[0]
        msg_imu.linear_acceleration.y = accel[1]
        msg_imu.linear_acceleration.z = accel[2]
        msg_imu.angular_velocity.x = math.radians(gyro[0])
        msg_imu.angular_velocity.y = math.radians(gyro[1])
        msg_imu.angular_velocity.z = math.radians(gyro[2])
        msg_imu.orientation.w = q[3]
        msg_imu.orientation.x = q[0]
        msg_imu.orientation.y = q[1]
        msg_imu.orientation.z = q[2]
        pub_imu.publish(msg_imu)

        msg_magn.header = hdr
        msg_magn.magnetic_field.x = mag[0]*1e-6
        msg_magn.magnetic_field.y = mag[1]*1e-6
        msg_magn.magnetic_field.z = mag[2]*1e-6
        pub_magn.publish(msg_magn)

        transform = TransformStamped(header=Header(stamp=rospy.Time.now(), frame_id="world"), child_frame_id="IMU")
        transform.transform.rotation = msg_imu.orientation
        br.sendTransformMessage(transform)

        rate.sleep()


if __name__ == '__main__':
    main()