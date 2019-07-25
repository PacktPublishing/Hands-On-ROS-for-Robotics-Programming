import time
from di_sensors.inertial_measurement_unit import InertialMeasurementUnit

print("Example program for reading di-IMU Sensor on a GoPiGo3 AD2 port")

imu = InertialMeasurementUnit(bus = "GPG3_AD2")

while True:
    # Read the magnetometer, gyroscope, accelerometer, euler, and temperature values
    mag   = imu.read_magnetometer()
    gyro  = imu.read_gyroscope()
    accel = imu.read_accelerometer()
    euler = imu.read_euler()
    temp  = imu.read_temperature()

    string_to_print = "Magnetometer X: {:.1f}  Y: {:.1f}  Z: {:.1f} " \
                      "Gyroscope X: {:.1f}  Y: {:.1f}  Z: {:.1f} " \
                      "Accelerometer X: {:.1f}  Y: {:.1f} Z: {:.1f} " \
                      "Euler Heading: {:.1f}  Roll: {:.1f}  Pitch: {:.1f} " \
                      "Temperature: {:.1f}C".format(mag[0], mag[1], mag[2],
                                                    gyro[0], gyro[1], gyro[2],
                                                    accel[0], accel[1], accel[2],
                                                    euler[0], euler[1], euler[2],
                                                    temp)
    print(string_to_print)

    time.sleep(1)