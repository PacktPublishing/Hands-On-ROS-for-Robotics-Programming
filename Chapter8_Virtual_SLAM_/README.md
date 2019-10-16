# Basic GoPiGo3
The basic version does not include the Laser Distance Sensor.

- Visualization: `roslaunch gopigo3_description gopigo3_basic_rviz.launch`
- Gazebo: `roslaunch virtual_slam gopigo3_basic_world.launch`
    - Remote control: `rosrun key_teleop key_teleop.py /key_vel:=/cmd_vel`

# GoPiGo3 with Laser Distance Sensor (LSD)
This model `gopigo3` includes the distance_sensor, the same than the infrared_sensor of `rover`.

- Visualization: `roslaunch gopigo3_description gopigo3_rviz.launch`
- Gazebo: `roslaunch virtual_slam gopigo3_world.launch`
    - Remote control: `rosrun key_teleop key_teleop.py /key_vel:=/cmd_vel`

If you want to move using control topic:
```
$ rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

## Add the 2D camera
Camera model and sensor as per `https://brjapon@bitbucket.org/gopigo/chapter8_virtual_slam_navigation.git`

```
T1 $ roslaunch virtual_slam gopigo_world.0.launch 
T2 $ rosrun key_teleop key_teleop.py /key_vel:=/cmd_vel
T3 $ rostopic echo /sensor/ir_front
T4 $ rosrun image_view image_view image:=/gopigo/camera1/image_raw
```

# SLAM with GoPiGo3 whole model
The whole model includes the Laser Distance Sensor [EAI YDLIDAR X4]( https://www.aliexpress.com/item/32908156152.html?spm=a2g0s.9042311.0.0.65d84c4d4mEiDw)

## Test the model: wander around
- Launch the Gazebo world: `T1 $ roslaunch virtual_slam gopigo3_world.launch`
- Launch python algorithm: `T2 $ rosrun virtual_slam wanderAround.py`

ROS dependencies:
`$ sudo apt install ros-melodic-map-server `

## Create a map

- Launch the Gazebo world: `T1 $ roslaunch virtual_slam gopigo3_world.launch`
- Start map building:      `T2 $ roslaunch virtual_slam gmapping_demo.launch`
- Launch rviz /set param.: `T3 $ roslaunch virtual_slam gopigo3_rviz_gmapping.launch`
- Start teleop:            `T4 $ rosrun key_teleop key_teleop.py /key_vel:=/cmd_vel`
   - OR let GoPiGo3 to map the enviroment by itself: `$ rosrun virtual_slam wanderAround.py`

Once finished the exploration, save the map to a file:
`T5 $ rosrun map_server map_saver -f ~/catkin_ws/test_map`

## Navigate to a Goal destination
ROS dependencies:
`$ sudo apt install ros-melodic-move-base ros-melodic-amcl`

- Launch the Gazebo world: `T1 $ roslaunch virtual_slam gopigo3_world.launch`
- Start AMCL algorithm:    `T2 $ roslaunch virtual_slam amcl_demo.launch map_file:=/home/ubuntu/catkin_ws/test_map.yaml`
- Launch rviz:             `T3 $ roslaunch virtual_slam gopigo3_rviz_amcl.launch`