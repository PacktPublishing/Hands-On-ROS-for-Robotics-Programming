This repository is the evolution of [gopigo3_node](https://github.com/ros-gopigo/gopigo3_node) for using easy libraries of GoPiGo3 and DI sensors within a ROS enviroment (from Python scripts)

# gopigo3 node (gopigo3_driver.py)
```bash
T1 $ roslaunch mygopigo gopigo3.launch
T2 $ rostopic list
```

# Distance sensor
Run nodes manually:
```bash
T1 $ roscore
T2 $ rosrun mygopigo easyDistance.py
T3 $ rostopic echo /distance_sensor/distance
```

```bash
T1 $ roslaunch mygopigo gopigo3.launch
T2 $ rostopic list
T3 $ rostopic echo /distance_sensor/distance
```
Topics are published in namespace `<name of node>/<~topic>` ('~' is equivalente to `<name of node>/`). For the case of this script `topic= distance` and `name of node= distance_sensor`:
```
    rospy.init_node("distance_sensor")
    pub_distance = rospy.Publisher("~distance", Range, queue_size=10)
```
## Using roslaunch
```bash
T1 $ roslaunch mygopigo easyDistance.launch
T2 $ rostopic echo /easyDistance_sensor/distance
```
`easyDistance_sensor` new name of the node is given in launch file and supersedes that of `easyDistance.py`

