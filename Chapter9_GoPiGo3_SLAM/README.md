# 1. [Robot] Visualize the laser point cloud in RViz
- **Option 1 [LDS]** Launch LDS node:
`> r1 $ roslaunch ydlidar lidar.launch`

- **Option 2 [LDS RViz]** Launch LDS node & RViz (without URDF model):
`> r1 $ roslaunch ydlidar lidar_view.launch`

- **Option 3 [LDS+RViz]** Launch LDS node & RViz (with URDF model):
`> r1 $ roslaunch ydlidar display.launch`


# 2. [Robot + PC] Visualize the laser point cloud in RViz
```
r1 $ roslaunch mygopigo gopigo3.launch
r2 $ roslaunch ydlidar ydlidar.launch
```
OR with a single launch file:
>`$ r2 $ roslaunch ydlidar gopigo3_ydlidar.launch`

The run the RViz visualization in the laptop PC:
>`$ T1 $ roslaunch ydlidar display_scan.launch`