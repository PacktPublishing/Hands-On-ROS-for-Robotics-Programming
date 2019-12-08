# Integrating GoPiGo3 in Turtlebot3 RL training scenarios

Stage worlds:
 - `$ roslaunch gopigo3_gazebo gopigo3_stage_1.launch`
 - `$ roslaunch gopigo3_gazebo gopigo3_stage_2.launch`

## Launch stage 1 OR stage 2
Select only one launch of 1 or 2 for both terminal T1 & T2
```
T1 $ roslaunch gopigo3_gazebo gopigo3_stage_1.launch
   $ roslaunch gopigo3_gazebo gopigo3_stage_2.launch

T2 $ conda activate tensorflow
(tensorflow) $ roslaunch gopigo3_dqn gopigo3_dqn_stage_1.launch
(tensorflow) $ roslaunch gopigo3_dqn gopigo3_dqn_stage_2.launch
```