First load the robot within Gazebo:
`T1 $ roslaunch cartpole_description main.launch`

Then proceed to the training:
```
T2 $ conda activate gym
```
`T2 $ (gym) roslaunch cartpole_dqn start_training.launch`