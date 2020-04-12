# Access Python env object
## Get this output by debugging `cartpole_dqn_debug.py`
```
$ conda activate gym
$ python
```
After entering the REPL, you can execute `cartpole_dqn_debug.py` line by line, or in blocks. Once `env` object is created, you can access it with `pprint` and `vars`:
> `>>> pprint(vars(env))`
```
{'_elapsed_steps': 0,
 '_max_episode_steps': 1000,
 'action_space': Discrete(2),
 'env': <openai_ros.task_envs.cartpole_stay_up.stay_up.CartPoleStayUpEnv object at 0x7fd26c69e210>,
 'metadata': {'render.modes': []},
 'observation_space': Box(4,),
 'reward_range': (-inf, inf)}
```

> `>>> pprint(vars(env.env))`
```
{'_base_pub': <rospy.topics.Publisher object at 0x7fd26c63db10>,
 '_pole_pub': <rospy.topics.Publisher object at 0x7fd26c63db90>,
 'action_space': Discrete(2),
 'control_type': 'velocity',
 'controllers_list': ['joint_state_controller',
                      'pole_joint_velocity_controller',
                      'foot_joint_velocity_controller'],
 'controllers_object': <openai_ros.controllers_connection.ControllersConnection instance at 0x7fd26c654eb0>,
 'cumulated_episode_reward': 0,
 'episode_num': 0,
 'gazebo': <openai_ros.gazebo_connection.GazeboConnection instance at 0x7fd26c654cd0>,
 'init_pos': 0.0,
 'max_base_pose_x': 1.0,
 'max_base_velocity': 50,
 'max_pole_angle': 0.7,
 'min_base_pose_x': -1.0,
 'min_pole_angle': -0.7,
 'n_actions': 2,
 'np_random': <mtrand.RandomState object at 0x7fd26c654f50>,
 'observation_space': Box(4,),
 'pos_step': 1.0,
 'publishers_array': [<rospy.topics.Publisher object at 0x7fd26c63db10>,
                      <rospy.topics.Publisher object at 0x7fd26c63db90>],
 'reset_controls': True,
 'reward_pub': <rospy.topics.Publisher object at 0x7fd26c651c10>,
 'robot_name_space': 'cartpole_v0',
 'running_step': 0.04,
 'spec': EnvSpec(CartPoleStayUp-v0),
 'steps_beyond_done': None,
 'wait_time': 0.1}
 ```