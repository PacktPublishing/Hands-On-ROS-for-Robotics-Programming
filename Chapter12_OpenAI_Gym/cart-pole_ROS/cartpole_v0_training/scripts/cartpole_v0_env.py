import gym
import rospy
import time
import numpy as np
import math
import copy
from gym import utils, spaces
import numpy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock

from gazebo_connection import GazeboConnection
from controllers_connection import ControllersConnection

from gym.utils import seeding
from gym.envs.registration import register

reg = register(
    id='MyCartPole-v0',
    entry_point='cartpole_v0_env:MyCartpoleEnv',
    timestep_limit=1000,
    )

class MyCartpoleEnv(gym.Env):

    def __init__(self):
        self.publishers_array = []
        self._base_pub = rospy.Publisher('/cartpole_v0/foot_joint_position_controller/command', Float64, queue_size=1)
        self._pole_pub = rospy.Publisher('/cartpole_v0/pole_joint_position_controller/command', Float64, queue_size=1)
        self.publishers_array.append(self._base_pub)
        self.publishers_array.append(self._pole_pub)

        self.action_space = spaces.Discrete(2) #l,r,L,R,nothing
        self._seed()
        
        #get configuration parameters
        self.min_pole_angle = rospy.get_param('/cartpole_v0/min_pole_angle')
        self.max_pole_angle = rospy.get_param('/cartpole_v0/max_pole_angle')
        self.max_base_velocity = rospy.get_param('/cartpole_v0/max_base_velocity')
        self.min_base_position = rospy.get_param('/cartpole_v0/min_base_pose_x')
        self.max_base_position = rospy.get_param('/cartpole_v0/max_base_pose_x')
        self.pos_step = rospy.get_param('/cartpole_v0/pos_step')
        self.running_step = rospy.get_param('/cartpole_v0/running_step')
        self.init_pos = rospy.get_param('/cartpole_v0/init_pos')
        self.wait_time = rospy.get_param('/cartpole_v0/wait_time')

        self.init_internal_vars(self.init_pos)

        rospy.Subscriber("/cartpole_v0/joint_states", JointState, self.joints_callback)

        # stablishes connection with simulator
        self.gazebo = GazeboConnection()
        self.controllers_object = ControllersConnection(namespace="cartpole_v0")

    def init_internal_vars(self, init_pos_value):
        self.pos = [init_pos_value]
        self.joints = None

    #always returns the current state of the joints
    def joints_callback(self, data):
        self.joints = data

    def get_clock_time(self):
        self.clock_time = None
        while self.clock_time is None and not rospy.is_shutdown():
            try:
                self.clock_time = rospy.wait_for_message("/clock", Clock, timeout=1.0)
                rospy.logdebug("Current clock_time READY=>" + str(self.clock_time))
            except:
                rospy.logdebug("Current clock_time not ready yet, retrying for getting Current clock_time")
        return self.clock_time
        
    def _seed(self, seed=None): #overriden function
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _step(self, action):#overriden function

        # Take action
        if action == 0: #LEFT
            rospy.logwarn("GO LEFT...")
            self.pos[0] -= self.pos_step
        elif action == 1: #RIGHT
            rospy.logwarn("GO RIGHT...")
            self.pos[0] += self.pos_step
        """
        elif action == 2: #Nothing
            rospy.logwarn("DO NOTHING...")

        elif action == 3: #LEFT BIG
            rospy.logwarn("GO LEFT BIG...")
            self.pos[0] -= self.pos_step * 2
        elif action == 4: #RIGHT BIG
            rospy.logwarn("GO RIGHT BIG...")
            self.pos[0] += self.pos_step * 2
        """

        rospy.logwarn("MOVING TO POS=="+str(self.pos))

        # 1st: unpause simulation
        rospy.logdebug("Unpause SIM...")
        self.gazebo.unpauseSim()

        self.move_joints(self.pos)
        rospy.logdebug("Wait for some time to execute movement, time="+str(self.running_step))
        rospy.sleep(self.running_step) #wait for some time
        rospy.logdebug("DONE Wait for some time to execute movement, time=" + str(self.running_step))

        # 3rd: pause simulation
        rospy.logdebug("Pause SIM...")
        self.gazebo.pauseSim()

        # 4th: get the observation
        observation, done, state = self.observation_checks()

        # 5th: get the reward
        if not done:
            step_reward = 0
            obs_reward = self.get_reward_for_observations(state)
            rospy.loginfo("Reward Values: Time="+str(step_reward)+",Obs="+str(obs_reward))
            reward = step_reward + int(obs_reward)
            rospy.loginfo("TOT Reward=" + str(reward))
        else:
            reward = -2000000

        now = rospy.get_rostime()

        # TODO: Seems that gym version 0.8 doesnt support this very well.
        return observation, reward, done, {}

    def _reset(self):



        rospy.logdebug("We UNPause the simulation to start having topic data")
        self.gazebo.unpauseSim()

        rospy.logdebug("CLOCK BEFORE RESET")
        self.get_clock_time()

        rospy.logdebug("SETTING INITIAL POSE TO AVOID")
        self.set_init_pose()
        time.sleep(self.wait_time * 2.0)
        rospy.logdebug("AFTER INITPOSE CHECKING SENSOR DATA")
        self.check_all_systems_ready()
        #rospy.logdebug("We deactivate gravity to check any reasidual effect of reseting the simulation")
        #self.gazebo.change_gravity(0.0, 0.0, 0.0)

        rospy.logdebug("RESETING SIMULATION")
        self.gazebo.pauseSim()
        self.gazebo.resetSim()
        self.gazebo.unpauseSim()
        rospy.logdebug("CLOCK AFTER RESET")
        self.get_clock_time()

        rospy.logdebug("RESETING CONTROLLERS SO THAT IT DOESNT WAIT FOR THE CLOCK")
        self.controllers_object.reset_cartpole_joint_controllers()
        rospy.logdebug("AFTER RESET CHECKING SENSOR DATA")
        self.check_all_systems_ready()
        rospy.logdebug("CLOCK AFTER SENSORS WORKING AGAIN")
        self.get_clock_time()
        #rospy.logdebug("We reactivating gravity...")
        #self.gazebo.change_gravity(0.0, 0.0, -9.81)
        rospy.logdebug("END")

        # 7th: pauses simulation
        rospy.logdebug("Pause SIM...")
        self.gazebo.pauseSim()

        # get the last observation got when paused, generated by the callbakc or the check_all_systems_ready
        # Depends on who was last
        observation, _, state = self.observation_checks()

        return observation
        
        
    '''
    UTILITY CODE FOLLOWS HERE
    '''
    
    def observation_checks(self):
        done = False
        data = self.joints
        #       base_postion            base_velocity           pole angle              pole velocity
        state = [round(data.position[1],1), round(data.velocity[1],1), round(data.position[0],3), round(data.velocity[0],3)]
        #   pole angle  pole velocity
        rospy.loginfo("BASEPOSITION=="+str(state[0]))
        rospy.loginfo("POLE ANGLE==" + str(state[2]))
        if (self.min_base_position >= state[0] or state[0] >= self.max_base_position): #check if the base is still within the ranges of (-2, 2)
            rospy.logerr("Base Ouside Limits==>min="+str(self.min_base_position)+",pos="+str(state[0])+",max="+str(self.max_base_position))
            done = True
        if (self.min_pole_angle >= state[2] or state[2] >= self.max_pole_angle): #check if pole has toppled over
            rospy.logerr(
                "Pole Angle Ouside Limits==>min=" + str(self.min_pole_angle) + ",pos=" + str(state[2]) + ",max=" + str(
                    self.max_pole_angle))
            done = True

        observations = [state[2]]

        return observations, done, state

    def get_reward_for_observations(self, state):
        """
        Gives more points for staying upright, gets data from given observations to avoid
        having different data than other previous functions
        :return:reward
        """

        pole_angle = state[2]
        pole_vel = state[3]

        rospy.logwarn("pole_angle for reward==>" + str(pole_angle))
        delta = 0.7 - abs(pole_angle)
        reward_pole_angle = math.exp(delta*10)

        # If we are moving to the left and the pole is falling left is Bad
        rospy.logwarn("pole_vel==>" + str(pole_vel))
        pole_vel_sign = numpy.sign(pole_vel)
        pole_angle_sign = numpy.sign(pole_angle)
        rospy.logwarn("pole_vel sign==>" + str(pole_vel_sign))
        rospy.logwarn("pole_angle sign==>" + str(pole_angle_sign))

        # We want inverted signs for the speeds. We multiply by -1 to make minus positive.
        # global_sign + = GOOD, global_sign - = BAD
        base_reward = 500
        if pole_vel != 0:
            global_sign = pole_angle_sign * pole_vel_sign * -1
            reward_for_efective_movement = base_reward * global_sign
        else:
            # Is a particular case. If it doesnt move then its good also
            reward_for_efective_movement = base_reward

        reward = reward_pole_angle + reward_for_efective_movement

        rospy.logwarn("reward==>" + str(reward)+"= r_pole_angle="+str(reward_pole_angle)+",r_movement= "+str(reward_for_efective_movement))
        return reward


    def check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while (self._base_pub.get_num_connections() == 0 and not rospy.is_shutdown()):
            rospy.logdebug("No susbribers to _base_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_base_pub Publisher Connected")

        while (self._pole_pub.get_num_connections() == 0 and not rospy.is_shutdown()):
            rospy.logdebug("No susbribers to _pole_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_pole_pub Publisher Connected")

        rospy.logdebug("All Publishers READY")

    def check_all_systems_ready(self, init=True):
        self.base_position = None
        while self.base_position is None and not rospy.is_shutdown():
            try:
                self.base_position = rospy.wait_for_message("/cartpole_v0/joint_states", JointState, timeout=1.0)
                rospy.logdebug("Current cartpole_v0/joint_states READY=>"+str(self.base_position))
                if init:
                    # We Check all the sensors are in their initial values
                    positions_ok = all(abs(i) <= 1.0e-02 for i in self.base_position.position)
                    velocity_ok = all(abs(i) <= 1.0e-02 for i in self.base_position.velocity)
                    efforts_ok = all(abs(i) <= 1.0e-01 for i in self.base_position.effort)
                    base_data_ok = positions_ok and velocity_ok and efforts_ok
                    rospy.logdebug("Checking Init Values Ok=>" + str(base_data_ok))
            except:
                rospy.logerr("Current cartpole_v0/joint_states not ready yet, retrying for getting joint_states")
        rospy.logdebug("ALL SYSTEMS READY")


    def move_joints(self, joints_array):
        joint_value = Float64()
        joint_value.data = joints_array[0]
        rospy.logdebug("Single Base JointsPos>>"+str(joint_value))
        self._base_pub.publish(joint_value)


    def set_init_pose(self):
        """
        Sets joints to initial position [0,0,0]
        :return:
        """
        self.check_publishers_connection()
        # Reset Internal pos variable
        self.init_internal_vars(self.init_pos)
        self.move_joints(self.pos)