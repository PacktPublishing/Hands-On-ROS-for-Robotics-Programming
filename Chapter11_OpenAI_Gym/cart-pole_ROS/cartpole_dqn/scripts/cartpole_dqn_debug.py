#!/usr/bin/env python
import rospy

import os
import random
import gym
import math
import numpy as np
from collections import deque
from keras.models import Sequential
from keras.layers import Dense
from keras.optimizers import Adam
from std_msgs.msg import Int16MultiArray
# Publish the reward of each episode
pub_reward = rospy.Publisher('reward', Int16MultiArray, queue_size=10)
reward_msg = Int16MultiArray()
from std_msgs.msg import Int16MultiArray
# Publish the reward of each episode
pub_ticks = rospy.Publisher('ticks', Int16MultiArray, queue_size=10)
ticks_msg = Int16MultiArray()

# import our training environment
from openai_ros.task_envs.cartpole_stay_up import stay_up

# FUNCTIONS +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# from class DQNCartPoleSolver
# Init model
model = Sequential()
model.add(Dense(24, input_dim=4, activation='tanh'))
model.add(Dense(48, activation='tanh'))
model.add(Dense(2, activation='linear'))
model.compile(loss='mse', optimizer=Adam(lr=alpha, decay=alpha_decay))

def remember(memory, state, action, reward, next_state, done):
    memory.append((state, action, reward, next_state, done))

def choose_action(env, model, state, epsilon):
    return env.action_space.sample() if (np.random.random() <= epsilon) else np.argmax(model.predict(state))

def get_epsilon(epsilon, epsilon_min, epsilon_decay, t):
    return max(epsilon_min, min(epsilon, 1.0 - math.log10((t + 1) * epsilon_decay)))

def preprocess_state(env, state):
    env.reset()
    return np.reshape(state, [1, 4])

def replay(memory, model, gamma, epsilon, epsilon_min, epsilon_decay, batch_size):
    x_batch, y_batch = [], []
    minibatch = random.sample(
        memory, min(len(memory), batch_size))
    for state, action, reward, next_state, done in minibatch:
        y_target = model.predict(state)
        y_target[0][action] = reward if done else reward + gamma * np.max(model.predict(next_state)[0])
        x_batch.append(state[0])
        y_batch.append(y_target[0])
        
    model.fit(np.array(x_batch), np.array(y_batch), batch_size=len(x_batch), verbose=0)
    if epsilon > epsilon_min:
        epsilon *= epsilon_decay


# MAIN FLOW +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
rospy.init_node('cartpole_n1try_algorithm', anonymous=True, log_level=rospy.INFO)
    
# INITIALIZATION (as of class DQNCartPoleSolver)
# -- Commented out are the default values in the setup of the class
# -- to be applied if the ROS parameter didn't exist
# n_episodes=1000
# n_win_ticks=195
# min_episodes= 100
# max_env_steps=None
# gamma=1.0
# epsilon=1.0, epsilon_min=0.01, epsilon_log_decay=0.995
# alpha=0.01, alpha_decay=0.01
# batch_size=64
# monitor=False
# quiet=False
n_episodes = rospy.get_param('/cartpole_v0/episodes_training')
n_win_ticks = rospy.get_param('/cartpole_v0/n_win_ticks')
min_episodes = rospy.get_param('/cartpole_v0/min_episodes')
max_env_steps = None
gamma =  rospy.get_param('/cartpole_v0/gamma')
epsilon = rospy.get_param('/cartpole_v0/epsilon')
epsilon_min = rospy.get_param('/cartpole_v0/epsilon_min')
epsilon_log_decay = rospy.get_param('/cartpole_v0/epsilon_decay')
alpha = rospy.get_param('/cartpole_v0/alpha')
alpha_decay = rospy.get_param('/cartpole_v0/alpha_decay')
batch_size = rospy.get_param('/cartpole_v0/batch_size')
monitor = rospy.get_param('/cartpole_v0/monitor')
quiet = rospy.get_param('/cartpole_v0/quiet')

# Commented out are the self __init__ of class DQNCartPoleSolver init parameter read from ROS parameter server
# Uncommented (or active line) of the variable name does not coincide with the ROS parameter variable
memory = deque(maxlen=100000)
env = gym.make('CartPoleStayUp-v0')
if monitor: env = gym.wrappers.Monitor(env, '../data/cartpole-1', force=True)
# gamma = gamma
# epsilon = epsilon
# epsilon_min = epsilon_min
epsilon_decay = epsilon_log_decay
# alpha = alpha
# alpha_decay = alpha_decay
# n_episodes = n_episodes
# n_win_ticks = n_win_ticks
# min_episodes = min_episodes
# batch_size = batch_size
# quiet = quiet
if max_env_steps is not None: env._max_episode_steps = max_env_steps

# FUNCTION run()
# from class DQNCartPoleSolver
print 'I am in' ##############################################################
rate = rospy.Rate(30)
        
scores = deque(maxlen=min_episodes)

# Exert an action to start
action = 0 #LEFT
#action = 1 #RIGHT
#action = 2 #LEFT BIG
#action = 3 #RIGHT BIG

# Compute the resulting state
env.reset()
state, reward0, done0, _ = env.step(action)

for e in range(n_episodes):
    print 'Episode ' + str(e) ################################################
    state = preprocess_state(env, state) ### -----!!!!!! state NOT DEFINED in first iteration e=0
    print 'Action in Episode ' + str(e) ######################################
    done = False
    i = 0                 # TICKS
    cumulated_reward = 0  # REWARD
    while not done:
        print 'Action in Episode ' + str(e) ##################################
        action = choose_action(env, model, state, get_epsilon(epsilon, epsilon_min, epsilon_decay, e))
        next_state, reward, done, _ = env.step(action)
        next_state = preprocess_state(env, next_state)
        remember(memory, state, action, reward, next_state, done)
        state = next_state
        i += 1                      # TICKS
       cumulated_reward += reward  # REWARD

    scores.append(i)
    mean_score = np.mean(scores)
    if mean_score >= n_win_ticks and e >= min_episodes:
        if not quiet: print('Ran {} episodes. Solved after {} trials'.format(e, e - min_episodes))
        return e - min_episodes
    if e % 1 == 0 and not quiet:
        print('[Episode {}] {} ticks - Mean survival time over last {} episodes was {} ticks.'.format(e, i, min_episodes ,mean_score))
        reward_msg.data = [e, mean_score]       # episode, mean_score
        pub_reward.publish(reward_msg)          # mean_score
        ticks_msg.data = [e, i]                 # episode, TICKS
        pub_ticks.publish(ticks_msg)            # TICKS

    replay(memory, model, gamma, epsilon, epsilon_min, epsilon_decay, batch_size)

if not quiet: print('Did not solve after {} episodes'.format(e))
return e