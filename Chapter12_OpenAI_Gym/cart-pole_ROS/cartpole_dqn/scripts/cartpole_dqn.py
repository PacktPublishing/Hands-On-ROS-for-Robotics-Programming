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


class DQNCartPoleSolver():
    def __init__(self, n_episodes=1000, n_win_ticks=195, min_episodes= 100, max_env_steps=None, gamma=1.0, epsilon=1.0, epsilon_min=0.01, epsilon_log_decay=0.995, alpha=0.01, alpha_decay=0.01, batch_size=64, monitor=False, quiet=False):
        self.memory = deque(maxlen=100000)
        self.env = gym.make('CartPoleStayUp-v0')
        if monitor: self.env = gym.wrappers.Monitor(self.env, '../data/cartpole-1', force=True)
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_min = epsilon_min
        self.epsilon_decay = epsilon_log_decay
        self.alpha = alpha
        self.alpha_decay = alpha_decay
        self.n_episodes = n_episodes
        self.n_win_ticks = n_win_ticks
        self.min_episodes = min_episodes
        self.batch_size = batch_size
        self.quiet = quiet
        if max_env_steps is not None: self.env._max_episode_steps = max_env_steps

        # Init model
        self.model = Sequential()
        self.model.add(Dense(24, input_dim=4, activation='tanh'))
        self.model.add(Dense(48, activation='tanh'))
        self.model.add(Dense(2, activation='linear'))
        self.model.compile(loss='mse', optimizer=Adam(lr=self.alpha, decay=self.alpha_decay))

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def choose_action(self, state, epsilon):
        return self.env.action_space.sample() if (np.random.random() <= epsilon) else np.argmax(self.model.predict(state))

    def get_epsilon(self, t):
        return max(self.epsilon_min, min(self.epsilon, 1.0 - math.log10((t + 1) * self.epsilon_decay)))

    def preprocess_state(self, state):
        return np.reshape(state, [1, 4])

    def replay(self, batch_size):
        x_batch, y_batch = [], []
        minibatch = random.sample(
            self.memory, min(len(self.memory), batch_size))
        for state, action, reward, next_state, done in minibatch:
            y_target = self.model.predict(state)
            y_target[0][action] = reward if done else reward + self.gamma * np.max(self.model.predict(next_state)[0])
            x_batch.append(state[0])
            y_batch.append(y_target[0])
        
        self.model.fit(np.array(x_batch), np.array(y_batch), batch_size=len(x_batch), verbose=0)
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def run(self):
        
        rate = rospy.Rate(30)
        
        scores = deque(maxlen=min_episodes)

        for e in range(self.n_episodes):
            state = self.preprocess_state(self.env.reset())
            done = False
            i = 0                 # TICKS
            cumulated_reward = 0  # REWARD
            while not done:
                action = self.choose_action(state, self.get_epsilon(e))
                next_state, reward, done, _ = self.env.step(action)
                next_state = self.preprocess_state(next_state)
                self.remember(state, action, reward, next_state, done)
                state = next_state
                i += 1                      # TICKS
                cumulated_reward += reward  # REWARD

            scores.append(i)
            mean_score = np.mean(scores)
            if mean_score >= self.n_win_ticks and e >= min_episodes:
                if not self.quiet: print('Ran {} episodes. Solved after {} trials'.format(e, e - min_episodes))
                return e - min_episodes
            if e % 1 == 0 and not self.quiet:
                print('[Episode {}] {} ticks - Mean survival time over last {} episodes was {} ticks.'.format(e, i, min_episodes ,mean_score))
                reward_msg.data = [e, mean_score]       # episode, mean_score
                pub_reward.publish(reward_msg)          # mean_score
                ticks_msg.data = [e, i]                 # episode, TICKS
                pub_ticks.publish(ticks_msg)            # TICKS

            self.replay(self.batch_size)

        if not self.quiet: print('Did not solve after {} episodes'.format(e))
        return e
        
if __name__ == '__main__':
    rospy.init_node('cartpole_n1try_algorithm', anonymous=True, log_level=rospy.INFO)
    
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
    
    
    agent = DQNCartPoleSolver(  n_episodes,
                                n_win_ticks,
                                min_episodes,
                                max_env_steps,
                                gamma,
                                epsilon,
                                epsilon_min,
                                epsilon_log_decay,
                                alpha,
                                alpha_decay,
                                batch_size,
                                monitor,
                                quiet)
    agent.run()