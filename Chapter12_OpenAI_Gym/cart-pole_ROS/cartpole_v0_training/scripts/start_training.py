#!/usr/bin/env python
import gym
import time
import numpy
import random
import time
import qlearn
from gym import wrappers

# ROS packages required
import rospy
import rospkg
from std_msgs.msg import Int16MultiArray
# Publish the reward of each episode
pub_reward = rospy.Publisher('reward', Int16MultiArray, queue_size=10)
reward_msg = Int16MultiArray()

# import our training environment
from openai_ros.task_envs.cartpole_stay_up import stay_up


if __name__ == '__main__':
    
    #rospy.init_node('cartpole_gym', anonymous=True, log_level=rospy.WARN)
    rospy.init_node('cartpole_gym', anonymous=True, log_level=rospy.INFO)

    # Create the Gym environment
    env = gym.make('CartPoleStayUp-v0')
    rospy.loginfo ( "Gym environment done")
        
    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('cartpole_v0_training')
    outdir = './tmp/training_results'
    env = wrappers.Monitor(env, outdir, force=True) 
    rospy.loginfo ( "Monitor Wrapper started")

    last_time_steps = numpy.ndarray(0)

    # Loads parameters from the ROS param server
    # Parameters are stored in a yaml file inside the config directory
    # They are loaded at runtime by the launch file
    Alpha = rospy.get_param("/cartpole_v0/alpha")
    Epsilon = rospy.get_param("/cartpole_v0/epsilon")
    Gamma = rospy.get_param("/cartpole_v0/gamma")
    epsilon_discount = rospy.get_param("/cartpole_v0/epsilon_discount")
    nepisodes = rospy.get_param("/cartpole_v0/nepisodes")
    nsteps = rospy.get_param("/cartpole_v0/nsteps")

    running_step = rospy.get_param("/cartpole_v0/running_step")

    # Initialises the algorithm that we are going to use for learning
    qlearn = qlearn.QLearn(actions=range(env.action_space.n),
                    alpha=Alpha, gamma=Gamma, epsilon=Epsilon)
    initial_epsilon = qlearn.epsilon

    start_time = time.time()
    highest_reward = 0

    # Starts the main training loop: the one about the episodes to do
    for x in range(nepisodes):
        rospy.logwarn("############### START EPISODE=>" + str(x+1))
        
        cumulated_reward = 0  
        done = False
        if qlearn.epsilon > 0.05:
            qlearn.epsilon *= epsilon_discount
        
        # Initialize the environment and get first state of the robot
        observation = env.reset()
        state = ''.join(map(str, observation))
        
        # Show on screen the actual situation of the robot
        # for each episode, we test the robot for nsteps
        for i in range(nsteps):
            rospy.logdebug("############### Start Step=>"+str(i))
            # Pick an action based on the current state
            action = qlearn.chooseAction(state)
            rospy.logdebug ("Next action is:%d", action)
            # Execute the action in the environment and get feedback
            observation, reward, done, info = env.step(action)
            rospy.logdebug("OBSERVATION" + str(observation) + " Reward = " + str(reward))
            cumulated_reward += reward
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            nextState = ''.join(map(str, observation))

            # Make the algorithm learn based on the results
            rospy.logdebug("############### state we were=>" + str(state))
            rospy.logdebug("############### action that we took=>" + str(action))
            rospy.logdebug("############### reward that action gave=>" + str(reward))
            rospy.logdebug("############### State in which we will start nect step=>" + str(nextState))
            qlearn.learn(state, action, reward, nextState)

            if not(done):
                state = nextState
            else:
                rospy.logwarn ("DONE")
                last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
                break
            rospy.logdebug("############### END Step=>" + str(i))
        
        reward_msg.data = [x, cumulated_reward]
        rospy.loginfo('topic /reward => ' + str(reward_msg.data))
        pub_reward.publish(reward_msg)

        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        rospy.logwarn ( ("EP: "+ str(x+1 )+ "] - Reward: " +str(cumulated_reward) + "     Time: %d:%02d:%02d" % (h, m, s)) + " - [alpha: "+str(round(qlearn.alpha,2))+" - gamma: "+str(round(qlearn.gamma,2))+" - epsilon: "+str(round(qlearn.epsilon,2)))

    rospy.logwarn ( ("\n|"+str(nepisodes)+"|"+str(qlearn.alpha)+"|"+str(qlearn.gamma)+"|"+str(initial_epsilon)+"*"+str(epsilon_discount)+"|"+str(highest_reward)+"| PICTURE |"))

    l = last_time_steps.tolist()
    l.sort()

    #print("Parameters: a="+str)
    rospy.logwarn("Overall score: {:0.2f}".format(last_time_steps.mean()))
    rospy.logwarn("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))

    env.close()

