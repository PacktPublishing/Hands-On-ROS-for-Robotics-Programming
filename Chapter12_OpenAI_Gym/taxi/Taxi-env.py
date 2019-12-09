import gym
env = gym.make('Taxi-v3')
env.reset()
for _ in range(150):
    env.render()
    env.step(env.action_space.sample()) # take a random action
    print _
env.close()