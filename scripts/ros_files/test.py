#!/usr/bin/env python3

# import mujoco_py    
# import rospkg
import time
import numpy as np
# from gym import spaces
import gym
import walker_env
import rospy
rospy.init_node('walker_agent_node', anonymous=True)
env = gym.make('quadwalker-v1')

steps = 1000
n_updates = 510

start = time.time()
for i in range(steps):
    state = env.reset()
    done = False
    # print(f'Checking if the state is part of the observation space: {env.observation_space.contains(state)}')
    # print(np.shape(state))
    # print(len(state))
    # print(state)
    r = 0
    for i in range(n_updates):
        action = env.action_space.sample()
        # print(action)
        obs, rew, done, info = env.step(action)
        # print(, rew)
        # r += rew
        # time.sleep(0.01)
        # print(obs[-12:  -9])
        # print(f'Checking if the state is part of the observation space: {env.observation_space.contains(state)}')
        # state
        # time.sleep(0.005)
        if done:
            break
    # print(i, r)
    # break
    # break
    # viewer.loop_once()
    # data, width, height = viewer.get_image()
    # img = np.fromstring(data, dtype='uint8').reshape(height, width, 3)[::-1,:,:]
    # imsave('imgs/out_' + str(i) + '.png', img)
    # for j in range(skip):
        # model.step()
# print()
end = time.time()
# print(end - start)

# viewer.finish()
viewer = None