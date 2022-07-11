#!/usr/bin/env python3
'''
    By Shivam Chavan <shivam31199@gmail.com>
    Visit our website at www.melodic.pythonanywhere.com
'''

import gym
from std_msgs.msg import Float32, Float32MultiArray
import rospy
import numpy as np
import time
from gym import utils, spaces
from geometry_msgs.msg import Pose
from gym.utils import seeding
from gym.envs.registration import register
from gazebo_connection import GazeboConnection
from joint_publisher import JointPub
from walker_state import WalkerState
from controllers_connection import ControllersConnection


#register the training environment in the gym as an available one
reg = register(
    id='spot-v1',
    entry_point='walker_env:SpotWalkerEnv')


class SpotWalkerEnv(gym.Env):

    def __init__(self):
        
        # We assume that a ROS node has already been created
        # before initialising the environment
        self.reward = 0
        self.running_step = 0.02

        self.gazebo = GazeboConnection()
        self.reward_publisher = rospy.Publisher('/reward_per_step', Float32, queue_size=10)
        self.obs_pub_object = rospy.Publisher('/obs', Float32MultiArray, queue_size= 2)
        self.action_pub_object = rospy.Publisher('/action_per_step', Float32MultiArray, queue_size= 2)
        self.controllers_object = ControllersConnection(namespace='/spot')
        self.walker_state_object = WalkerState(desired_x = 6, done_reward= -1000)
        self.walker_joint_pubisher_object = JointPub()
        
        # self.action_space = spaces.Box(low=-np.inf, high=np.inf, shape=(10,),dtype=np.float32)
        self.action_space = spaces.Box(low=-1.4, high=1.4, shape=(8,),dtype=np.float)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(40,),dtype=np.float)
        # print(self.action_space)
        # print(self.observation_space)
        self.reward_range = (-1000, np.inf)
        self.seed()

    # A function to initialize the random generator
    def seed(self, seed=0):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    # Resets the state of the environment and returns an initial observation.
    def reset(self):
        self.gazebo.pauseSim()
        self.gazebo.resetSim()
        self.gazebo.change_gravity(0.0, 0.0, 0.0)
        self.controllers_object.reset_walker_joint_controllers()
        self.walker_joint_pubisher_object.set_init_pose()
        self.walker_state_object.check_all_systems_ready()
        # print('Quickly done')
        observation = self.walker_state_object.get_observations()
        # print(len(observation))
        self.gazebo.change_gravity(0.0, 0.0, -9.81)
        self.gazebo.pauseSim()
        # print(len(state))
        # print('Last Eps Total Reward: {}'.format(self.reward), flush=True, end='\r')
        self.reward = 0
        self.step_number = 0

        return observation

    def step(self, action):
        # t1 = time.time()
        action_msg = Float32MultiArray()
        observation_msg = Float32MultiArray()
        reward_msg = Float32()
        self.step_number += 1
        action_pose_dict = self.walker_state_object.dump_previous_actions(action,self.step_number)
        self.gazebo.unpauseSim()
        self.walker_joint_pubisher_object.move_joints(action_pose_dict)
        action_msg.data = list(action)
        self.gazebo.pauseSim()

        observation = self.walker_state_object.get_observations()
        observation_msg.data = list(observation)
        reward, done = self.walker_state_object.process_data()
        reward_msg.data = reward
        # msg.data = reward
        self.reward_publisher.publish(reward_msg)
        self.obs_pub_object.publish(observation_msg)
        self.action_pub_object.publish(action_msg)
        # t2 = time.time()
        # print(t2-t1)
        # print(self.step_number)
        return observation, reward, done, {}