import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import pybullet as p



class SpotEnv(gym.Env):
    metadata = {'render.modes': ['human']}  

    def __init__(self):
        # self.client = p.connect(p.DIRECT)
        self.action_space = spaces.Box(low=np.array([0, -0.6]), high=np.array([1, 0.6]))
        self.observation_space = spaces.Box(low=np.array([-10]), high=np.array([10]))
        self.np_random, _ = seeding.np_random()

        

        # pass

    def step(self, action):
        pass

    def reset(self):
        pass

    def render(self):
        pass

    def close(self):
        pass
    def seed(self, seed=None): 
        self.np_random, seed = seeding.np_random(seed)
        return [seed]