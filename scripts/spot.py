import pybullet as p
import numpy as np
import os
import rospkg


rospack = rospkg.RosPack()
pkg_path = rospack.get_path('spot_mini_ros')
urdf_path = pkg_path + '/urdf/spot.urdf'


class Spot:
    def __init__(self, client):
        self.client = client
        f_name = urdf_path
        self.spot = p.loadURDF(fileName=f_name,
                              basePosition=[0, 0, 0.3],
                              physicsClientId=client)

    def get_ids(self):
        return self.client, self.car

    def apply_action(self, action):
        pass
    
    def get_observation(self):
        p.get_observations(self.spot)
        pass