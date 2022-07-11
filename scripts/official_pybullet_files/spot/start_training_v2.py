#!/usr/bin/python3


from statistics import mode
from stable_baselines3 import A2C, PPO
from stable_baselines3.common.env_util import make_vec_env
import gym
import torch as th
from torch import nn
import numpy as np
from spot_gym_env import spotGymEnv
from stable_baselines3.common.policies import ActorCriticPolicy

import rospkg
# import torch as th
env = make_vec_env(spotGymEnv, n_envs=12)
# env = spotGymEnv(render=False)

import rospkg
rospack = rospkg.RosPack()



if __name__ == "__main__":
    pkg_path = rospack.get_path('spot_mini_ros')
    log_path =  pkg_path+'/training_results'
    outdir_2 = pkg_path + '/training_results/{}m_ppo_model.zip'
    # buffer_outdir = pkg_path + '/training_results/{}m_ppo_save_replay_buffer'

    # outdir_for_replaybuffer = pkg_path + '/training_results/{}00K_ppo_replay_buffer'
    policy_kwargs = dict(activation_fn=th.nn.ReLU, net_arch=[1024, dict(pi=[400, 300], vf=[400, 300, 300, 300])])

    model = PPO('MlpPolicy', env, verbose=4, learning_rate=0.0004, tensorboard_log=log_path, n_steps=10240, policy_kwargs=policy_kwargs, create_eval_env=True, seed=10)
    # model = PPO.load(in_dir, env=env)
    for i in range(1, 1000):
        model.learn(total_timesteps= 2000000, log_interval=1,)
        model.save(outdir_2.format(i*2))
        # model.save_replay_buffer(buffer_outdir.format(i*2))
        # model.sa

