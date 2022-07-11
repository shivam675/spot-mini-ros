#!/usr/bin/python3

import gym
from stable_baselines3 import PPO, DDPG, TD3
# from stable_baselines3.common.vec_env import DummyVecEnv
import walker_env
import rospy
import rospkg
# import torch as th

rospack = rospkg.RosPack()

if __name__ == "__main__":
    rospy.init_node('bipedal_gym', anonymous=True, log_level=rospy.INFO)
    env_name = 'quadwalker-v1'
    env = gym.make(env_name)
    pkg_path = rospack.get_path('quad_walker')
    log_path =  pkg_path+'/training_results'
    # in_dir = pkg_path + '/training_results/1900K_ppo_model.zip'
    outdir_2 = pkg_path + '/training_results/{}0K_ppo_model.zip'
    # outdir_for_replaybuffer = pkg_path + '/training_results/{}00K_ppo_replay_buffer'

    model = PPO('MlpPolicy', env, verbose=4, learning_rate=0.0003, tensorboard_log=log_path, n_steps=1024)
    # model = PPO.load(in_dir, env=env)
    for i in range(1, 101):
        model.learn(total_timesteps= 20000, log_interval=1)
        model.save(outdir_2.format(i))

