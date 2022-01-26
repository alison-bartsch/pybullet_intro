import gym
import gym_cartpole
import time
from stable_baselines3 import PPO
import numpy as np
import matplotlib.pyplot as plt
import os
import imageio

def main():

  # Create and wrap the environment
  env = gym.make('mycartpole-v0')
  # model = PPO('MlpPolicy',env,verbose=1)
  # model.learn(total_timesteps=50000)
  # model.save('./ppo_cartpole')

  model = PPO.load('ppo_cartpole')
  frames = []
  obs=env.reset()
  for i in range(500):
      action,_state=model.predict(obs,deterministic=True)
      obs,reward,done,info=env.step(action)
      frames.append(env.render())
      
      if done:
          obs = env.reset()
  imageio.mimsave('my_gif.gif', frames, 'GIF', duration=0.02)
    
if __name__ == '__main__':
    main()