# # 1. It renders instances for 500 timesteps, performing random actions.
# import gym
# env = gym.make('Acrobot-v1')
# env.reset()
# for _ in range(500):
#     env.render()
#     env.step(env.action_space.sample())

# # 2. To check all env available, uninstalled ones are also shown.
# from gym import envs 
# print(envs.registry.all())





# import gym
# env = gym.make('MountainCarContinuous-v0') # try for different environments
# observation = env.reset()
# for t in range(100):
#         env.render()
#         print(observation)
#         action = env.action_space.sample()
#         observation, reward, done, info = env.step(action)
#         print(observation, reward, done, info)
#         if done:
#             print("Finished after {} timesteps".format(t+1))
#             break



# import gym
# env = gym.make('Acrobot-v1')
# env.reset()
# for _ in range(1000):
#     env.render()
#     observation, reward, done, info = env.step(env.action_space.sample()) # take a random action
# env.close()



import gym
env = gym.make('CartPole-v0')
for i_episode in range(20):
    observation = env.reset()
    for t in range(100):
        env.render()
        print(observation)
        action = env.action_space.sample()  # take a random action still?
        observation, reward, done, info = env.step(action)
        if done:
            print("Episode finished after {} timesteps".format(t+1))
            break
env.close()


