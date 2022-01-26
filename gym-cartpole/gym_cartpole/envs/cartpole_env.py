import gym
# from gym import error, spaces, utils
# from gym.utils import seeding
import numpy as np
import pybullet as p
# import time
import matplotlib.pyplot as plt
from gym_cartpole.resources.cartpole import Cartpole
from gym_cartpole.resources.plane import Plane

class Cartpole_Env(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        # set the action space & observation space
        self.action_space = gym.spaces.Discrete(2)
        self.position = 7.5
        self.angle = 25/180*3.14159
        self.observation_space = gym.spaces.box.Box(
                low=np.array([-self.position*2, -np.inf, -self.angle*2, -np.inf], dtype=np.float32),
                high=np.array([self.position*2, np.inf, self.angle*2, np.inf], dtype=np.float32))
        # self.np_random, _ = gym.utils.seeding.np_random()
        self.client = p.connect(p.DIRECT)
        # self.client = p.connect(p.GUI)
        p.setTimeStep(1/24, self.client)

        self.cartpole = None
        self.done = False
        self.rendered_img = None
        self.reset()

        self.viewMatrix = p.computeViewMatrix(cameraEyePosition=[0, 3, 1],
                                        cameraTargetPosition=[0, 0, 0],
                                        cameraUpVector=[0, 0, 1])
        self.projectionMatrix = p.computeProjectionMatrixFOV(fov=50,
                                                            aspect=1.0,
                                                            nearVal=0.1,
                                                            farVal=100)
    
    def step(self, action):
        self.cartpole.apply_action(action)
        p.stepSimulation()
        ob = self.cartpole.get_observation()

        self.done = bool(ob[0] < -self.position
                    or ob[0] > self.position
                    or ob[2] < -self.angle
                    or ob[2] > self.angle)

        if not self.done:
            reward = 1.0
        else:
            reward = 0

        return ob, reward, self.done, dict()

    def reset(self):
        p.resetSimulation(self.client)
        p.setGravity(0, 0, -9.8)
        # Reload the plane and car
        Plane(self.client)
        self.cartpole = Cartpole(self.client)

        self.done = False

        # # Get observation to return
        observation = self.cartpole.get_observation()
        return observation

    def render(self, mode='human', close=False):
        if self.rendered_img is None:
            self.rendered_img = plt.imshow(np.zeros((224, 224, 4)))

        _, _, frame, _, _ = p.getCameraImage(width=224, height=224,
                                              viewMatrix=self.viewMatrix,
                                              projectionMatrix=self.projectionMatrix)
        # plt.imshow(rgbImg)
        frame = np.reshape(frame, (224, 224, 4))
        self.rendered_img.set_data(frame)
        plt.draw()
        plt.pause(.01)
        return frame

    def close(self):
        p.disconnect(self.client)