import pybullet as p
import os
import math


class Cartpole:
    def __init__(self, client):
        self.client = client
        f_name = os.path.join(os.path.dirname(__file__), 'simplecartpole.urdf')
        self.cartpole = p.loadURDF(fileName = f_name,
                              basePosition = [0, 0, 0],
                              physicsClientId = client)

        self.slider_to_car_joints = 0
        self.car_to_pole_joints = 1
        self.speed = 1

        p.setJointMotorControl2(self.cartpole, self.car_to_pole_joints, 
                                    controlMode=p.VELOCITY_CONTROL,
                                    force = 0)

    def get_ids(self):
        return self.cartpole, self.client

    def apply_action(self, action):
        # Expects action to be two dimensional
        if action == 0:
            direction = -1
        if action == 1:
            direction = 1

        # Set the car speed
        p.setJointMotorControl2(self.cartpole, self.slider_to_car_joints,
                                    controlMode = p.VELOCITY_CONTROL,
                                    targetVelocity = self.speed * direction,
                                    physicsClientId = self.client)


    def get_observation(self):
        # Get the position and orientation of the car in the simulation
        car_state = p.getJointState(self.cartpole,self.slider_to_car_joints)[0:2]
        rod_state = p.getJointState(self.cartpole, self.car_to_pole_joints)[0:2]

        observation = car_state + rod_state

        return observation
