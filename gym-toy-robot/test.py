import pybullet
import pybullet_data
import time

pybullet.connect(pybullet.GUI)
pybullet.setGravity(0, 0, -9.81)

angle = pybullet.addUserDebugParameter('Steering', -0.5, 0.5, 0)
throttle = pybullet.addUserDebugParameter('Throttle', 0, 20, 0)
car = pybullet.loadURDF('simplecar.urdf', [0, 0, 0.1])
plane = pybullet.loadURDF('simpleplane.urdf')

time.sleep(3)

wheel_indices = [1, 3, 4, 5]
hinge_indices = [0, 2]

while True:
	user_angle = pybullet.readUserDebugParameter(angle)
	user_throttle = pybullet.readUserDebugParameter(throttle)

	for joint_index in wheel_indices:
		pybullet.setJointMotorControl2(car, joint_index, pybullet.VELOCITY_CONTROL, targetVelocity=user_throttle)

	for joint_index in hinge_indices:
		pybullet.setJointMotorControl2(car, joint_index, pybullet.POSITION_CONTROL, targetPosition=user_angle)

	pybullet.stepSimulation()




# car = pybullet.loadURDF('simplecar.urdf')
# number_of_joints = pybullet.getNumJoints(car)

# for joint_number in range(number_of_joints):
# 	info = pybullet.getJointInfo(car, joint_number)
# 	print(info[0], ": ", info[1])








# client = pybullet.connect(pybullet.GUI)
# pybullet.setGravity(0, 0, -9.81, physicsClientId=client)

# pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
# planeID = pybullet.loadURDF("plane.urdf")

# carID = pybullet.loadURDF("racecar/racecar.urdf", basePosition=[0,0,0.2])

# position, orientation = pybullet.getBasePositionAndOrientation(carID)

# for __ in range(300):
# 	pos, ori = pybullet.getBasePositionAndOrientation(carID)
# 	pybullet.applyExternalForce(carID, 0, [50, 0, 0], pos, pybullet.WORLD_FRAME)
# 	pybullet.stepSimulation()
# 	time.sleep(1./240.)