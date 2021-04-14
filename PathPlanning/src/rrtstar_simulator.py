import pybullet as p
import numpy as np

# creating the environment
p.connect(p.GUI)

p.loadURDF("../data/plane.urdf")
p.setGravity(0, 0, -10)

# adding static obstacles to the environment (uncomment for obstacles)
obs_1 = p.loadURDF("../data/obstacle.urdf", [0, 10, 0], useFixedBase=1)
obs_2 = p.loadURDF("../data/obstacle.urdf", [-5, -10, 0], useFixedBase=1)
obs_3 = p.loadURDF("../data/obstacle.urdf", [5, -10, 0], useFixedBase=1)
obs_4 = p.loadURDF("../data/obstacle.urdf", [-5, 0, 0], useFixedBase=1)
obs_5 = p.loadURDF("../data/obstacle.urdf", [5, 0, 0], useFixedBase=1)

# initializing the robot
husky1pos = [1, 1, 0.1]
husky2pos = [-1, -1, 0.1]
husky_1 = p.loadURDF("../data/husky/husky.urdf", husky1pos)
husky_2 = p.loadURDF("../data/husky/husky.urdf", husky2pos)

# Adjust the position of the camera
p.resetDebugVisualizerCamera(cameraDistance=25, cameraYaw=-180, cameraPitch=-120, cameraTargetPosition=[0, 0, 0])

numJoints = p.getNumJoints(husky_1)
for joint in range(numJoints):
    print(p.getJointInfo(husky_1, joint))
targetVel = 10  # rad/s
maxForce = 100  # Newton

for joint in range(2, 6):
    p.setJointMotorControl(husky_1, joint, p.VELOCITY_CONTROL, targetVel, maxForce)
    p.setJointMotorControl(husky_2, joint, p.VELOCITY_CONTROL, -targetVel, maxForce)

carPos, carOrn = p.getBasePositionAndOrientation(husky_1)
print("-----------------" + str(carOrn))
p.resetBasePositionAndOrientation(husky_1, carPos, [0, 0, 1, 1])
for step in range(3000):
    p.stepSimulation()

targetVel = -10
for joint in range(2, 6):
    p.setJointMotorControl(husky_1, joint, p.VELOCITY_CONTROL, targetVel, maxForce)
    p.setJointMotorControl(husky_2, joint, p.VELOCITY_CONTROL, -targetVel, maxForce)
# for step in range(3000):
while (True):
    p.stepSimulation()

p.getContactPoints(husky_1)
p.disconnect()
