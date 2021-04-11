from time import sleep

import pybullet as p
import pybullet_data

p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("../data/plane.urdf")
p.loadURDF("../data/wall.urdf")
p.setGravity(0, 0, -10)
husky1pos = [1, 1, 0.1]
husky2pos = [-1, -1, 0.1]

husky = p.loadURDF("../data/husky/husky.urdf", husky1pos[0], husky1pos[1], husky1pos[2])
husky_2 = p.loadURDF("../data/husky/husky.urdf", husky2pos[0], husky2pos[1], husky2pos[2])

numJoints = p.getNumJoints(husky)
for joint in range(numJoints):
  print(p.getJointInfo(husky, joint))
targetVel = 10  #rad/s
maxForce = 100  #Newton


for joint in range(2, 6):
  p.setJointMotorControl(husky, joint, p.VELOCITY_CONTROL, targetVel, maxForce)
  p.setJointMotorControl(husky_2, joint, p.VELOCITY_CONTROL, -targetVel, maxForce)


carPos, carOrn = p.getBasePositionAndOrientation(husky)
print("-----------------" + str(carOrn))
p.resetBasePositionAndOrientation(husky, carPos, [0, 0, 1, 1])
for step in range(3000):
  p.stepSimulation()

targetVel = -10
for joint in range(2, 6):
  p.setJointMotorControl(husky, joint, p.VELOCITY_CONTROL, targetVel, maxForce)
  p.setJointMotorControl(husky_2, joint, p.VELOCITY_CONTROL, -targetVel, maxForce)
for step in range(3000):
  p.stepSimulation()

p.getContactPoints(husky)

p.disconnect()