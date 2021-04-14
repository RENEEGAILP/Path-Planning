from time import sleep

import pybullet as p
import pybullet_data

p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("../data/plane.urdf")
# p.loadURDF("../data/wall.urdf")
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
# for step in range(3000):
while(True):
  p.stepSimulation()

p.getContactPoints(husky)


# take robot_1 to desired target location
# referred Kinodynamic...have to figure out which joints we need to move
def moveRobotToGoal(robot_id, target_location):
    print("Hello")

    robot_pos, robot_orn = p.getBasePositionAndOrientation(robot_id)
    euler_orn = p.getEulerFromQuaternion(robot_orn)
    #  pose = [cubePos[0], cubePos[1], cubeOrn[2], 0.0, 0.0]

    # Controller parameters
    Kp = 0.7  # 1.5

    # get the goal location
    xd = target_location[0]
    yd = target_location[1]

    # get the current robot location
    xa = robot_pos[0]
    ya = robot_pos[1]
    thetaa = euler_orn[2]

    # determine how far to rotate to face the goal point
    # PS. ALL ANGLES ARE IN RADIANS
    dt = (np.arctan2((yd - ya), (xd - xa))) - thetaa
    # restrict angle to (-pi,pi)
    dt = ((dt + np.pi) % (2.0 * np.pi)) - np.pi
    dt = ((dt * 180.0) / np.pi)

    # control input for angular velocity
    W = (Kp * dt)

    # find distance to goal
    d = np.sqrt(pow((xd - xa), 2) + pow((yd - ya), 2))

    # control input for linear velocity
    if d > 0.05:
        V = 10.0
    else:
        V = 0.0

    # Parameters
    R = 0.7  # radius of wheels
    L = 1  # total distance between wheels

    # Compute desired vel_r, vel_l needed to ensure w
    Vr = ((2.0 * V) + (W * L)) / (2 * R)
    Vl = ((2.0 * V) - (W * L)) / (2 * R)

    # Set theta1
    p.setJointMotorControl2(robot_id, 0, p.POSITION_CONTROL, targetPosition=0.0, force=750)

    # Set theta2
    p.setJointMotorControl2(robot_id, 1, p.POSITION_CONTROL, targetPosition=0.0, force=750)

    for i in range(6000):
        p.setJointMotorControl2(robot_id, 2, p.VELOCITY_CONTROL, targetVelocity=-Vr, force=500)
        p.setJointMotorControl2(robot_id, 3, p.VELOCITY_CONTROL, targetVelocity=-Vl, force=500)
        p.setJointMotorControl2(robot_id, 4, p.VELOCITY_CONTROL, targetVelocity=-Vr, force=500)
        p.setJointMotorControl2(robot_id, 5, p.VELOCITY_CONTROL, targetVelocity=-Vl, force=500)
        p.stepSimulation()
    return


pos, orn = p.getBasePositionAndOrientation(husky_1)
moveRobotToGoal(husky_1, [10, 10])

# p.resetBasePositionAndOrientation(husky_1, [10, 10, 0], [0, 0, 1, 1])
#
# numJoints = p.getNumJoints(husky_1)
# for joint in range(numJoints):
#   print(p.getJointInfo(husky_1, joint))
# targetVel = 1  #rad/s
# maxForce = 100  #Newton
#
# for joint in range(2, 6):
#   p.setJointMotorControl2(husky_1, joint, p.POSITION_CONTROL, targetPosition=100.0, force=maxForce)
#   p.setJointMotorControl(husky_1, joint, p.VELOCITY_CONTROL, targetVel, maxForce)
#   #p.setJointMotorControl(husky_2, joint, p.VELOCITY_CONTROL, -targetVel, maxForce)
#
# while(True):
#   p.stepSimulation()
#
#
# exit(0)
# ------------------------------------------------------





p.disconnect()