import pybullet as p
import numpy as np
import operator

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
husky1pos = [-12, -12, 0.1]
husky2pos = [-1, -1, 0.1]
husky_1 = p.loadURDF("../data/husky/husky.urdf", husky1pos)
husky_2 = p.loadURDF("../data/husky/husky.urdf", husky2pos)

# Adjust the position of the camera
p.resetDebugVisualizerCamera(cameraDistance=12, cameraYaw=-180, cameraPitch=-120, cameraTargetPosition=[0, 0, 0])


def moveRobotToTarget(robot_id, target_location):

    # calculate new orientation and reset the robots
    robot_pos, robot_orn = p.getBasePositionAndOrientation(robot_id)
    euler_orn = p.getEulerFromQuaternion(robot_orn)
    print(euler_orn)
    target_orn = np.arctan2(target_location[0], target_location[1])
    print(target_orn)
    quaternion = p.getQuaternionFromEuler([euler_orn[0], euler_orn[1], euler_orn[2] + target_orn])
    p.resetBasePositionAndOrientation(robot_id, robot_pos, quaternion)

    # set joint controls
    numJoints = p.getNumJoints(robot_id)
    for joint in range(numJoints):
        print(p.getJointInfo(robot_id, joint))
    targetVel = 10  # rad/s
    maxForce = 100  # Newton
    for joint in range(2, 6):
        p.setJointMotorControl(husky_1, joint, p.VELOCITY_CONTROL, targetVel, maxForce)
        p.setJointMotorControl(husky_2, joint, p.VELOCITY_CONTROL, -targetVel, maxForce)

    # check collision
    F_L_Wheel_pos = p.getJointInfo(robot_id, 2)[14]
    F_R_Wheel_pos = p.getJointInfo(robot_id, 3)[14]
    F_L_pos = tuple(map(operator.add, F_L_Wheel_pos, robot_pos))
    F_R_pos = tuple(map(operator.add, F_R_Wheel_pos, robot_pos))
    target_L_pos = tuple(map(operator.add, F_L_Wheel_pos, target_location))
    target_R_pos = tuple(map(operator.add, F_R_Wheel_pos, target_location))

    collision_obj = p.rayTestBatch([F_L_pos, F_R_pos], [target_L_pos, target_R_pos])

    if collision_obj[0][0] == -1:
        print("No collision")
    else:
        print("Collision")

    dist = 100
    count = 0
    while dist > 0.05:
        count += 1
        # find Euclidean distance to target position
        current_pos, _ = p.getBasePositionAndOrientation(robot_id)
        dist = np.sqrt(pow((target_location[0] - current_pos[0]), 2) + pow((target_location[1] - current_pos[1]), 2))
        p.stepSimulation()
        # print(count)


camera_image = p.getCameraImage(5, 5)

success = moveRobotToTarget(husky_1, [10, 10, 0.1])

print("Robot moved")
# --------------------------TEST CODE-----------------------------------------
# numJoints = p.getNumJoints(husky_1)
# for joint in range(numJoints):
#     print(p.getJointInfo(husky_1, joint))
# targetVel = 10  # rad/s
# maxForce = 100  # Newton
#
# for joint in range(2, 6):
#     p.setJointMotorControl(husky_1, joint, p.VELOCITY_CONTROL, targetVel, maxForce)
#     p.setJointMotorControl(husky_2, joint, p.VELOCITY_CONTROL, -targetVel, maxForce)
#
# carPos, carOrn = p.getBasePositionAndOrientation(husky_1)
# print("-----------------" + str(carOrn))
# p.resetBasePositionAndOrientation(husky_1, carPos, [0, 0, 1, 1])
# for step in range(3000):
#     p.stepSimulation()
#
# targetVel = -10
# for joint in range(2, 6):
#     p.setJointMotorControl(husky_1, joint, p.VELOCITY_CONTROL, targetVel, maxForce)
#     p.setJointMotorControl(husky_2, joint, p.VELOCITY_CONTROL, -targetVel, maxForce)
# # for step in range(3000):
# while (True):
#     p.stepSimulation()

p.getContactPoints(husky_1)
p.disconnect()
