import pybullet as p
import numpy as np
import operator
import math


# creating the environment
# p.connect(p.GUI)
#
# p.loadURDF("../data/plane.urdf")
# p.setGravity(0, 0, -10)
# p.setTimeStep(0.009)

# adding static obstacles to the environment (uncomment for obstacles)
# obs_1 = p.loadURDF("../data/obstacle.urdf", [0, 10, 0], useFixedBase=1)
# obs_2 = p.loadURDF("../data/obstacle.urdf", [-5, -10, 0], useFixedBase=1)
# obs_3 = p.loadURDF("../data/obstacle.urdf", [5, -10, 0], useFixedBase=1)
# obs_4 = p.loadURDF("../data/obstacle.urdf", [-5, 0, 0], useFixedBase=1)
# obs_5 = p.loadURDF("../data/obstacle.urdf", [5, 0, 0], useFixedBase=1)

# initializing the robot
# husky_start_pos = [-12, -12, 0.1]
# husky_goal_pos = [10, 10, 0.1]
# husky_1 = p.loadURDF("../data/husky/husky.urdf", husky_start_pos)
#
# # Adjust the position of the camera
# p.resetDebugVisualizerCamera(cameraDistance=20, cameraYaw=-180, cameraPitch=-120, cameraTargetPosition=[0, 0, 0])


# def setJointControlsOfHusky(robot_id):
#     # set joint controls
#     # uncomment to get info about all joints
#     # numJoints = p.getNumJoints(robot_id)
#     # for joint in range(numJoints):
#     #     print(p.getJointInfo(robot_id, joint))
#
#     target_vel = 10  # rad/s
#     max_force = 100  # Newton
#     for joint in range(2, 6):
#         p.setJointMotorControl(husky_1, joint, p.VELOCITY_CONTROL, target_vel, max_force)
#
#
# def rotateHuskyToFaceTarget(robot_id, target_location):
#     # calculate new orientation and reset the robots
#     robot_pos, robot_orn = p.getBasePositionAndOrientation(robot_id)
#     (roll, pitch, theta) = p.getEulerFromQuaternion(robot_orn)
#     # calculate how much the robot has to rotate to face the target
#     angle_to_goal = np.arctan2(target_location[1] - robot_pos[1], target_location[0] - robot_pos[0])
#     # restrict angle to (-pi,pi)
#     angle_to_goal = ((angle_to_goal + np.pi) % (2.0 * np.pi)) - np.pi
#
#     quaternion = p.getQuaternionFromEuler([roll, pitch, angle_to_goal])
#     p.resetBasePositionAndOrientation(robot_id, robot_pos, quaternion)
#
#
# def checkCollision(robot_id, target_location):
#     # shoot rays from both the front wheel to target to check collision
#     robot_pos, robot_orn = p.getBasePositionAndOrientation(robot_id)
#     f_l_rel_pos = p.getJointInfo(robot_id, 2)[14]
#     f_r_rel_pos = p.getJointInfo(robot_id, 3)[14]
#     f_l_pos = tuple(map(operator.add, f_l_rel_pos, robot_pos))
#     f_r_pos = tuple(map(operator.add, f_r_rel_pos, robot_pos))
#     target_f_l_pos = tuple(map(operator.add, f_l_rel_pos, target_location))
#     target_f_r_pos = tuple(map(operator.add, f_r_rel_pos, target_location))
#
#     # shoot rays
#     collision_obj = p.rayTestBatch([f_l_pos, f_r_pos], [target_f_l_pos, target_f_r_pos])
#
#     # check if object ids are detected
#     if collision_obj[0][0] == -1:
#         print("No collision")
#         return False
#     else:
#         print("Collision")
#         return True
#
#
# def moveRobotInternal(robot_id, target_location):
#     dist = 100
#     count = 0
#     while dist > 0.1:
#         count += 1
#         # find Euclidean distance to target position
#         current_pos, _ = p.getBasePositionAndOrientation(robot_id)
#         dist = np.sqrt(pow((target_location[0] - current_pos[0]), 2) + pow((target_location[1] - current_pos[1]), 2))
#         p.stepSimulation()
#         # print(count)
#
#
# def moveRobotToTarget(robot_id, target_location):
#     rotateHuskyToFaceTarget(robot_id, target_location)
#     # check collision should be called after rotating the husky
#     collision = checkCollision(robot_id, target_location)
#     if not collision:
#         setJointControlsOfHusky(robot_id)
#         moveRobotInternal(robot_id, target_location)
#         return True
#     else:
#         # ToDO: Reset husky to original orientation?
#         print("Robot not moved. Path is in collision with obstacles")
#         return False
#
#
# success = moveRobotToTarget(husky_1, [10, 10, 0.1])
# success = moveRobotToTarget(husky_1, [10, -10, 0.1])
# success = moveRobotToTarget(husky_1, [-10, -10, 0.1])
# success = moveRobotToTarget(husky_1, [-10, 10, 0.1])
# print("Robot moved")
# p.disconnect()

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class RRTStar:

    def __init__(self, start_pos, goal_pos, step_len,
                 goal_sample_rate, search_radius, iter_max):

        self.start_pos = Node(start_pos[0], start_pos[1])
        self.goal_pos = Node(goal_pos[0], goal_pos[1])
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.iter_max = iter_max
        self.vertex = [self.start_pos]
        self.path = []

        # environment
        self.x_range = (-15, 15)
        self.y_range = (-15, 15)
        self.robot_id = -1
        self.obstacles = []
        self.env = None

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.env.disconnect()

    def setup_environment(self, obstacles=True):

        # creating the environment
        p.connect(p.GUI)
        p.loadURDF("../data/plane.urdf")
        p.setGravity(0, 0, -10)
        p.setTimeStep(0.004)

        if obstacles:
            self.obstacles.append(p.loadURDF("../data/obstacle.urdf", [0, 10, 0], useFixedBase=1))
            self.obstacles.append(p.loadURDF("../data/obstacle.urdf", [-5, -10, 0], useFixedBase=1))
            self.obstacles.append(p.loadURDF("../data/obstacle.urdf", [5, -10, 0], useFixedBase=1))
            self.obstacles.append(p.loadURDF("../data/obstacle.urdf", [-5, 0, 0], useFixedBase=1))
            self.obstacles.append(p.loadURDF("../data/obstacle.urdf", [5, 0, 0], useFixedBase=1))

        # initializing the robot
        husky_start_pos = [self.start_pos.x, self.start_pos.y, 0.1]
        self.robot_id = p.loadURDF("../data/husky/husky.urdf", husky_start_pos)

        # Adjust the position of the camera
        p.resetDebugVisualizerCamera(cameraDistance=20, cameraYaw=-180, cameraPitch=-120,
                                     cameraTargetPosition=[0, 0, 0])
        self.env = p

    def generate_random_node(self):

        if np.random.random() > self.goal_sample_rate:
            return Node(np.random.uniform(self.x_range[0], self.x_range[1]),
                        np.random.uniform(self.y_range[0], self.y_range[1]))
        else:
            return self.goal_pos

    def nearest_neighbor(self, rand_node):
        return self.vertex[int(np.argmin([math.hypot(nd.x - rand_node.x, nd.y - rand_node.y)
                                          for nd in self.vertex]))]

    def get_distance_and_angle(self, node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    def new_state(self, node_start, node_goal):
        dist, theta = self.get_distance_and_angle(node_start, node_goal)

        dist = min(self.step_len, dist)
        node_new = Node(node_start.x + (dist * math.cos(theta)),
                        node_start.y + (dist * math.sin(theta)))

        node_new.parent = node_start

        return node_new

    # returns a list of indexes of vertex that are in the search radius and has no collision to node_new
    def find_near_neighbor(self, node_new):
        n = len(self.vertex) + 1
        # ToDo: figure out equation for r
        r = min(self.search_radius * math.sqrt((math.log(n) / n)), self.step_len)

        dist_table = [math.hypot(nd.x - node_new.x, nd.y - node_new.y) for nd in self.vertex]
        dist_table_index = [ind for ind in range(len(dist_table)) if dist_table[ind] <= r and
                            not self.check_collision(node_new, self.vertex[ind])]

        return dist_table_index

    def check_collision(self, node_nearest, node_new):
        # shoot rays from both the front wheel to target to check collision
        # robot_pos, robot_orn = p.getBasePositionAndOrientation(robot_id)
        f_l_rel_pos = self.env.getJointInfo(self.robot_id, 2)[14]
        f_r_rel_pos = self.env.getJointInfo(self.robot_id, 3)[14]

        # we can't check for a single ray because the wheels are spread out
        # pad the robot by a small value to avoid going near obstacles
        # f_l_rel_pos = tuple(map(operator.mul, f_l_rel_pos, [2, 2, 0.0]))
        # f_r_rel_pos = tuple(map(operator.mul, f_r_rel_pos, [2, 2, 0.0]))

        f_l_pos = tuple(map(operator.add, f_l_rel_pos, [node_nearest.x, node_nearest.y, 0.0]))
        f_r_pos = tuple(map(operator.add, f_r_rel_pos, [node_nearest.x, node_nearest.y, 0.0]))
        target_f_l_pos = tuple(map(operator.add, f_l_rel_pos, [node_new.x, node_new.y, 0.0]))
        target_f_r_pos = tuple(map(operator.add, f_r_rel_pos, [node_new.x, node_new.y, 0.0]))

        # shoot rays
        collision_obj = self.env.rayTestBatch([f_l_pos, f_r_pos], [target_f_l_pos, target_f_r_pos])

        # check if object ids are detected
        if collision_obj[0][0] == -1:
            # print("No collision")
            return False
        else:
            # print("Collision")
            return True

    # calculate the cost from the root to the node_p by traversing its parents
    def cost_from_root(self, node_p):
        node = node_p
        cost = 0.0

        while node.parent:
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent

        return cost

    def get_new_cost(self, node_start, node_end):
        dist, _ = self.get_distance_and_angle(node_start, node_end)
        return self.cost_from_root(node_start) + dist

    # check is node_new can be reached from any other neighbour within search index with a lower cost
    # and update the parent
    def choose_parent(self, node_new, neighbor_index):
        cost = [self.get_new_cost(self.vertex[i], node_new) for i in neighbor_index]
        cost_min_index = neighbor_index[int(np.argmin(cost))]
        node_new.parent = self.vertex[cost_min_index]

    # if any of the neighbour can be reached from node_new with a lower cost, then update the
    # parent of the neighbour
    def rewire(self, node_new, neighbor_index):
        for i in neighbor_index:
            node_neighbor = self.vertex[i]

            if self.cost_from_root(node_neighbor) > self.get_new_cost(node_new, node_neighbor):
                node_neighbor.parent = node_new

    # searches for the closest vertex from the existing list of vertex whose distance is
    # less than the step_len to the goal and set this vertex as the parent of the goal
    def search_goal_parent(self):
        dist_list = [math.hypot(n.x - self.goal_pos.x, n.y - self.goal_pos.y) for n in self.vertex]
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.step_len]

        if len(node_index) > 0:
            cost_list = [dist_list[i] + self.cost_from_root(self.vertex[i]) for i in node_index
                         if not self.check_collision(self.vertex[i], self.goal_pos)]
            return node_index[int(np.argmin(cost_list))]

        # ToDo: What to do if no vertex is near to the goal vertex?
        return len(self.vertex) - 1

    # traverse back to the root to find the path to the goal
    def extract_path(self, node_end):
        path = [[self.goal_pos.x, self.goal_pos.y]]
        node = node_end

        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def rotate_husky_to_face_target(self, target_location):
        # calculate new orientation and reset the robots
        robot_pos, robot_orn = self.env.getBasePositionAndOrientation(self.robot_id)
        (roll, pitch, theta) = self.env.getEulerFromQuaternion(robot_orn)
        # calculate how much the robot has to rotate to face the target
        angle_to_goal = np.arctan2(target_location[1] - robot_pos[1], target_location[0] - robot_pos[0])
        # restrict angle to (-pi,pi)
        angle_to_goal = ((angle_to_goal + np.pi) % (2.0 * np.pi)) - np.pi

        quaternion = self.env.getQuaternionFromEuler([roll, pitch, angle_to_goal])
        p.resetBasePositionAndOrientation(self.robot_id, robot_pos, quaternion)

    def set_joint_controls_of_husky(self):
        # set joint controls
        # uncomment to get info about all joints
        # numJoints = p.getNumJoints(robot_id)
        # for joint in range(numJoints):
        #     print(p.getJointInfo(robot_id, joint))

        target_vel = 10  # rad/s
        max_force = 100  # Newton
        for joint in range(2, 6):
            self.env.setJointMotorControl(self.robot_id, joint, p.VELOCITY_CONTROL, target_vel, max_force)

    def move_robot_internal(self, target_location):
        dist = 100
        count = 0
        while dist > 0.5:
            count += 1
            # find Euclidean distance to target position
            current_pos, _ = self.env.getBasePositionAndOrientation(self.robot_id)
            dist = np.sqrt(pow((target_location[0] - current_pos[0]), 2) +
                           pow((target_location[1] - current_pos[1]), 2))
            self.env.stepSimulation()

    def move_robot_to_target(self, target_location):
        self.rotate_husky_to_face_target(target_location)
        # check collision should be called after rotating the husky
        # collision = checkCollision(self.robot_id, target_location)
        # if not collision:
        #     setJointControlsOfHusky(self.robot_id)
        #     moveRobotInternal(robot_id, target_location)
        #     return True
        # else:
        #     # ToDO: Reset husky to original orientation?
        #     print("Robot not moved. Path is in collision with obstacles")
        #     return False
        self.set_joint_controls_of_husky()
        self.move_robot_internal(target_location)

    def move_robot(self):
        for position in reversed(self.path):
            self.move_robot_to_target(position)

    def draw_path(self):

        path_3d = [node + [0.1] for node in self.path]
        for i in range(1, len(self.path)):
            self.env.addUserDebugLine(path_3d[i - 1],
                                      path_3d[i],
                                      [0, 0, 0])

    def planning(self):
        for k in range(self.iter_max):

            if k % 100 == 0:
                print("Iteration: " + str(k))

            node_rand = self.generate_random_node()
            node_nearest = self.nearest_neighbor(node_rand)
            node_new = self.new_state(node_nearest, node_rand)

            if node_new and not self.check_collision(node_nearest, node_new):
                neighbor_index = self.find_near_neighbor(node_new)
                self.vertex.append(node_new)

                # check is node_new can be reached from any other neighbour within search index with a lower cost
                if neighbor_index:
                    self.choose_parent(node_new, neighbor_index)
                    self.rewire(node_new, neighbor_index)

        index = self.search_goal_parent()
        self.path = self.extract_path(self.vertex[index])
        # self.move_robot()
        return self.path


rrt_star = RRTStar(start_pos=[-12, -12],
                   goal_pos=[12, 12],
                   step_len=4,
                   goal_sample_rate=0.002,
                   search_radius=5,
                   iter_max=2000)

rrt_star.setup_environment(True)
path = rrt_star.planning()
rrt_star.draw_path()
print(path)
rrt_star.move_robot()
print("The end!")
