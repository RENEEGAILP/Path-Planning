from time import sleep

import pybullet as p
import numpy as np
import math
from shapely.geometry import LineString


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class RRT:

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
        self.node_path = []

        # constants
        self.STEP_SIZE = 0.004
        self.BOUNDARY_FACTOR = 1.5

        # robot
        self.robot_id = -1
        self.f_l_wheel_rel_pos = -1
        self.f_r_wheel_rel_pos = -1

        # environment
        self.x_range = (-15, 15)
        self.y_range = (-15, 15)
        self.obstacles = []
        self.env = None
        self.tree_debug_lines = {}

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.env.disconnect()

    def setup_environment(self, obstacles=True):
        # Creating and initializing the environment
        p.connect(p.GUI)
        p.loadURDF("../data/plane.urdf")
        p.setGravity(0, 0, -10)
        p.setTimeStep(self.STEP_SIZE)

        if obstacles:
            self.obstacles.append(p.loadURDF("../data/obstacle.urdf", [-4, -6, 0], useFixedBase=1))
            self.obstacles.append(p.loadURDF("../data/obstacle.urdf", [4, -8, 0], useFixedBase=1))
            self.obstacles.append(p.loadURDF("../data/obstacle.urdf", [-4, 8, 0], useFixedBase=1))
            self.obstacles.append(p.loadURDF("../data/obstacle.urdf", [4, 6, 0], useFixedBase=1))
            self.obstacles.append(p.loadURDF("../data/obstacle_2.urdf", [4, -1, 0], useFixedBase=1))
            self.obstacles.append(p.loadURDF("../data/obstacle_2.urdf", [-10, -8, 0], useFixedBase=1))
            self.obstacles.append(p.loadURDF("../data/obstacle_3.urdf", [10, 1, 0], useFixedBase=1))
            self.obstacles.append(p.loadURDF("../data/obstacle_4.urdf", [-6, 0, 0], useFixedBase=1))

        # initializing the robot
        husky_start_pos = [self.start_pos.x, self.start_pos.y, 0.1]
        self.robot_id \
            = p.loadURDF("../data/husky/husky.urdf", husky_start_pos)
        self.f_l_wheel_rel_pos = p.getJointInfo(self.robot_id, 2)[14]
        self.f_r_wheel_rel_pos = p.getJointInfo(self.robot_id, 3)[14]

        # Adjust the position of the camera
        p.resetDebugVisualizerCamera(cameraDistance=18, cameraYaw=-180, cameraPitch=-91,
                                     cameraTargetPosition=[0, 0, 0])
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        self.env = p

    def generate_random_node(self):
        # returns a random sample node
        if np.random.random() > self.goal_sample_rate:
            return Node(np.random.uniform(self.x_range[0], self.x_range[1]),
                        np.random.uniform(self.y_range[0], self.y_range[1]))
        else:
            return self.goal_pos

    def nearest_neighbor(self, rand_node):
        # Returns the nearest neighbor to rand_node from the list of existing vertices
        return self.vertex[int(np.argmin([math.hypot(nd.x - rand_node.x, nd.y - rand_node.y)
                                          for nd in self.vertex]))]

    def get_distance_and_angle(self, node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    def new_state(self, node_start, node_goal):
        # Returns a node along the path of start and goal which has the shortest possible distance to start
        dist, theta = self.get_distance_and_angle(node_start, node_goal)

        dist = min(self.step_len, dist)
        node_new = Node(node_start.x + (dist * math.cos(theta)),
                        node_start.y + (dist * math.sin(theta)))

        node_new.parent = node_start

        return node_new

    def find_near_neighbor(self, node_new):
        # returns a list of indexes of vertex that are in the search radius and has no collision to node_new
        n = len(self.vertex) + 1
        r = min(self.search_radius * math.sqrt((math.log(n) / n)), self.step_len)

        dist_table = [math.hypot(nd.x - node_new.x, nd.y - node_new.y) for nd in self.vertex]
        dist_table_index = [ind for ind in range(len(dist_table)) if dist_table[ind] <= r and
                            not self.check_collision(node_new, self.vertex[ind])]

        return dist_table_index

    def get_wheel_position(self, wheel_node, theta):
        dist_between_wheels = (self.f_l_wheel_rel_pos[1] - self.f_r_wheel_rel_pos[1]) * self.BOUNDARY_FACTOR
        dist = self.f_l_wheel_rel_pos[0]
        node_new = Node(wheel_node.x + (dist * math.cos(theta)),
                        wheel_node.y + (dist * math.sin(theta)))

        a = (wheel_node.x, wheel_node.y)
        b = (node_new.x, node_new.y)
        ab = LineString([a, b])
        left = ab.parallel_offset(dist_between_wheels / 2, 'left')
        right = ab.parallel_offset(dist_between_wheels / 2, 'right')
        c = left.boundary[1]
        d = right.boundary[0]
        f_l_pos = [c.x, c.y, 0.25]
        f_r_pos = [d.x, d.y, 0.25]
        return [f_l_pos, f_r_pos]

    def check_collision(self, node_start, node_goal, print_debug_lines=False):
        # shoot rays from both the front wheel to target to check collision

        _, theta = self.get_distance_and_angle(node_start, node_goal)
        start_wheel_pos = self.get_wheel_position(node_start, theta)
        target_wheel_pos = self.get_wheel_position(node_goal, theta)

        if print_debug_lines:
            self.env.addUserDebugLine(start_wheel_pos[0],
                                      target_wheel_pos[0],
                                      [1, 0, 0])
            self.env.addUserDebugLine(start_wheel_pos[1],
                                      target_wheel_pos[1],
                                      [1, 0, 0])

        # shoot rays
        collision_f_l = self.env.rayTest(start_wheel_pos[0], target_wheel_pos[0])
        collision_f_r = self.env.rayTest(start_wheel_pos[1], target_wheel_pos[1])

        # check if object ids are detected
        if collision_f_l[0][0] == -1 and collision_f_r[0][0] == -1:
            # print("No collision")
            return False
        else:
            # print("Collision")
            return True

    def cost_from_root(self, node_p):
        # Returns the cost from the root to the node_p by traversing its parents
        node = node_p
        cost = 0.0
        while node.parent:
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent
        return cost

    def get_new_cost(self, node_start, node_end):
        # Returns cost from root to node_end
        dist, _ = self.get_distance_and_angle(node_start, node_end)
        return self.cost_from_root(node_start) + dist

    def search_goal_parent(self):
        # searches for the closest vertex from the existing list of vertex whose distance is
        # less than the step_len to the goal and set this vertex as the parent of the goal
        dist_list = [math.hypot(n.x - self.goal_pos.x, n.y - self.goal_pos.y) for n in self.vertex]
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.step_len]

        if len(node_index) > 0:
            cost_list = [dist_list[i] + self.cost_from_root(self.vertex[i]) for i in node_index
                         if not self.check_collision(self.vertex[i], self.goal_pos)]
            return node_index[int(np.argmin(cost_list))]

        # ToDo: What to do if no vertex is near to the goal vertex?
        return len(self.vertex) - 1

    def extract_path(self, node_end):
        # traverse back to the root to find the path to the goal
        planned_path = [[self.goal_pos.x, self.goal_pos.y]]
        node = node_end

        while node.parent is not None:
            planned_path.append([node.x, node.y])
            self.node_path.append(node)
            node = node.parent
        planned_path.append([node.x, node.y])
        self.node_path.append(node)

        return planned_path

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
        while dist > 0.2:
            count += 1
            # find Euclidean distance to target position
            current_pos, _ = self.env.getBasePositionAndOrientation(self.robot_id)
            dist = np.sqrt(pow((target_location[0] - current_pos[0]), 2) +
                           pow((target_location[1] - current_pos[1]), 2))
            self.env.stepSimulation()

    def check_collision_from_current_pos(self, node_goal):

        robot_pos, robot_orn = self.env.getBasePositionAndOrientation(self.robot_id)
        node_start = Node(robot_pos[0], robot_pos[1])
        node_goal = Node(node_goal[0], node_goal[1])
        return self.check_collision(node_start, node_goal, True)

    def move_robot_to_target(self, target_location):
        self.rotate_husky_to_face_target(target_location)
        # check collision should be called after rotating the husky
        collision = self.check_collision_from_current_pos(target_location)
        if collision:
            # ToDO: Reset husky to original orientation?
            print("Robot not moved. Path is in collision with obstacles")
            return False
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
                                      [0, 0, 0],
                                      5)

    def planning(self):
        for k in range(self.iter_max):

            if k % 100 == 0:
                print("Iteration: " + str(k))

            node_rand = self.generate_random_node()
            node_nearest = self.nearest_neighbor(node_rand)
            node_new = self.new_state(node_nearest, node_rand)

            if node_new and not self.check_collision(node_nearest, node_new):
                self.vertex.append(node_new)
                self.tree_debug_lines[((node_new.x, node_new.y), (node_new.parent.x, node_new.parent.y))] = \
                    self.env.addUserDebugLine([node_new.x, node_new.y, 0.1],
                                              [node_new.parent.x, node_new.parent.y, 0.1],
                                              [0, 0, 1])

        index = self.search_goal_parent()
        self.path = self.extract_path(self.vertex[index])
        return self.path


rrt = RRT(start_pos=[-12, 12],
          goal_pos=[12, -12],
          step_len=1,
          goal_sample_rate=0.002,
          search_radius=12,
          iter_max=5000)

rrt.setup_environment(True)
sleep(3)
path = rrt.planning()
#input("Press Enter to continue...")
rrt.draw_path()
print(path)
print(rrt.cost_from_root(rrt.node_path[0]))
input("Press Enter again to continue...")
rrt.move_robot()
input("Press Enter again to exit...")
print("The end!")
