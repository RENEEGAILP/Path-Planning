from shapely.geometry import LineString

a = (10, 20)
b = (15, 30)
cd_length = 6

ab = LineString([a, b])
left = ab.parallel_offset(cd_length / 2, 'left')
right = ab.parallel_offset(cd_length / 2, 'right')
c = left.boundary[1]
d = right.boundary[0]  # note the different orientation for right offset
cd = LineString([c, d])
print(c.y)
print(cd)

# robot_base = [10, 20]
# l_rel = [2, 3]
# r_rel = [2, -3]
# dist = 11.18
def find_wheel_coordinates(robot_base, l_rel, r_rel, theta):
    dist = l_rel[0]
    new_robot_base = Node(node_start.x + (dist * math.cos(theta)),
                    node_start.y + (dist * math.sin(theta)))

