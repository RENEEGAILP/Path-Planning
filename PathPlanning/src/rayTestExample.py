import pybullet as p
import numpy as np
import pybullet_data

p.connect(p.GUI)
p.setGravity(0, 0, -9.8)

p.loadURDF("plane.urdf")
p.loadURDF("obstacle.urdf", [0, 5, 0.1], useFixedBase=1)

p.resetDebugVisualizerCamera(cameraDistance=26, cameraYaw=-180, cameraPitch=-120,
                             cameraTargetPosition=[0, 0, 0])

for i in np.arange(-3.0, 4.0,0.1):
    for j in np.arange(2.0, 9.0,0.1):
        for x in np.arange(-3.0, 4.0,0.1):
            for y in np.arange(2.0, 9.0,0.1):
                if i != x and j != y:
                    res_1 = p.rayTest([i, j, 0.2], [x, y, 0.2])
                    res_2 = p.rayTest([i, j, 0.2], [x, y, 0.2])
                    if res_1[0][0] == -1 and res_2[0][0] == -1:
                        p.addUserDebugLine([i, j, 0.2], [x, y, 0.2], [0, 0, 0])
                        #print("No collision")
                    if (res_1[0][0] != -1 and res_2[0][0] == -1) or (res_1[0][0] == -1 and res_2[0][0] != -1):
                        print("Collision found once only")
                    #if res_1[0][0] != -1 and res_2[0][0] != -1:
                        #print("Collision detected")

while True:
    p.stepSimulation()
