import time
import pybullet as p2
import pybullet_data
import pybullet_utils.bullet_client as bc
import os

print(f'isfile: {os.path.isfile("./seesaw.urdf")}')

p = bc.BulletClient(connection_mode=p2.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # turn GUI off
p.resetSimulation()

board = p.loadURDF("./seesaw.urdf", [0, 0, 0], useFixedBase=1)
# ball = p.loadURDF("./ball.urdf", [0.1, 0, 0])
p.resetDebugVisualizerCamera(cameraDistance=.6, cameraYaw=180, cameraPitch=-50, cameraTargetPosition=[0, 0, 0])

p.setGravity(0, 0, -9.8)

for _ in range(1000000):
    p.stepSimulation()
    time.sleep(1. / 240.)
    p.applyExternalForce(board, -1, [0, 0, 100], [0.1, 0, 0], p.LINK_FRAME)
