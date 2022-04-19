from struct import pack
import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.81)

# Load Models
planeId = p.loadURDF("plane.urdf")

table_a = p.loadURDF("table/table.urdf",[0,0,0])
tray_a = p.loadURDF("tray/traybox.urdf",[0.5,0.7,0.65])
tray_b = p.loadURDF("tray/traybox.urdf",[-0.5,-0.7,0.65])
arm_b = p.loadURDF("franka_panda/panda.urdf",[-0.6,0,0.65], p.getQuaternionFromEuler([0,0,0]), useFixedBase=1)
arm_b  = p.loadURDF("franka_panda/panda.urdf",[0.6,0,0.65], p.getQuaternionFromEuler([0,0,3]), useFixedBase=1)

#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(arm)
print(cubePos,cubeOrn)
p.disconnect()
