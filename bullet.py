from struct import pack
from turtle import position
import pybullet as p
import time
import pybullet_data

def stepSim(steps):
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(0.01)

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.81)

# Load Models [x, y ,z]
planeId = p.loadURDF("plane.urdf")
p.loadURDF("lego/lego.urdf",[-0.49,-0.75,0.7])

table_a = p.loadURDF("table/table.urdf",[0,0,0])
table_b = p.loadURDF("tablee/table_square.urdf",[0,0,0])
tray_a = p.loadURDF("tray/traybox.urdf",[0.5,0.7,0.68])
tray_b = p.loadURDF("tray/traybox.urdf",[-0.5,-0.7,0.68])
arm_a = p.loadURDF("franka_panda/panda.urdf",[-0.55,0,0.65], p.getQuaternionFromEuler([0,0,0]), useFixedBase=1)
arm_b  = p.loadURDF("franka_panda/panda.urdf",[0.55,0,0.65], p.getQuaternionFromEuler([0,0,3]), useFixedBase=1)

joint_nums = p.getNumJoints(arm_a)
# position, orientation = p.getBasePositionAndOrientation(arm_a)
# p.getJointInfo(arm_a, 7)
# E2 ~ H
#              B E1  3   E2  5    H    W  X  X  R  L  
prepareGrab = [0, 1, 0,-1.3, 0, 2.3, 0.8]

box_6 = [0, 1.2, 0,-0.9, 0, 2.1, 0.8]

pickedUp     = [-1.5, 1, 0,-0.8, 0, 1.8, 0.8]
base_p1     = [-1.0, 1, 0,-0.8, 0, 1.8, 0.8]
base_p2     = [-0.5, 1, 0,-0.8, 0, 1.8, 0.8]
main_base    = [0, 1, 0,-0.8, 0, 1.8, 0.8]
trayPrep    = [-1.5, 1, 0,-1.2, 0, 2.2, 0.8]
trayGrab    = [-1.5, 1, 0,-1.25, 0, 2.25, 0.8]

steps = 100
p.setRealTimeSimulation(0)
#p.setJointMotorControl2(arm_a, 1, p.POSITION_CONTROL, targetPosition=0.5)

p.setJointMotorControl2(arm_a, 10, p.VELOCITY_CONTROL,targetVelocity=2)
p.setJointMotorControl2(arm_a, 11, p.VELOCITY_CONTROL,targetVelocity=2)
stepSim(steps)

p.setJointMotorControlArray(arm_a, range(7), p.POSITION_CONTROL, targetPositions=[1]*7)
p.setJointMotorControlArray(arm_a, range(joint_nums-5), p.POSITION_CONTROL,
targetPositions=trayPrep)

p.setJointMotorControlArray(arm_b, range(7), p.POSITION_CONTROL,
targetPositions=[1]*7)

stepSim(steps)


p.setJointMotorControl2(arm_a, 10, p.VELOCITY_CONTROL,targetVelocity=-2, force=2)
p.setJointMotorControl2(arm_a, 11, p.VELOCITY_CONTROL,targetVelocity=-2, force=2)

stepSim(steps)

p.setJointMotorControlArray(arm_a, range(joint_nums-5), p.POSITION_CONTROL,
targetPositions=pickedUp)
stepSim(steps)

p.setJointMotorControlArray(arm_a, range(joint_nums-5), p.POSITION_CONTROL,
targetPositions=base_p1)
stepSim(steps)

p.setJointMotorControlArray(arm_a, range(joint_nums-5), p.POSITION_CONTROL,
targetPositions=base_p2)
stepSim(steps)

p.setJointMotorControlArray(arm_a, range(joint_nums-5), p.POSITION_CONTROL,
targetPositions=main_base)
stepSim(steps)

p.setJointMotorControlArray(arm_a, range(joint_nums-5), p.POSITION_CONTROL,
targetPositions=box_6)
stepSim(steps)

cubePos, cubeOrn = p.getBasePositionAndOrientation(arm_a)
print(cubePos,cubeOrn)

while True:
    p.stepSimulation()
    time.sleep(0.01)

p.disconnect()

