from struct import pack
from turtle import position
import pybullet as p
import time
import pybullet_data

def stepSim(steps):
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(0.01)

def grab(arm):
    p.setJointMotorControl2(arm, 10, p.POSITION_CONTROL,targetPosition=0.02, force=50)
    p.setJointMotorControl2(arm, 9, p.POSITION_CONTROL,targetPosition=0.02, force=50)

def release(arm):
    p.setJointMotorControl2(arm, 10, p.POSITION_CONTROL,targetPosition=1.5)
    p.setJointMotorControl2(arm, 9, p.POSITION_CONTROL,targetPosition=1.5)

def resetPos(arm):
    startingPos = [-1.5, 1, 0, 0, 0, 1.8,0.8, 0, 0, 1, 1]
    p.setJointMotorControlArray(arm, range(11), p.POSITION_CONTROL,targetPositions=startingPos)

def move_arm(arm, pos):
    p.setJointMotorControlArray(arm, range(7), p.POSITION_CONTROL,targetPositions=pos)

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.81)

# Load Models [x, y ,z]
planeId = p.loadURDF("plane.urdf")
#p.loadURDF("cube_small.urdf",[-0.49,-0.76,0.72])
o_cube = p.loadURDF("cube_small.urdf",[-0.49,-0.76,0.73])
x_cube = p.loadURDF("cube_small.urdf",[-0.49,-0.90,0.73])
x_cube = p.loadURDF("cube_small.urdf",[-0.49,-0.83,0.73])
x_cube = p.loadURDF("cube_small.urdf",[-0.49,-0.69,0.73])
x_cube = p.loadURDF("cube_small.urdf",[-0.49,-0.62,0.73])
o_text = p.loadTexture("play_cubes/o_cube.png")
x_text = p.loadTexture("play_cubes/x_cube.png")

p.changeVisualShape(o_cube, -1, textureUniqueId=o_text)
p.changeVisualShape(x_cube, -1, textureUniqueId=x_text)

table_a = p.loadURDF("table/table.urdf",[0,0,0])
table_b = p.loadURDF("tic_tac_toe_board/table_square.urdf",[0,0,0])
tray_a = p.loadURDF("tray/traybox.urdf",[0.5,0.75,0.69])
tray_b = p.loadURDF("tray/traybox.urdf",[-0.5,-0.75,0.69])
arm_a = p.loadURDF("franka_panda/panda.urdf",[-0.55,0,0.65], p.getQuaternionFromEuler([0,0,0]), useFixedBase=1)
arm_b  = p.loadURDF("franka_panda/panda.urdf",[0.55,0,0.65], p.getQuaternionFromEuler([0,0,3]), useFixedBase=1)

joint_nums = p.getNumJoints(arm_a)
# position, orientation = p.getBasePositionAndOrientation(arm_a)
# p.getJointInfo(arm_a, 7)
# E2 ~ H
#                  B E1  3 E2  5    H   W  X  X  R  L  
#startingPos = [-1.5, 1, 0, 0, 0, 1.8,0.8, 0, 0, 1, 1]

box_6 = [0, 1.1, 0,-1.15, 0, 2.25, 0.8]
# pere = p.calculateInverseKinematics(arm_a, 7, [0.1,0.1,0.4])
trayPrep    = [-1.5, 1, 0,-1.2, 0, 2.2, 0.8]
pickedUp     = [-1.5, 1, 0,-0.8, 0, 1.8, 0.8]
base_p1     = [-1.0, 1, 0,-0.8, 0, 1.8, 0.8]
base_p2     = [-0.5, 1, 0,-0.8, 0, 1.8, 0.8]
main_base    = [0, 1, 0,-0.8, 0, 1.8, 0.8]

quick = 60
slow = 100
p.setRealTimeSimulation(0)
# Calculate and move arm up after pick up
#p.setJointMotorControl2(arm_a, 1, p.POSITION_CONTROL, targetPosition=0.5)

p.setJointMotorControlArray(arm_b, range(joint_nums), p.POSITION_CONTROL,
targetPositions=[1]*joint_nums)
p.setJointMotorControl2(arm_b, 3, p.POSITION_CONTROL, targetPosition=-2)

resetPos(arm_a)
stepSim(quick)
move_arm(arm_a, trayPrep)

stepSim(quick)

grab(arm_a)
stepSim(quick)

move_arm(arm_a, pickedUp)
stepSim(slow)

move_arm(arm_a, base_p1)
stepSim(slow)

move_arm(arm_a, base_p2)
stepSim(slow)

move_arm(arm_a, main_base)
stepSim(slow)

move_arm(arm_a, box_6)
stepSim(slow)

release(arm_a)
stepSim(quick)

move_arm(arm_a, main_base)
stepSim(quick)

resetPos(arm_a)
stepSim(quick)

p.getCameraImage(400,400,renderer=p.ER_TINY_RENDERER)

cubePos, cubeOrn = p.getBasePositionAndOrientation(arm_a)
print(cubePos,cubeOrn)

while True:
    p.stepSimulation()
    time.sleep(0.01)

p.disconnect()

