from struct import pack
from turtle import position
import pybullet as p
import time
import pybullet_data

<<<<<<< Updated upstream
=======
from PIL import Image
import PIL
import cv2
import numpy as np

# Variables ----------------------------------------------------------------
#                  B E1  3 E2  5    H   W  X  X  R  L  
#startingPos = [-1.5, 1, 0, 0, 0, 1.8,0.8, 0, 0, 1, 1]
x_cubes = [ [-1.5, 1.66, 0,0, 0, 1.85, 0.8],        # 1st Cube
            [-1.5, 1, 0,-1.2, 0, 2.2, 0.8],         # 2nd Cube
            [-1.49, 0.7, 0,-1.72, 0, 2.47, 0.8],    # 3rd Cube
            [-1.48, 0.55, 0,-2.01, 0, 2.6, 0.8],    # 4th Cube
            [-1.48, 0.4, 0,-2.27, 0, 2.7, 0.8]]     # 5th Cube
pickedUp     = [-1.5, 0.8, 0,-1.2, 0, 2.5, 0.8]
base_p1     = [-1.0, 0.8, 0,-1.2, 0, 2, 0.6]
base_p2     = [-0.5, 0.6, 0,-1.6, 0, 2.2, 0.3]
base_p3     = [-0, 0.4, 0,-2, 0, 2.4, 0.8]
mid_base    = [-0.8, 0.6, 0,-1.5, 0, 2.2, 0.3]
main_base    = [0, 0.3, 0,-2.2, 0, 2.5, 0.8]

boxes = [[0.51, 0, 0,-2.8, 0, 2.8, 1.1],    # Box 1
        [0.40, 0.5, 0,-2.2, 0, 2.8, 1.2],   # Box 2
        [0.28, 1.0, 0,-1.0, 0, 2.0, 1.0],   # Box 3
        [0, 0, 0,-2.95, 0, 3, 0.8],         # Box 4
        [0, 0.5, 0,-2.3, 0, 3, 0.8],        # Box 5
        [0, 1.0, 0,-1.2, 0, 2.25, 0.8],     # Box 6
        [-0.51, 0, 0,-2.8, 0, 2.8, 0.4],    # Box 7
        [-0.40, 0.5, 0,-2.2, 0, 2.8, 0.4],  # Box 8
        [-0.28, 1.0, 0,-1.0, 0, 2.0, 0.6]]  # Box 9

quick = 60
slow = 90
x_picked = 0
o_picked = 0

# Functions ----------------------------------------------------------------

# Step simulation
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream

def move_arm(arm, pos):
    p.setJointMotorControlArray(arm, range(7), p.POSITION_CONTROL,targetPositions=pos)
=======
    stepSim(quick)

def move_arm(arm, pos, speed):
    p.setJointMotorControlArray(arm, range(7), p.POSITION_CONTROL,targetPositions=pos,forces=[100]*7)
    stepSim(speed)

# Load all cubes and textures
def load_cubes():
    incr = 0
    x_cube = [0]*5
    o_cube = [0]*5
    x_text = p.loadTexture("play_cubes/x_cube.png")
    o_text = p.loadTexture("play_cubes/o_cube.png")

    for i in range(5):
        x_cube[i] = p.loadURDF("cube_small.urdf",[-0.49,-0.54 - incr,0.73])
        o_cube[i] = p.loadURDF("cube_small.urdf",[0.6,0.49 + incr,0.73])
        incr += 0.07
        p.changeVisualShape(o_cube[i], -1, textureUniqueId=o_text)
        p.changeVisualShape(x_cube[i], -1, textureUniqueId=x_text)

# Pick up the next available cube
def pick_up_cube(sign):
    global x_picked
    global o_picked

    if sign == 0:
        move_arm(arm_a, x_cubes[x_picked%5], quick)
        grab(arm_a)
        move_arm(arm_a, pickedUp, slow)
        x_picked += 1
    else:
        move_arm(arm_b, x_cubes[o_picked%5], quick)
        grab(arm_b)
        move_arm(arm_b, pickedUp, slow)
        o_picked += 1

# Place cubes into selected box
def place_cube(sign, box_num):
    pick_up_cube(sign)

    if sign == 0:
        arm = arm_a
    else:
        arm = arm_b

    move_arm(arm, mid_base,slow)
    move_arm(arm, main_base, slow)
    move_arm(arm, boxes[box_num-1], slow)
    release(arm)
    move_arm(arm, main_base, quick)
    resetPos(arm)

# Main ---------------------------------------------------------------------
>>>>>>> Stashed changes

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
<<<<<<< Updated upstream
table_b = p.loadURDF("tic_tac_toe_board/table_square.urdf",[0,0,0])
tray_a = p.loadURDF("tray/traybox.urdf",[0.5,0.75,0.69])
tray_b = p.loadURDF("tray/traybox.urdf",[-0.5,-0.75,0.69])
=======
table_b = p.loadURDF("tic_tac_toe_board/table_square.urdf",[0,0.005,0])
tray_a = p.loadURDF("tray/traybox.urdf",[0.5,0.65,0.68])
tray_b = p.loadURDF("tray/traybox.urdf",[-0.6,-0.68,0.68])
>>>>>>> Stashed changes
arm_a = p.loadURDF("franka_panda/panda.urdf",[-0.55,0,0.65], p.getQuaternionFromEuler([0,0,0]), useFixedBase=1)
arm_b  = p.loadURDF("franka_panda/panda.urdf",[0.55,-0.05,0.65], p.getQuaternionFromEuler([0,0,3]), useFixedBase=1)

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

<<<<<<< Updated upstream
p.setJointMotorControlArray(arm_b, range(joint_nums), p.POSITION_CONTROL,
targetPositions=[1]*joint_nums)
p.setJointMotorControl2(arm_b, 3, p.POSITION_CONTROL, targetPosition=-2)

resetPos(arm_a)
stepSim(quick)
move_arm(arm_a, trayPrep)
=======
viewMatrix = p.computeViewMatrix(
    cameraEyePosition=[0, 0, 1.4],
    cameraTargetPosition=[0, 0, 0],
    cameraUpVector=[0, 1, 0])
>>>>>>> Stashed changes

stepSim(quick)

grab(arm_a)
stepSim(quick)

<<<<<<< Updated upstream
move_arm(arm_a, pickedUp)
stepSim(slow)

move_arm(arm_a, base_p1)
stepSim(slow)
=======
# Controlling Arm ----------------------------------------------------------
resetPos(arm_a)
resetPos(arm_b)

place_cube(0,6)
place_cube(0,5)
place_cube(0,7)

place_cube(1,1)
place_cube(1,4)
place_cube(1,6)

place_cube(0,2)
>>>>>>> Stashed changes

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

