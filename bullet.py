from struct import pack
from turtle import position
import pybullet as p
import time
import pybullet_data

from PIL import Image
import PIL
import cv2
import numpy as np

# Variables ----------------------------------------------------------------
"""
               B  E1  R1  E2  R2    H    W  X  X  R  L  
position = [-1.5,  1,  0,  0,  0, 1.8, 0.8, 0, 0, 1, 1]

B - Base
E1 - Elbow 1
E2 - Elbow 2
R1 - Elbow 1 Rotate
R2 - Elbow 2 Rotate
H - Hand
W - Wrist rotate
R - Right finger
L - Left finger

"""
cubes = [ [-1.5, 1.66, 0,0, 0, 1.85, 0.8],      # 1st Cube
        [-1.5, 1, 0,-1.2, 0, 2.2, 0.8],         # 2nd Cube
        [-1.49, 0.7, 0,-1.72, 0, 2.47, 0.8],    # 3rd Cube
        [-1.48, 0.55, 0,-2.01, 0, 2.6, 0.8],    # 4th Cube
        [-1.48, 0.4, 0,-2.27, 0, 2.7, 0.8]]     # 5th Cube

pickedUp     = [-1.5, 0.8, 0,-1.2, 0, 2.5, 0.8]
mid_base    = [-0.8, 0.6, 0,-1.5, 0, 2.2, 0.3]
main_base    = [0, 0.3, 0,-2.2, 0, 2.5, 0.8]

boxes = [[0.51, 0, 0,-2.8, 0, 2.8, 1.1],    # Box 1
        [0.40, 0.5, 0,-2.2, 0, 2.8, 1.2],   # Box 2
        [0.28, 1.0, 0,-1.0, 0, 2.0, 1.0],   # Box 3
        [0, 0, 0,-2.95, 0, 3, 0.8],         # Box 4
        [0, 0.5, 0,-2.3, 0, 3, 0.8],        # Box 5
        [0, 1.0, 0,-1.2, 0, 2.25, 0.8],     # Box 6
        [-0.51, 0.1, 0,-2.7, 0, 2.8, 0.3],  # Box 7
        [-0.36, 0.5, 0,-2.1, 0, 2.8, 0.4],  # Box 8
        [-0.24, 1.1, 0,-0.9, 0, 2.0, 0.5]]  # Box 9

quick = 60
slow = 80
x_picked = 0
o_picked = 0

# Functions ----------------------------------------------------------------

# Step simulation
def stepSim(steps):
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(0.01)

def grab(arm):
    p.setJointMotorControl2(arm, 10, p.POSITION_CONTROL,targetPosition=0.0185, force=60)
    p.setJointMotorControl2(arm, 9, p.POSITION_CONTROL,targetPosition=0.0185, force=60)
    stepSim(50)

def release(arm):
    p.setJointMotorControl2(arm, 10, p.POSITION_CONTROL,targetPosition=1.5)
    p.setJointMotorControl2(arm, 9, p.POSITION_CONTROL,targetPosition=1.5)
    stepSim(50)

def resetPos(arm):
    startingPos = [-1.5, 1, 0, -0.8, 0, 1.8,0.8, 0, 0, 1, 1]
    p.setJointMotorControlArray(arm, range(11), p.POSITION_CONTROL,targetPositions=startingPos)
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
        o_cube[i] = p.loadURDF("cube_small.urdf",[0.592,0.47 + incr,0.73])
        incr += 0.07
        p.changeVisualShape(o_cube[i], -1, textureUniqueId=o_text)
        p.changeVisualShape(x_cube[i], -1, textureUniqueId=x_text)

# Pick up the next available cube
def pick_up_cube(sign):
    global x_picked
    global o_picked

    # Sign 0 for X and 1 for O
    if sign == 0:
        p.setJointMotorControlArray(arm_a, range(7), p.POSITION_CONTROL,targetPositions=cubes[x_picked%5])
        stepSim(quick)
        grab(arm_a)
        move_arm(arm_a, pickedUp, slow)
        x_picked += 1
    else:
        p.setJointMotorControlArray(arm_b, range(7), p.POSITION_CONTROL,targetPositions=cubes[o_picked%5])
        stepSim(quick)
        grab(arm_b)
        move_arm(arm_b, pickedUp, slow)
        o_picked += 1

"""
Place cubes into selected box
    sign - 0 for X and 1 for O
    box_num - position to place cube in
                    O
            1 2 3 
            4 5 6
            7 8 9
        X
"""
def place_cube(sign, box_num):
    # Change arm based on sign
    if sign == 0:
        arm = arm_a
    else:
        arm = arm_b
        box_num = 10-box_num
        
    pick_up_cube(sign)
    move_arm(arm, mid_base,slow)
    move_arm(arm, main_base, slow)
    move_arm(arm, boxes[box_num-1], slow)
    release(arm)
    move_arm(arm, main_base, quick)
    resetPos(arm)

# Take picture from camera and save
def take_picture():
    viewMatrix = p.computeViewMatrix(
        cameraEyePosition=[0, 0, 1.35],
        cameraTargetPosition=[0, 0, 0],
        cameraUpVector=[0, 1, 0])

    projectionMatrix = p.computeProjectionMatrixFOV(
        fov=45.0,
        aspect=1.0,
        nearVal=0.1,
        farVal=3.1)

    img = p.getCameraImage(600,600,viewMatrix=viewMatrix,projectionMatrix=projectionMatrix,renderer=p.ER_TINY_RENDERER)
    rgbBuf = img[2]
    rgbim = Image.fromarray(rgbBuf)
    rgbim.save('rgb_pic.png')

# Use color mask to seperate X, O
def save_color_mask():
    img = cv2.imread('rgb_pic.png')
    img = cv2.resize(img, (600,600))
    mask_b = cv2.inRange(img, (50,0,0), (255, 50, 50))
    mask_r = cv2.inRange(img, (0, 0, 50), (50, 50,255))
    cv2.imwrite("blue_mask.png", mask_b)
    cv2.imwrite("red_mask.png", mask_r)
    #cv2.imshow("Blue Mask", mask_b)
    #cv2.imshow("Red Mask", mask_r)

# Split mask into 9 boxes
def crop_board(color):
    # Opens a image in RGB mode
    im = Image.open(f"{color}_mask.png")

    box = 1
    # im1 = im.crop((left, top, right, bottom))
    for i in range(0, 600,200):
        for j in range(0,600,200):
            im1 = im.crop((j, i, j+200, i+200))
            im1.save(f"board_crops/{color}/{box}.png")
            box += 1

# Convert image info into 2D grid
def convert_grid(grid, color):
    box = 1

    if color == 'red':
        symbol = 1
    else:
        symbol = -1

    for i in range(3):
        for j in range(3):
            img = Image.open(f"board_crops/{color}/{box}.png")
            box += 1
            extrema = img.convert("L").getextrema()

            # Image not blank
            if extrema != (0, 0):
                grid[i][j] = symbol

    return grid

# Update grid with current state of board
def update_grid(grid):
    # Save camera rgb image and use color mask to seperate X, O
    take_picture()
    save_color_mask()

    # Split x,o masks into 9 boxes
    crop_board('red')
    crop_board('blue')
    
    # Convert image info into 2D grid
    grid = convert_grid(grid, 'red')
    grid = convert_grid(grid, 'blue')

    print(grid)
    return grid

# Check if the current state is a winning state
def is_win(grid):
	for i in range(3):
		if sum(grid[i, :]) == 3:
			return 1
		if sum(grid[i, :]) == -3:
			return -1
	for j in range(3):
		if sum(grid[:, j]) == 3:
			return 1
		if sum(grid[:, j]) == -3:
			return -1
	if (grid[0, 0] + grid[1, 1] + grid[2, 2]) == 3 or (grid[0, 2] + grid[1, 1] + grid[2, 0]) == 3:
		return 1
	if (grid[0, 0] + grid[1, 1] + grid[2, 2]) == -3 or (grid[0, 2] + grid[1, 1] + grid[2, 0]) == -3:
		return -1
	return 0

# Checks if the grid is full
def is_full(grid):
	for r in range(3):
		for c in range(3):
			if(grid[r, c] == 0):
				return 0
	return 1

# Reset simulation and reset arm position
def reset_simulation():
    global x_picked
    global o_picked
    x_picked = 0
    o_picked = 0
    p.resetSimulation()

# Main ---------------------------------------------------------------------
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
grid = np.zeros((3, 3))

# Load models
load_cubes()
planeId = p.loadURDF("plane.urdf")
table_a = p.loadURDF("table/table.urdf",[0,0,0])
table_b = p.loadURDF("tic_tac_toe_board/table_square.urdf",[0,0.005,0])
tray_a = p.loadURDF("tray/traybox.urdf",[-0.62,-0.68,0.68]) # Red
tray_b = p.loadURDF("tray/traybox.urdf",[0.64,0.64,0.68])
arm_a = p.loadURDF("franka_panda/panda.urdf",[-0.55,0,0.65], p.getQuaternionFromEuler([0,0,0]), useFixedBase=1) # Red
arm_b  = p.loadURDF("franka_panda/panda.urdf",[0.548,-0.07,0.65], p.getQuaternionFromEuler([0,0,3]), useFixedBase=1)

# Set physics
p.setGravity(0,0,-9.81)
p.setRealTimeSimulation(0)

# Controlling Arm ----------------------------------------------------------
resetPos(arm_a)
resetPos(arm_b)

#place_cube(1,1)
#update_grid(grid)
#place_cube(0,2)
#update_grid(grid)
#place_cube(1,3)
#update_grid(grid)
#place_cube(0,5)
#update_grid(grid)
#place_cube(1,8)
#update_grid(grid)
#place_cube(0,9)
#update_grid(grid)
#place_cube(1,7)
#update_grid(grid)

# --------------------------------------------------------------------------

#cubePos, cubeOrn = p.getBasePositionAndOrientation(arm_a)
#print(cubePos,cubeOrn)

#---------------------------------------------------------------------------
#The fun part
#Step 1: establish action state pairs

#States = I googled it because i'm lazy
States = 5478

#Actions = The actions represent the 9 different spaces a block can be dropped
Actions = 9
#using the matrix to limit the number of actions will be necessary, considering you can't drop one block on another

#Now that that's established we can set up the Q-table
qtable = np.random.rand(States, Actions).tolist()

#We can set up the robot to act greedily most of the time, but sometimes do something random just to see what happens
#This is done using an epsilon value
epsilon = 0.08

'''
So here's what happens now. We run the robot through a bunch of different games, and assign reward values to
the states based on whether the robot wins or loses. Let's say for example the robot does move X in state Y and
that causes it to lose the game, the algorithm lowers the reward score of that state. We can lower the scores more
for example if the robot does something that immediately loses them the game, versus their first move which doesn't
matter as much. The opposite happens when the robot does a move that wins them the game. That way the table can be
constantly updated every time it plays so the robot gets better every game.

Challenges: how to establish and distinguish the states, how to tell what the winning states or losing states are,
whether or not to have the opponent act randomly during training or allow user influence (random probably better),
and how to explain during the presentation that this is actually useful.
'''

gridstr = str(grid)
print(gridstr)

'''
OK LISTEN THE FUCK UP FUTURE CHARLIE, WE GOT THIS IN THE FUCKIN BAG NOW ALL WE GOTTA DO IS WRITE THE SHIT
so heres the fuckin plan
step 1: get the current state as a STRING (SIDE NOTE IM A FUCKING GENIUS)
step 2: check if that string is in the array of established states
step 3: if it isn't, add it to the array and the index of that state corresponds to the index in the SA array
step 4: pick an action based on greedy alg + which spaces are open
step 5: repeat until the game ends, then update the reward values based on if it won or lost
step 6: rinse and repeat baby, ggez
'''
gridarr = [None]
for counter in range(50):
	movesarr = np.zeros((4, 2))
	index = -1
	movind = 0
	while(1):
		if is_win(grid)==0:
			if is_full(grid) == 0:
				while(1):
					r = np.random.randint(0,3)
					c = np.random.randint(0,3)
					if grid[r, c] == 0:
						#print(r, c)
						place_cube(1, (3*r+(c+1)))
						update_grid(grid)
						break
			else:
				print("Tie game! Reward values slightly reduced")
				for i in range(movind):
					qtable[int(movesarr[i][0])][int(movesarr[i][1])] -= 0.25
					#if(i == movind-1): qtable[int(movesarr[i][0])][int(movesarr[i][1])] -= 1
					print(qtable[int(movesarr[i][0])][int(movesarr[i][1])])
				break
		else:
			if is_win(grid) == 1:
				print("Good job! Increasing reward values")
				for i in range(movind):
					if(i == movind-1): qtable[int(movesarr[i][0])][int(movesarr[i][1])] += 1
					else: qtable[int(movesarr[i][0])][int(movesarr[i][1])] += 0.5
					print(qtable[int(movesarr[i][0])][int(movesarr[i][1])])
			else:
				print("You suck! Decreasing reward values")
				for i in range(movind):
					if(i == movind-1): qtable[int(movesarr[i][0])][int(movesarr[i][1])] -= 1
					else: qtable[int(movesarr[i][0])][int(movesarr[i][1])] -= 0.5
					print(qtable[int(movesarr[i][0])][int(movesarr[i][1])])
			break
		gridstr = str(grid)
		if is_win(grid)==0:
			if is_full(grid) == 0:
				if gridstr not in gridarr: 
					gridarr.append(gridstr)
				index = gridarr.index(gridstr)
				print(index)
				movesarr[movind][0] = index
				best_move = 0
				best_num = 0
				r = 0
				c = 0
				for i in range(9):
					#print(r, c)
					if qtable[index][i] > best_move and grid[r, c] == 0:
						best_move = qtable[index][i]
						best_num = i
					if (c < 2):
						c = c+1
					else:
						c = 0
						r = r+1
				movesarr[movind][1] = best_num
				place_cube(0,best_num+1)
				update_grid(grid)
				movind+=1
			else:
				print("Tie game! Reward values slightly reduced")
				for i in range(movind):
					qtable[int(movesarr[i][0])][int(movesarr[i][1])] -= 0.25
					#if(i == movind-1): qtable[int(movesarr[i][0])][int(movesarr[i][1])] -= 1
					print(qtable[int(movesarr[i][0])][int(movesarr[i][1])])
				break
		else:
			#if movind == 4: movind -= 1
			if is_win(grid) == 1:
				print("Good job! Increasing reward values")
				for i in range(movind):
					if(i == movind-1): qtable[int(movesarr[i][0])][int(movesarr[i][1])] += 1
					else: qtable[int(movesarr[i][0])][int(movesarr[i][1])] += 0.5
					print(qtable[int(movesarr[i][0])][int(movesarr[i][1])])
			else:
				print("You suck! Decreasing reward values")
				for i in range(movind):
					if(i == movind-1): qtable[int(movesarr[i][0])][int(movesarr[i][1])] -= 1
					else: qtable[int(movesarr[i][0])][int(movesarr[i][1])] -= 0.5
					print(qtable[int(movesarr[i][0])][int(movesarr[i][1])])
			break

	reset_simulation()
	#physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
	p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
	grid = np.zeros((3, 3))

	# Load models
	load_cubes()
	planeId = p.loadURDF("plane.urdf")
	table_a = p.loadURDF("table/table.urdf",[0,0,0])
	table_b = p.loadURDF("tic_tac_toe_board/table_square.urdf",[0,0.005,0])
	tray_a = p.loadURDF("tray/traybox.urdf",[0.62,0.65,0.67])
	tray_b = p.loadURDF("tray/traybox.urdf",[-0.62,-0.68,0.67])
	arm_a = p.loadURDF("franka_panda/panda.urdf",[-0.55,0,0.65], p.getQuaternionFromEuler([0,0,0]), useFixedBase=1)
	arm_b  = p.loadURDF("franka_panda/panda.urdf",[0.548,-0.07,0.65], p.getQuaternionFromEuler([0,0,3]), useFixedBase=1)
	p.setGravity(0,0,-9.81)
	p.setRealTimeSimulation(0)
	resetPos(arm_a)
	resetPos(arm_b)

#---------------------------------------------------------------------------



print("\nFinished!\n")

while True:
    p.stepSimulation()
    time.sleep(0.01)

p.disconnect()

