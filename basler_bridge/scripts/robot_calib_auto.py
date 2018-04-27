#!/usr/bin/env python
import sys
sys.dont_write_bytecode = True

import numpy as np
import rospy
import tf
import actionlib
from kinect_msgs.srv import Calibrate
from robot_control_msgs.srv import GetPose
from robot_control_msgs.srv import SetCamera
from robot_control_msgs.msg import RobotControlAction, RobotControlGoal
from geometry_msgs.msg import Pose
import time
from robot_calib_lib import *

"""
(ROBOT CALIBRATION)

[-]                                 [-]





    0-------1
    |  _y   |
    | |     |
    | x     |
    |       |
    2-------3

[-]                                 [-]

0. Enter in the (PinOffset) variable. Ensure this is correct. Also Enter the (boardSize) and (squareSize).
1. Rotate EF so Robot Controller says orientation is 0.
2. Place calibration board matching numbers on the floor
3. Move Pin Towards black corner poins and record it
4. Record the robot position values
5. Run Script to compute transform and note RMSE value

"""

# Get Camera Type
RGB_CALIBRATE_TOPIC = ''
ABB_CALIBRATE_TOPIC = ''
user_input = raw_input("\nEnter camera type {side, bottom, exit}:\n")
if user_input == 'side':
    RGB_CALIBRATE_TOPIC = '/RGB0/Calibrate'
    ABB_CALIBRATE_TOPIC = '/ABB/set_side_camera'
elif user_input == 'bottom':
    RGB_CALIBRATE_TOPIC = '/RGB1/Calibrate'
    ABB_CALIBRATE_TOPIC = '/ABB/set_bottom_camera'
else:
    sys.exit()
print "Waiting for services.. (" + RGB_CALIBRATE_TOPIC + ")"
rospy.wait_for_service(RGB_CALIBRATE_TOPIC)
print "Waiting for services.. (" + ABB_CALIBRATE_TOPIC + ")"
rospy.wait_for_service(ABB_CALIBRATE_TOPIC)

rospy.init_node('robot_calibrator')

## Board Parameters
"(SET THIS VALUE)"
boardSize = (9,6)
squareSize = 0.117

## Offset to EF (Base Frame)
"(SET THIS VALUE)"
calibMat = getYawMatrix(0.)
PinOffset = [-36.51 * 1e-3 ,114.96 * 1e-3 ,680.30 * 1e-3]
PinOffset = performTransform(PinOffset, calibMat)

## Board Positions
"(SET THIS VALUE)"
P0_r = np.array(addOffset([839.09 * 1e-3, -4552.61 * 1e-3, 3637.39 * 1e-3], PinOffset))
P1_r = np.array(addOffset([838.50 * 1e-3, -3968.42 * 1e-3, 3635.07 * 1e-3], PinOffset))
P2_r = np.array(addOffset([-96.97 * 1e-3, -4551.12 * 1e-3, 3641.63 * 1e-3], PinOffset))
P3_r = np.array(addOffset([-97.85 * 1e-3, -3967.84 * 1e-3, 3640.78 * 1e-3], PinOffset))

################ AUTO MODE ################

A_Full = np.matrix([0,0,0])
B_Full = np.matrix([0,0,0])
count = 0

move_poses_side = [
    [convertMMToMetre([3188, -4072, 1581]), [0,0,0]],
    [convertMMToMetre([2902, -4072, 1794]), [0,0,0]],
    [convertMMToMetre([2473, -4072, 2369]), [0,0,0]],
    [convertMMToMetre([2059, -4072, 2873]), [0,0,0]],
    [convertMMToMetre([1897, -4072, 3189]), [0,0,0]],

    [convertMMToMetre([1649, -2802, 2848]), [0,0,45]],
    [convertMMToMetre([1828, -2653, 2551]), [0,0,45]],
    [convertMMToMetre([2019, -2372, 2272]), [0,0,45]],
    [convertMMToMetre([2326, -2101, 1967]), [0,0,45]],
    #[convertMMToMetre([2482, -1850, 1689]), [0,0,45]],

    #[convertMMToMetre([320, -1266, 1689]), [0,0,90]],
    [convertMMToMetre([320, -1526, 1988]), [0,0,90]],
    [convertMMToMetre([320, -1938, 2455]), [0,0,90]],
    [convertMMToMetre([320, -2306, 2906]), [0,0,90]],
    [convertMMToMetre([320, -2590, 3220]), [0,0,90]]
]
move_poses_side = list(reversed(move_poses_side))

move_poses_bottom = [
    [convertMMToMetre([567, -4107, 2692]), [0,0,0]],
    [convertMMToMetre([567, -4107, 2118]), [0,0,0]],
    [convertMMToMetre([567, -4107, 1729]), [0,0,0]],
    [convertMMToMetre([567, -4107, 1447]), [0,0,0]]
]
move_poses_bottom = list(reversed(move_poses_bottom))

if user_input == 'side':
    move_poses = move_poses_side
elif user_input == 'bottom':
    move_poses = move_poses_bottom

for move_pose in move_poses:
    moveRobot(move_pose[0], move_pose[1])
    time.sleep(3)

    A, B = generatePairs(boardSize, squareSize, [P0_r, P1_r, P2_r, P3_r], RGB_CALIBRATE_TOPIC)
    if A is None or B is None:
        print "Failed to get pairs"
        continue

    # Check Data Temp
    A_Full_Copy = np.copy(A_Full)
    B_Full_Copy = np.copy(B_Full)
    A_Full_Copy = np.vstack([A_Full_Copy, A])
    B_Full_Copy = np.vstack([B_Full_Copy, B])
    A_Full_Copy = np.delete(A_Full_Copy, (0), axis=0)
    B_Full_Copy = np.delete(B_Full_Copy, (0), axis=0)
    ret_R, ret_t = rigidTransform3D(A_Full_Copy, B_Full_Copy)
    globalRMSE = testTransform(count+4,A_Full_Copy,B_Full_Copy,ret_R,ret_t)
    ret_R, ret_t = rigidTransform3D(A, B)
    localRMSE = testTransform(4,A,B,ret_R,ret_t)
    print "Global RMSE: " + str(globalRMSE)
    print "Local RMSE: " + str(localRMSE)
    if globalRMSE > 0.04:
        print "Ignoring Data Points.."
        continue

    A_Full = np.vstack([A_Full, A])
    B_Full = np.vstack([B_Full, B])
    count = count + 4
    print "Added data.."

A_Full = np.delete(A_Full, (0), axis=0)
B_Full = np.delete(B_Full, (0), axis=0)

print ""
print "Point Pairs"
print A_Full
print B_Full
print ""

################ 4 POINT MODE ################

### Board to Camera
#"(Set by Calibrator)"
#cTb = getcTb(squareSize)
#bTc = inv(cTb)

### Robot Positions
#"(Set by Robot Controller)"
#O_r, Oe_r = getRobotPose()
#yawMat = getYawMatrix(-Oe_r[2])
##O_r = np.array(addOffset([973.29 * 1e-3, -4372.28 * 1e-3, 2458.89 * 1e-3]))

### 4 Points in Board Frame
#x_len = squareSize*(boardSize[0]-1)
#y_len = squareSize*(boardSize[1]-1)
#P0_b = np.asarray([0., 0., 0., 1.])
#P1_b = np.asarray([0., y_len, 0., 1.])
#P2_b = np.asarray([x_len, 0., 0., 1.])
#P3_b = np.asarray([x_len, y_len, 0., 1.])
#P0_b = np.reshape(P0_b,(4,1))
#P1_b = np.reshape(P1_b,(4,1))
#P2_b = np.reshape(P2_b,(4,1))
#P3_b = np.reshape(P3_b,(4,1))

### 4 Points in Camera Frame
#P0_c = cTb * P0_b
#P1_c = cTb * P1_b
#P2_c = cTb * P2_b
#P3_c = cTb * P3_b
#P0_c = np.array([float(P0_c[0]),float(P0_c[1]),float(P0_c[2])])
#P1_c = np.array([float(P1_c[0]),float(P1_c[1]),float(P1_c[2])])
#P2_c = np.array([float(P2_c[0]),float(P2_c[1]),float(P2_c[2])])
#P3_c = np.array([float(P3_c[0]),float(P3_c[1]),float(P3_c[2])])

### Compute the corrsp points (Transform from EF bf to current EF frame)
#P0_e = P0_r - O_r
#P1_e = P1_r - O_r
#P2_e = P2_r - O_r
#P3_e = P3_r - O_r
#P0_e = performTransform(P0_e, inv(yawMat))
#P1_e = performTransform(P1_e, inv(yawMat))
#P2_e = performTransform(P2_e, inv(yawMat))
#P3_e = performTransform(P3_e, inv(yawMat))

### Setup Matrix
#n = 4
#A_Full = np.matrix([P0_c, P1_c, P2_c, P3_c])
#B_Full = np.matrix([P0_e, P1_e, P2_e, P3_e])

################ MULTI POINT MODE ################

# A_Full = np.matrix([0,0,0])
# B_Full = np.matrix([0,0,0])
# count = 0

# while True:
#     user_input = raw_input("\nMove the robot: Then type {ok, calibrate, exit}:\n")
#     if user_input == 'exit':
#         sys.exit()
#     if user_input == 'ok':
#         A, B = generatePairs(boardSize, squareSize, [P0_r, P1_r, P2_r, P3_r])
#         if A is None or B is None:
#             print "Failed to get pairs"
#             continue
#         A_Full = np.vstack([A_Full, A])
#         B_Full = np.vstack([B_Full, B])
#         count = count + 4
#     if user_input == 'calibrate':
#         A_Full = np.delete(A_Full, (0), axis=0)
#         B_Full = np.delete(B_Full, (0), axis=0)
#         break

# print ""
# print "Point Pairs"
# print A_Full
# print B_Full
# print ""

################ SVD ################

## recover the transformation
ret_R, ret_t = rigidTransform3D(A_Full, B_Full)

print "Rotation"
print ret_R
print ""

print "Translation"
print ret_t
print ""

## full transform decomposed
print "Full Transform"
transform = np.matrix([
    [ret_R[0,0], ret_R[0,1], ret_R[0,2], ret_t[0].tolist()[0][0]],
    [ret_R[1,0], ret_R[1,1], ret_R[1,2], ret_t[1].tolist()[0][0]],
    [ret_R[2,0], ret_R[2,1], ret_R[2,2], ret_t[2].tolist()[0][0]],
    [0, 0, 0, 1],
    ])
print transform
print ""

print "Euler"
scale, shear, rpy_angles, translation_vector, perspective = tf.transformations.decompose_matrix(transform)
print (rpy_angles[0]*(180./np.pi), rpy_angles[1]*(180./np.pi), rpy_angles[2]*(180./np.pi))
print translation_vector
print ""

## error
rmse = testTransform(count,A_Full,B_Full,ret_R,ret_t)
print "RMSE: " + str(rmse)

if rmse > 0.04:
    print "Error is too High. Calibration Failure"
    sys.exit()

## Set
transform_list = []
transform_list.append(ret_R[0,0])
transform_list.append(ret_R[0,1])
transform_list.append(ret_R[0,2])
transform_list.append(ret_t[0].tolist()[0][0])
transform_list.append(ret_R[1,0])
transform_list.append(ret_R[1,1])
transform_list.append(ret_R[1,2])
transform_list.append(ret_t[1].tolist()[0][0])
transform_list.append(ret_R[2,0])
transform_list.append(ret_R[2,1])
transform_list.append(ret_R[2,2])
transform_list.append(ret_t[2].tolist()[0][0])
setTransform(transform_list, ABB_CALIBRATE_TOPIC)
