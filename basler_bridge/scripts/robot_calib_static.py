#!/usr/bin/env python
import sys
sys.dont_write_bytecode = True

import numpy as np
import rospy
import tf
from kinect_msgs.srv import Calibrate
from robot_control_msgs.srv import GetPose
from robot_control_msgs.srv import SetCamera
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

"""

rospy.init_node('robot_calibrator')

## Board Parameters
"(SET THIS VALUE)"
boardSize = (9,6)
squareSize = 0.117
boardThickness = 0.03
squareShift = -2

## Offset to EF (Base Frame)
"(SET THIS VALUE)"
calibMat = getYawMatrix(0.)
PinOffset = [-36.51 * 1e-3 ,114.96 * 1e-3 ,680.30 * 1e-3]
PinOffset = performTransform(PinOffset, calibMat)

## Robot Positions
"(SET THIS VALUE)"
P0_r = np.array(addOffset([839.09 * 1e-3, -4552.61 * 1e-3, (3637.39 * 1e-3)+boardThickness], PinOffset))
P1_r = np.array(addOffset([838.50 * 1e-3, -3968.42 * 1e-3, (3635.07 * 1e-3)+boardThickness], PinOffset))
P2_r = np.array(addOffset([-96.97 * 1e-3, -4551.12 * 1e-3, (3641.63 * 1e-3)+boardThickness], PinOffset))
P3_r = np.array(addOffset([-97.85 * 1e-3, -3967.84 * 1e-3, (3640.78 * 1e-3)+boardThickness], PinOffset))

## Shift to Base
P0_r[1] = -P0_r[1]
P1_r[1] = -P1_r[1]
P2_r[1] = -P2_r[1]
P3_r[1] = -P3_r[1]
P0_r[2] = 4.344 - P0_r[2]
P1_r[2] = 4.344 - P1_r[2]
P2_r[2] = 4.344 - P2_r[2]
P3_r[2] = 4.344 - P3_r[2]

## Board Positions
x_len = squareSize*(boardSize[0]-1)
y_len = squareSize*(boardSize[1]-1)
P0_b = np.asarray([0. + squareShift*squareSize, 0. + squareShift*squareSize, 0.])
P1_b = np.asarray([0. + squareShift*squareSize, y_len + squareShift*squareSize, 0.])
P2_b = np.asarray([x_len + squareShift*squareSize, 0. + squareShift*squareSize, 0.])
P3_b = np.asarray([x_len + squareShift*squareSize, y_len + squareShift*squareSize, 0.])

## calibrate
A = np.matrix([P0_b, P1_b, P2_b, P3_b])
B = np.matrix([P0_r, P1_r, P2_r, P3_r])

## recover the transformation
ret_R, ret_t = rigidTransform3D(A, B)

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
rmse = testTransform(4,A,B,ret_R,ret_t)
print "RMSE: " + str(rmse)

if rmse > 0.025:
    print "Error is too High. Calibration Failure"
    sys.exit()
