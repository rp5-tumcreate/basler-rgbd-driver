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

def moveRobot(position, orientation):
    client = actionlib.SimpleActionClient('action_server_robot_control', RobotControlAction)
    client.wait_for_server()
    goal = RobotControlGoal()
    goal.action_id = 10003
    goal.object_uuid = ''
    goal.tool_id = 30000
    goal.speed = 100
    goal.acceleration = 20
    goal.payload = 5

    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    quaternion = tf.transformations.quaternion_from_euler(orientation[0]/((180./np.pi)),orientation[1]/((180./np.pi)),orientation[2]/((180./np.pi)))
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    goal.target_pose = pose

    client.send_goal(goal)
    client.wait_for_result()
    print client.get_result()
    pass

def getYawMatrix(angle):
    yaw = np.matrix([[np.cos(np.radians(angle)), -np.sin(np.radians(angle)), 0., 0.],
                     [np.sin(np.radians(angle)), np.cos(np.radians(angle)), 0., 0.],
                     [0., 0., 1., 0.],
                     [0., 0., 0., 1. ]])
    return yaw

def inv(m):
    return np.linalg.inv(m)

def addOffset(m, offset=[0.,0.,0.]):
    m[0] = m[0]+offset[0]
    m[1] = m[1]+offset[1]
    m[2] = m[2]+offset[2]
    return m

def rigidTransform3D(A, B):
    assert len(A) == len(B)
    N = A.shape[0];
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - np.tile(centroid_A, (N, 1))
    BB = B - np.tile(centroid_B, (N, 1))
    H = np.transpose(AA) * BB
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T * U.T
    if np.linalg.det(R) < 0:
       print "Reflection detected"
       Vt[2,:] *= -1
       R = Vt.T * U.T
    t = -R*centroid_A.T + centroid_B.T
    return R, t

def testTransform(n,A,B,R,t):
    A2 = (R*A.T) + np.tile(t, (1, n))
    A2 = A2.T
    err = A2 - B
    err = np.multiply(err, err)
    err = np.sum(err)
    rmse = np.sqrt(err/n);
    return rmse

def getcTb(square_size, RGB_CALIBRATE_TOPIC):
    print "waiting for calibration.."
    rospy.wait_for_service(RGB_CALIBRATE_TOPIC)
    try:
        calibrate = rospy.ServiceProxy(RGB_CALIBRATE_TOPIC, Calibrate)
        resp = calibrate("calibration_frame", "9x6", square_size, True, 0, 0, 0, 0)
        if len(resp.worldToCamera) == 0:
            print "Failed to Calibrate"
            sys.exit()
        cTb = np.matrix([[resp.worldToCamera[0], resp.worldToCamera[1], resp.worldToCamera[2], resp.worldToCamera[3]],
                         [resp.worldToCamera[4], resp.worldToCamera[5], resp.worldToCamera[6], resp.worldToCamera[7]],
                         [resp.worldToCamera[8], resp.worldToCamera[9], resp.worldToCamera[10], resp.worldToCamera[11]],
                         [0., 0., 0., 1. ]])
        print "cTb: "
        print cTb
        return cTb
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return None

def getRobotPose():
    print "waiting for getpose.."
    rospy.wait_for_service('/ABB/get_pose')
    try:
        getPose = rospy.ServiceProxy('/ABB/get_pose', GetPose)
        resp = getPose()
        position = np.array(addOffset([resp.pose.position.x * 1e-3, -resp.pose.position.y * 1e-3, ((4.344*1000)-resp.pose.position.z) * 1e-3]))
        quaternion = (
            resp.pose.orientation.x,
            resp.pose.orientation.y,
            resp.pose.orientation.z,
            resp.pose.orientation.w)
        orientation = tf.transformations.euler_from_quaternion(quaternion)
        orientation = [orientation[0]*(180./np.pi), -orientation[1]*(180./np.pi), -orientation[2]*(180./np.pi)]
        return position, orientation
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return None, None

def setTransform(transform, ABB_CALIBRATE_TOPIC):
    print "waiting for setcamera.."
    rospy.wait_for_service(ABB_CALIBRATE_TOPIC)
    try:
        setCamera = rospy.ServiceProxy(ABB_CALIBRATE_TOPIC, SetCamera)
        resp = setCamera(transform)
        print resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        sys.exit()

def performTransform(arr, transform):
    if type(arr) == np.ndarray:
        nx = arr[0]*transform[0,0] + arr[1]*transform[0,1] + arr[2]*transform[0,2] + 1*transform[0,3]
        ny = arr[0]*transform[1,0] + arr[1]*transform[1,1] + arr[2]*transform[1,2] + 1*transform[1,3]
        nz = arr[0]*transform[2,0] + arr[1]*transform[2,1] + arr[2]*transform[2,2] + 1*transform[2,3]
        return np.array([nx,ny,nz])
    else:
        nx = arr[0]*transform[0,0] + arr[1]*transform[0,1] + arr[2]*transform[0,2] + 1*transform[0,3]
        ny = arr[0]*transform[1,0] + arr[1]*transform[1,1] + arr[2]*transform[1,2] + 1*transform[1,3]
        nz = arr[0]*transform[2,0] + arr[1]*transform[2,1] + arr[2]*transform[2,2] + 1*transform[2,3]
        return [nx,ny,nz]

def generatePairs(boardSize, squareSize, robotCalibPoints, RGB_CALIBRATE_TOPIC):
    # 4 Points in Board Frame
    x_len = squareSize*(boardSize[0]-1)
    y_len = squareSize*(boardSize[1]-1)
    P0_b = np.asarray([0., 0., 0., 1.])
    P1_b = np.asarray([0., y_len, 0., 1.])
    P2_b = np.asarray([x_len, 0., 0., 1.])
    P3_b = np.asarray([x_len, y_len, 0., 1.])
    P0_b = np.reshape(P0_b,(4,1))
    P1_b = np.reshape(P1_b,(4,1))
    P2_b = np.reshape(P2_b,(4,1))
    P3_b = np.reshape(P3_b,(4,1))

    # Board to Camera
    cTb = getcTb(squareSize, RGB_CALIBRATE_TOPIC)
    if cTb is None:
        return None, None
    bTc = inv(cTb)

    # 4 Points in Camera Frame
    P0_c = cTb * P0_b
    P1_c = cTb * P1_b
    P2_c = cTb * P2_b
    P3_c = cTb * P3_b
    P0_c = np.array([float(P0_c[0]),float(P0_c[1]),float(P0_c[2])])
    P1_c = np.array([float(P1_c[0]),float(P1_c[1]),float(P1_c[2])])
    P2_c = np.array([float(P2_c[0]),float(P2_c[1]),float(P2_c[2])])
    P3_c = np.array([float(P3_c[0]),float(P3_c[1]),float(P3_c[2])])

    # Robot Position
    O_r, Oe_r = getRobotPose()
    if O_r is None or Oe_r is None:
        return None, None
    yawMat = getYawMatrix(Oe_r[2])

    # 4 Points in Robot Frame
    if len(robotCalibPoints) is not 4:
        print "Wrong number of robot calib points"
        sys.exit()
    P0_e = robotCalibPoints[0] - O_r
    P1_e = robotCalibPoints[1] - O_r
    P2_e = robotCalibPoints[2] - O_r
    P3_e = robotCalibPoints[3] - O_r
    P0_e = performTransform(P0_e, inv(yawMat))
    P1_e = performTransform(P1_e, inv(yawMat))
    P2_e = performTransform(P2_e, inv(yawMat))
    P3_e = performTransform(P3_e, inv(yawMat))

    # Generate Pair Matrix
    camPoints = np.matrix([P0_c, P1_c, P2_c, P3_c])
    efPoints = np.matrix([P0_e, P1_e, P2_e, P3_e])

    # Return
    return camPoints, efPoints

def convertMMToMetre(position):
    newPosition = position
    newPosition[0] = newPosition[0] * 1e-3
    newPosition[1] = newPosition[1] * 1e-3
    newPosition[2] = newPosition[2] * 1e-3
    return newPosition
