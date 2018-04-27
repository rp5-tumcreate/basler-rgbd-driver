#ifndef CAMERA_DATA_H
#define CAMERA_DATA_H

#include <string>
#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/opencv.hpp>

#include <dirent.h>
#include <sstream>

#define K2_CALIB_COLOR         "calib_color.yaml"
#define K2_CALIB_IR            "calib_ir.yaml"
#define K2_CALIB_POSE          "calib_pose.yaml"
#define K2_CALIB_DEPTH         "calib_depth.yaml"
#define K2_CALIB_WORLD         "calib_world.yaml"
#define K2_CALIB_IR_RESOLUTION "calib_ir_resolution.yaml"
#define K2_CALIB_COLOR_RESOLUTION "calib_color_resolution.yaml"

#define K2_CALIB_RESOLUTION    "resolution"
#define K2_CALIB_CAMERA_MATRIX "cameraMatrix"
#define K2_CALIB_DISTORTION    "distortionCoefficients"
#define K2_CALIB_ROTATION      "rotation"
#define K2_CALIB_PROJECTION    "projection"
#define K2_CALIB_TRANSLATION   "translation"
#define K2_CALIB_ESSENTIAL     "essential"
#define K2_CALIB_FUNDAMENTAL   "fundamental"
#define K2_CALIB_DEPTH_SHIFT   "depthShift"
#define K2_CALIB_WORLD_TRANSFORM   "camera_to_world"

using namespace std;

class CameraData
{
private:

    tf::Transform tfFromMat(cv::Mat transformMatrix){
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(transformMatrix.at<double>(0, 3), transformMatrix.at<double>(1, 3), transformMatrix.at<double>(2, 3)));
        tf::Quaternion q;
        tf::Matrix3x3 tf3d;
        tf3d.setValue(
            transformMatrix.at<double>(0, 0), transformMatrix.at<double>(0, 1), transformMatrix.at<double>(0, 2),
            transformMatrix.at<double>(1, 0), transformMatrix.at<double>(1, 1), transformMatrix.at<double>(1, 2),
            transformMatrix.at<double>(2, 0), transformMatrix.at<double>(2, 1), transformMatrix.at<double>(2, 2)
        );
        tf3d.getRotation(q);
        transform.setRotation(q);
        return transform;
    }

    cv::Mat matFromTF(tf::Transform transform){
        tf::Quaternion q = transform.getRotation();
        tf::Vector3 t = transform.getOrigin();
        tf::Matrix3x3 rot;
        rot.setRotation(q);
        cv::Mat transformMatrix = (cv::Mat_<double>(4,4) <<
                                    rot[0][0], rot[0][1], rot[0][2], t[0],
                                    rot[1][0], rot[1][1], rot[1][2], t[1],
                                    rot[2][0], rot[2][1], rot[2][2], t[2],
                                    0, 0, 0, 1
                                  );
        return transformMatrix;
    }

public:

    struct CameraObject{
        int cameraNo;
        std::map<std::string, tf::Transform> cameraToFramesTF;
        std::map<std::string, bool> cameraToFramesDir;

        cv::Mat cameraMatrixColor, cameraMatrixColorPadded, distortionColor, map1Color, map2Color;
        sensor_msgs::CameraInfo cameraInfoColor;

        cv::Mat cameraMatrixIR, cameraMatrixIRPadded, distortionIR, map1IR, map2IR;
        sensor_msgs::CameraInfo cameraInfoIR;

        cv::Mat colorToDepth, depthToColor;
        tf::Transform colorToDepthTF, depthToColorTF;

        double depthShift;

        void clear(){
            cameraToFramesTF.clear();
            cameraToFramesDir.clear();
            cameraMatrixColor.release();
            cameraMatrixColorPadded.release();
            distortionColor.release();
            map1Color.release();
            map2Color.release();
            cameraInfoColor = sensor_msgs::CameraInfo();
            cameraMatrixIR.release();
            cameraMatrixIRPadded.release();
            distortionIR.release();
            map1IR.release();
            map2IR.release();
            cameraInfoIR = sensor_msgs::CameraInfo();
            colorToDepth.release();
            depthToColor.release();
            colorToDepthTF = tf::Transform();
            depthToColorTF = tf::Transform();
            depthShift = 0.0;
        }
    };

    CameraData();
    bool getCameraObject(std::string package, std::string serialNumber, CameraObject& c, cv::Size colorSize=cv::Size(1920,1080), cv::Size irSize=cv::Size(640,480));
    void saveTransforms(std::string package, std::string serialNumber, std::string frame, cv::Mat extMat, bool direction, cv::Size colorSize=cv::Size(1920,1080), cv::Size irSize=cv::Size(640,480));
    void saveCameraMatrixColor(std::string package, std::string serialNumber, sensor_msgs::CameraInfo cameraInfo, cv::Size colorSize=cv::Size(1920,1080), cv::Size irSize=cv::Size(640,480));
    void saveCameraMatrixIR(std::string package, std::string serialNumber, sensor_msgs::CameraInfo cameraInfo, cv::Size colorSize=cv::Size(1920,1080), cv::Size irSize=cv::Size(640,480));
    void clearExtrinsics(std::string package, std::string serialNumber);

};


#endif
