#include <string>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <basler_bridge/basler_rgb_bridgeConfig.h>

#include <ConsumerImplHelper/ToFCamera.h>
#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <boost/algorithm/string.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>

#include <camera_msgs/Calibrate.h>
#include <camera_msgs/Clear.h>
#include <camera_msgs/Trigger.h>

#include "CameraData.h"
#include "ExtrinsicCalibration.h"

#include "BaslerRGB.h"

using namespace std;

namespace basler_bridge {

class BaslerRGBBridgeNodelet : public nodelet::Nodelet{

public:
    BaslerRGBBridgeNodelet();
    ~BaslerRGBBridgeNodelet();
    virtual void onInit();
    void connectCb();
    void sensorWorkerCallback();
    void tfWorkerCallback();
    bool intrinsicSaveCallback(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res);
    bool extrinsicSaveCallback(camera_msgs::Calibrate::Request &req, camera_msgs::Calibrate::Response &res);
    bool clearExtrinsicsCallback(camera_msgs::Clear::Request &req, camera_msgs::Clear::Response &res);
    bool triggerCallback(camera_msgs::Trigger::Request &req, camera_msgs::Trigger::Response &res);
    void drCallback(basler_bridge::basler_rgb_bridgeConfig &config, uint32_t level);

    dynamic_reconfigure::Server<basler_bridge::basler_rgb_bridgeConfig>* drServer_;
    boost::thread sensorWorker_, tfWorker_;
    ros::ServiceServer instrinsicCalibrateService_;
    ros::ServiceServer extrinsicCalibrateService_;
    ros::ServiceServer clearExtrinsicsService_;
    ros::ServiceServer triggerService_;

    std::string baslerName_;
    std::string serialNumber_;
    int queueSize_;
    bool trigger_;

    boost::mutex calibrationPreviewMutex_;
    cv::Mat calibrationPreviewImage_;
    ExtrinsicCalibration extrinsicCalibration_;
    CameraData cameraData_;
    CameraData::CameraObject cameraObject_;

    BaslerRGB rgbCamera_;

    ros::Publisher pubImageColor_;
    ros::Publisher pubImageColorRect_;
    ros::Publisher pubCalibrationPreview_;
    ros::Publisher pubCameraInfo_;
    bool camState_;
    bool publish_;
    int camCount_;
};

BaslerRGBBridgeNodelet::BaslerRGBBridgeNodelet(){
    camState_ = false;
}

BaslerRGBBridgeNodelet::~BaslerRGBBridgeNodelet(){
    delete drServer_;
}

void BaslerRGBBridgeNodelet::onInit(){

    // Setup node handles
    ros::NodeHandle& nh         = getMTNodeHandle();
    ros::NodeHandle& privateNh = getMTPrivateNodeHandle();

    // Read Params
    privateNh.getParam("queue_size", queueSize_);
    privateNh.getParam("basler_name", baslerName_);
    privateNh.getParam("serial_number", serialNumber_);
    privateNh.getParam("trigger", trigger_);

    // Camera Data
    if(!cameraData_.getCameraObject("basler_bridge", serialNumber_, cameraObject_)){
        ROS_ERROR("No calibration data found");
        throw;
    }

    // RGB Params
    std::string settingsPath = ros::package::getPath("basler_bridge")+"/data/"+serialNumber_+"/settings.pfs";
    rgbCamera_.setParams(serialNumber_, settingsPath);

    // Dynamic Reconfigure
    drServer_ = new dynamic_reconfigure::Server<basler_bridge::basler_rgb_bridgeConfig>(privateNh);
    dynamic_reconfigure::Server<basler_bridge::basler_rgb_bridgeConfig>::CallbackType f;
    f = boost::bind(&BaslerRGBBridgeNodelet::drCallback, this, _1, _2);
    drServer_->setCallback(f);

    // Publisher
    ros::SubscriberStatusCallback connect_cb = boost::bind(&BaslerRGBBridgeNodelet::connectCb, this);
    pubImageColor_ = nh.advertise<sensor_msgs::Image>(baslerName_+"/image_color", 1, connect_cb, connect_cb);
    pubImageColorRect_ = nh.advertise<sensor_msgs::Image>(baslerName_+"/image_color_rect", 1, connect_cb, connect_cb);
    pubCalibrationPreview_ = nh.advertise<sensor_msgs::Image>(baslerName_+"/calibration_preview", 1, connect_cb, connect_cb);
    pubCameraInfo_ = nh.advertise<sensor_msgs::CameraInfo>(baslerName_+"/camera_info", 1, connect_cb, connect_cb);

    // Calibration
    extrinsicCalibration_ = ExtrinsicCalibration(cv::Size(9,6), 0.117, ExtrinsicCalibration::CHESSBOARD);
    extrinsicCalibrateService_ = nh.advertiseService<camera_msgs::Calibrate::Request, camera_msgs::Calibrate::Response>(baslerName_ + "/Calibrate", boost::bind(&BaslerRGBBridgeNodelet::extrinsicSaveCallback, this, _1, _2));
    instrinsicCalibrateService_ = nh.advertiseService<sensor_msgs::SetCameraInfo::Request, sensor_msgs::SetCameraInfo::Response>(baslerName_ + "/set_camera_info", boost::bind(&BaslerRGBBridgeNodelet::intrinsicSaveCallback, this, _1, _2));
    clearExtrinsicsService_ = nh.advertiseService<camera_msgs::Clear::Request, camera_msgs::Clear::Response>(baslerName_ + "/Clear", boost::bind(&BaslerRGBBridgeNodelet::clearExtrinsicsCallback, this, _1, _2));
    triggerService_ = nh.advertiseService<camera_msgs::Trigger::Request, camera_msgs::Trigger::Response>(baslerName_ + "/Trigger", boost::bind(&BaslerRGBBridgeNodelet::triggerCallback, this, _1, _2));

    // Start worker
    sensorWorker_ = boost::thread(boost::bind(&BaslerRGBBridgeNodelet::sensorWorkerCallback, this));
    tfWorker_ = boost::thread(boost::bind(&BaslerRGBBridgeNodelet::tfWorkerCallback, this));

    // rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.117 image:=/TOF0/intensity camera:=/TOF0
    if(trigger_) camState_ = true;
}

void BaslerRGBBridgeNodelet::drCallback(basler_bridge::basler_rgb_bridgeConfig &config, uint32_t level){
    ROS_INFO("drCallback");
}

bool BaslerRGBBridgeNodelet::triggerCallback(camera_msgs::Trigger::Request &req, camera_msgs::Trigger::Response &res){
    if(!trigger_) return false;
    if(req.trigger) publish_ = true;
    else publish_ = false;
    res.success = true;
    return true;
}

void BaslerRGBBridgeNodelet::connectCb()
{
    if(trigger_) return;

    if (pubImageColor_.getNumSubscribers() == 0 && pubImageColorRect_.getNumSubscribers() == 0 && pubCameraInfo_.getNumSubscribers() == 0)
    {
        // Stop Camera
        camState_ = false;
        // Send signal to a thread to stop cam
    }
    else
    {
        // Start camera
        camState_ = true;
        // Send signal to a thread to start cam
    }
}

bool BaslerRGBBridgeNodelet::clearExtrinsicsCallback(camera_msgs::Clear::Request &req, camera_msgs::Clear::Response &res){
    cameraData_.clearExtrinsics("basler_bridge", serialNumber_);
    cameraData_.getCameraObject("basler_bridge", serialNumber_, cameraObject_);
    res.response = "Cleared";
    return true;
}

bool BaslerRGBBridgeNodelet::extrinsicSaveCallback(camera_msgs::Calibrate::Request &req, camera_msgs::Calibrate::Response &res){
    calibrationPreviewMutex_.lock();
    cv::Mat calibrationPreviewImage = calibrationPreviewImage_.clone();
    calibrationPreviewMutex_.unlock();

    // Set Extrinsics
    try {
        std::vector<std::string> details;
        boost::split(details, req.board_size, boost::is_any_of("x"));
        cv::Size boardSize = cv::Size(boost::lexical_cast<int>(details[0]), boost::lexical_cast<int>(details[1]));
        extrinsicCalibration_.setBoardSize(boardSize);
        extrinsicCalibration_.setSquareSize(req.square_size);
        if(req.frame.length() == 0) throw;
    } catch( const std::exception &e) {
        cerr << e.what() << endl;
        res.response = "Some parameters were invalid";
        return false;
    }

    // Extract an image
    bool cameraWasRunning = camState_;
    if(!cameraWasRunning) camState_ = true;
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    if(!cameraWasRunning) camState_ = false;
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    // Calibration code
    cv::Mat extMat;
    extrinsicCalibration_.setCameraMatrix(cameraObject_.cameraMatrixColor.clone());
    bool ok = extrinsicCalibration_.calibrateExtrinsics(calibrationPreviewImage_, extMat, req.square_shift, req.height_shift, Eigen::Vector3f(req.x_shift,req.y_shift,0));
    if(!ok){
        res.response = "Calibration failed";
        return false;
    }
    extrinsicCalibration_.checkCalibration(calibrationPreviewImage, extMat);

    // Save extrinsics
    if(req.frame != "calibration_frame"){
    cameraData_.saveTransforms("basler_bridge", serialNumber_, req.frame, extMat, req.direction);
    cameraData_.getCameraObject("basler_bridge", serialNumber_, cameraObject_);
    }else{
    cout << "cTb" << endl;
    cout << extMat << endl;
    for(int r=0; r<4; r++){
        for(int c=0; c<4; c++){
            if(r==3 && c==3) res.worldToCamera.push_back(1);
            else res.worldToCamera.push_back(extMat.at<double>(r,c));
        }
    }
    }

    // Publish
    sensor_msgs::ImagePtr calibrationPreviewMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", calibrationPreviewImage).toImageMsg();
    pubCalibrationPreview_.publish(calibrationPreviewMsg);

    res.response = "Calibrated";
    return true;
}

bool BaslerRGBBridgeNodelet::intrinsicSaveCallback(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res){
    ROS_INFO("Calibrating");
    cameraData_.saveCameraMatrixColor("basler_bridge", serialNumber_, req.camera_info);
    cameraData_.getCameraObject("basler_bridge", serialNumber_, cameraObject_);
    res.status_message = "";
    res.success = true;
    return true;
}

void BaslerRGBBridgeNodelet::tfWorkerCallback(){
    while(1){
        // Send TF
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
        static tf::TransformBroadcaster br;
        for(std::map<std::string, tf::Transform>::iterator iterator = cameraObject_.cameraToFramesTF.begin(); iterator != cameraObject_.cameraToFramesTF.end(); iterator++) {
            std::string extFrame = iterator->first;
            tf::Transform transform = iterator->second;
            bool direction = cameraObject_.cameraToFramesDir[extFrame];
            if(direction){
                br.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), extFrame, baslerName_ + "_rgb_optical_frame"));
            }else{
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),  baslerName_ + "_rgb_optical_frame", extFrame));
            }
        }
        //ros::Time publishTime = ros::Time::now();
        //cameraObject_.cameraInfoColor.header.stamp = publishTime;
        //pubCameraInfo_.publish(cameraObject_.cameraInfoColor);
    }
}

void BaslerRGBBridgeNodelet::sensorWorkerCallback(){
    while(1){
        // Need some signal here
        if(!camState_){
            camCount_ = 0;
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            rgbCamera_.stop();
            continue;
        }else{
            // Start RGB Camera and Grab images
            rgbCamera_.start();
            rgbCamera_.grabImages();
            cv::Mat rgbMap = rgbCamera_.getRGBMap().clone();

            // Rect
            cv::Mat rgbMapRect = rgbMap.clone();
            cv::remap(rgbMap, rgbMapRect, cameraObject_.map1Color, cameraObject_.map2Color, cv::INTER_LINEAR);

            // For calibration store image
            calibrationPreviewMutex_.lock();
            calibrationPreviewImage_ = rgbMapRect;
            calibrationPreviewMutex_.unlock();

            // Setup Msg
            sensor_msgs::ImagePtr rgbMapMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgbMap).toImageMsg();
            sensor_msgs::ImagePtr rgbMapRectMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgbMapRect).toImageMsg();
            rgbMapMsg->header.frame_id=baslerName_+"_rgb_optical_frame";
            rgbMapRectMsg->header.frame_id=baslerName_+"_rgb_optical_frame";
            cameraObject_.cameraInfoColor.header.frame_id=baslerName_+"_rgb_optical_frame";

            // Publish Msg
            ros::Time publishTime = ros::Time::now();
            rgbMapMsg->header.stamp = publishTime;
            rgbMapRectMsg->header.stamp = publishTime;
            if(trigger_){
                if(!publish_) continue;
            }
            pubImageColor_.publish(rgbMapMsg);
            pubImageColorRect_.publish(rgbMapRectMsg);
            cameraObject_.cameraInfoColor.header.stamp = publishTime;
            pubCameraInfo_.publish(cameraObject_.cameraInfoColor);
            camCount_++;
        }
    }
}

}

//// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (basler_bridge, basler_rgb_bridge, basler_bridge::BaslerRGBBridgeNodelet, nodelet::Nodelet);

