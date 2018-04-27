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
#include <basler_bridge/basler_tof_bridgeConfig.h>
#if CV_MAJOR_VERSION >= 3
#include <opencv2/core/ocl.hpp>
#endif

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

#include "BaslerTOF.h"

using namespace std;

namespace basler_bridge {

class BaslerTOFBridgeNodelet : public nodelet::Nodelet{

public:
    BaslerTOFBridgeNodelet();
    ~BaslerTOFBridgeNodelet();
    virtual void onInit();
    void connectCb();
    void sensorWorkerCallback();
    void tfWorkerCallback();
    bool intrinsicSaveCallback(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res);
    bool extrinsicSaveCallback(camera_msgs::Calibrate::Request &req, camera_msgs::Calibrate::Response &res);
    bool clearExtrinsicsCallback(camera_msgs::Clear::Request &req, camera_msgs::Clear::Response &res);
    bool triggerCallback(camera_msgs::Trigger::Request &req, camera_msgs::Trigger::Response &res);
    void drCallback(basler_bridge::basler_tof_bridgeConfig &config, uint32_t level);

    dynamic_reconfigure::Server<basler_bridge::basler_tof_bridgeConfig>* drServer_;
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

    BaslerTOF tofCamera_;

    ros::Publisher pubIntensity_;
    ros::Publisher pubIntensityRect_;
    ros::Publisher pubDepthRect_;
    ros::Publisher pubCalibrationPreview_;
    ros::Publisher pubCameraInfo_;
    bool camState_;
    bool publish_;
    int camCount_;

    float d0,d1,d2;

    std::map<int, boost::posix_time::ptime> clocks;
    void clockStart(int key = 0) { clocks[key] = boost::posix_time::microsec_clock::local_time(); }
    double clockEnd(int key = 0) { return fabs((float)(boost::posix_time::microsec_clock::local_time() - clocks[key]).total_milliseconds())/1000; }
};

BaslerTOFBridgeNodelet::BaslerTOFBridgeNodelet(){
    camState_ = false;
}

BaslerTOFBridgeNodelet::~BaslerTOFBridgeNodelet(){
    delete drServer_;
}

void BaslerTOFBridgeNodelet::onInit(){

    #if CV_MAJOR_VERSION >= 3
    cv::ocl::setUseOpenCL(true);
    cv::ocl::Context context;
    if (!context.create(cv::ocl::Device::TYPE_GPU))
    {
        cout << "Failed creating the context..." << endl;
        return;
    }
    cout << context.ndevices() << " GPU devices are detected." << endl;
    for (int i = 0; i < context.ndevices(); i++)
    {
        cv::ocl::Device device = context.device(i);
        cout << "name                 : " << device.name() << endl;
        cout << "available            : " << device.available() << endl;
        cout << "imageSupport         : " << device.imageSupport() << endl;
        cout << "OpenCL_C_Version     : " << device.OpenCL_C_Version() << endl;
        cout << endl;
    }
    #endif

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

    // TOF Params
    std::string settingsPath = ros::package::getPath("basler_bridge")+"/data/"+serialNumber_+"/settings.pfs";
    tofCamera_.setParams(serialNumber_, settingsPath);

    // Dynamic Reconfigure
    drServer_ = new dynamic_reconfigure::Server<basler_bridge::basler_tof_bridgeConfig>(privateNh);
    dynamic_reconfigure::Server<basler_bridge::basler_tof_bridgeConfig>::CallbackType f;
    f = boost::bind(&BaslerTOFBridgeNodelet::drCallback, this, _1, _2);
    drServer_->setCallback(f);

    // Publisher
    ros::SubscriberStatusCallback connect_cb = boost::bind(&BaslerTOFBridgeNodelet::connectCb, this);
    pubIntensity_ = nh.advertise<sensor_msgs::Image>(baslerName_+"/intensity", 1, connect_cb, connect_cb);
    pubIntensityRect_ = nh.advertise<sensor_msgs::Image>(baslerName_+"/intensity_rect", 1, connect_cb, connect_cb);
    pubDepthRect_ = nh.advertise<sensor_msgs::Image>(baslerName_+"/depth_rect", 1, connect_cb, connect_cb);
    pubCalibrationPreview_ = nh.advertise<sensor_msgs::Image>(baslerName_+"/calibration_preview", 1, connect_cb, connect_cb);
    pubCameraInfo_ = nh.advertise<sensor_msgs::CameraInfo>(baslerName_+"/camera_info", 1, connect_cb, connect_cb);

    // Calibration
    extrinsicCalibration_ = ExtrinsicCalibration(cv::Size(9,6), 0.108, ExtrinsicCalibration::CHESSBOARD);
    extrinsicCalibrateService_ = nh.advertiseService<camera_msgs::Calibrate::Request, camera_msgs::Calibrate::Response>(baslerName_ + "/Calibrate", boost::bind(&BaslerTOFBridgeNodelet::extrinsicSaveCallback, this, _1, _2));
    instrinsicCalibrateService_ = nh.advertiseService<sensor_msgs::SetCameraInfo::Request, sensor_msgs::SetCameraInfo::Response>(baslerName_ + "/set_camera_info", boost::bind(&BaslerTOFBridgeNodelet::intrinsicSaveCallback, this, _1, _2));
    clearExtrinsicsService_ = nh.advertiseService<camera_msgs::Clear::Request, camera_msgs::Clear::Response>(baslerName_ + "/Clear", boost::bind(&BaslerTOFBridgeNodelet::clearExtrinsicsCallback, this, _1, _2));
    triggerService_ = nh.advertiseService<camera_msgs::Trigger::Request, camera_msgs::Trigger::Response>(baslerName_ + "/Trigger", boost::bind(&BaslerTOFBridgeNodelet::triggerCallback, this, _1, _2));

    // Start worker
    sensorWorker_ = boost::thread(boost::bind(&BaslerTOFBridgeNodelet::sensorWorkerCallback, this));
    tfWorker_ = boost::thread(boost::bind(&BaslerTOFBridgeNodelet::tfWorkerCallback, this));

    // rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.108 image:=/TOF0/intensity camera:=/TOF0
    if(trigger_) camState_ = true;
}

void BaslerTOFBridgeNodelet::drCallback(basler_bridge::basler_tof_bridgeConfig &config, uint32_t level){
    ROS_INFO("drCallback");
    tofCamera_.addSettings("ConfidenceThreshold", config.confidence_threshold);
    tofCamera_.addSettings("FilterSpatial", config.spatial_filter);
    tofCamera_.addSettings("FilterTemporal", config.temporal_filter);
    tofCamera_.addSettings("FilterStrength", config.strength);
    tofCamera_.addSettings("OutlierTolerance", config.outlier_tolerance);
    d0 = config.d0;
    d1 = config.d1;
    d2 = config.d2;
}

bool BaslerTOFBridgeNodelet::triggerCallback(camera_msgs::Trigger::Request &req, camera_msgs::Trigger::Response &res){
    if(!trigger_) return false;
    if(req.trigger) publish_ = true;
    else publish_ = false;
    res.success = true;
    return true;
}

void BaslerTOFBridgeNodelet::connectCb()
{
    if(trigger_) return;

    if (pubIntensity_.getNumSubscribers() == 0 && pubIntensityRect_.getNumSubscribers() == 0 && pubDepthRect_.getNumSubscribers() == 0 && pubCameraInfo_.getNumSubscribers() == 0)
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

bool BaslerTOFBridgeNodelet::clearExtrinsicsCallback(camera_msgs::Clear::Request &req, camera_msgs::Clear::Response &res){
    cameraData_.clearExtrinsics("basler_bridge", serialNumber_);
    cameraData_.getCameraObject("basler_bridge", serialNumber_, cameraObject_);
    res.response = "Cleared";
    return true;
}

bool BaslerTOFBridgeNodelet::extrinsicSaveCallback(camera_msgs::Calibrate::Request &req, camera_msgs::Calibrate::Response &res){
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
    cv::Mat doubledInstrinsic = cameraObject_.cameraMatrixIR.clone();
    doubledInstrinsic.at<double>(0,0)*=2;
    doubledInstrinsic.at<double>(1,1)*=2;
    doubledInstrinsic.at<double>(0,2)*=2;
    doubledInstrinsic.at<double>(1,2)*=2;
    extrinsicCalibration_.setCameraMatrix(doubledInstrinsic);
    bool ok = extrinsicCalibration_.calibrateExtrinsics(calibrationPreviewImage_, extMat, req.square_shift, req.height_shift, Eigen::Vector3f(req.x_shift,req.y_shift,0));
    if(!ok){
        res.response = "Calibration failed";
        return false;
    }
    extrinsicCalibration_.checkCalibration(calibrationPreviewImage, extMat);

    // Save extrinsics
    cameraData_.saveTransforms("basler_bridge", serialNumber_, req.frame, extMat, req.direction);
    cameraData_.getCameraObject("basler_bridge", serialNumber_, cameraObject_);
    cout << extMat << endl;

    // Publish
    sensor_msgs::ImagePtr calibrationPreviewMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", calibrationPreviewImage).toImageMsg();
    pubCalibrationPreview_.publish(calibrationPreviewMsg);

    res.response = "Calibrated";
    return true;
}

bool BaslerTOFBridgeNodelet::intrinsicSaveCallback(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res){
    ROS_INFO("Calibrating");
    cameraData_.saveCameraMatrixIR("basler_bridge", serialNumber_, req.camera_info);
    cameraData_.getCameraObject("basler_bridge", serialNumber_, cameraObject_);
    res.status_message = "";
    res.success = true;
    return true;
}

void BaslerTOFBridgeNodelet::tfWorkerCallback(){
    while(1){
        // Send TF
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
        static tf::TransformBroadcaster br;
        for(std::map<std::string, tf::Transform>::iterator iterator = cameraObject_.cameraToFramesTF.begin(); iterator != cameraObject_.cameraToFramesTF.end(); iterator++) {
            std::string extFrame = iterator->first;
            tf::Transform transform = iterator->second;
            bool direction = cameraObject_.cameraToFramesDir[extFrame];
            if(direction){
                br.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), extFrame, baslerName_ + "_ir_optical_frame"));
            }else{
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),  baslerName_ + "_ir_optical_frame", extFrame));
            }
        }
        //ros::Time publishTime = ros::Time::now();
        //cameraObject_.cameraInfoIR.header.stamp = publishTime;
        //pubCameraInfo_.publish(cameraObject_.cameraInfoIR);
    }
}

void BaslerTOFBridgeNodelet::sensorWorkerCallback(){
    while(1){
        // Need some signal here
        if(!camState_){
            camCount_ = 0;
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            tofCamera_.stop();
            continue;
        }else{

            image_geometry::PinholeCameraModel irModel;
            irModel.fromCameraInfo(cameraObject_.cameraInfoIR);

            // Start TOF Camera and Grab images
            std::string settingsPath = ros::package::getPath("basler_bridge")+"/data/"+serialNumber_+"/settings.pfs";
            tofCamera_.start();
            if(!tofCamera_.grabImages()){
                ROS_ERROR("Failed to grab TOF images");
                continue;
            }

            //Get Range Map
            cv::Mat rangeMap = tofCamera_.getRangeMap().clone();

            // Range Correction (Should put all this on a GPU sometime)
            float depthMin = 0.;
            float depthMax = 13320.;
            cv::remap(rangeMap, rangeMap, cameraObject_.map1IR, cameraObject_.map2IR, cv::INTER_NEAREST);
            int i,j;
            uint16_t* p;
            #pragma omp parallel for collapse(2) shared(rangeMap)
            for(int v=0; v<rangeMap.rows; ++v){
                for(int u=0; u<rangeMap.cols; ++u){
                    uint16_t& val = rangeMap.at<uint16_t>(v,u);
                    float range = (depthMin + ( (float)val * ( depthMax - depthMin ) / 65535 )) * 0.001;
                    float theta = atan2((u-irModel.cx()),irModel.fx());
                    float phi = atan2(((v-irModel.cy())*cos(theta)),irModel.fy());
                    float z = range*cos(theta)*cos(phi);
                    val = (short)(z*1000);
                }
            }

            // Bilateral Filter (Should put all this on a GPU sometime)
            if(d0 > 0){
                #if CV_MAJOR_VERSION >= 3
                cv::Mat conv, conv2;
                rangeMap.convertTo(conv, CV_32FC1);
                cv::UMat convU = conv.getUMat(cv::ACCESS_READ);
                cv::UMat result;
                cv::bilateralFilter(convU, result, d0, d1, d2);
                conv2 = result.getMat(cv::ACCESS_READ);
                conv2.convertTo(rangeMap, CV_16UC1);
                conv2.release();
                #else
                cv::Mat conv, conv2;
                rangeMap.convertTo(conv, CV_32FC1);
                cv::bilateralFilter(conv, conv2, d0, d1, d2);
                conv2.convertTo(rangeMap, CV_16UC1);
                #endif
            }

            // Get Intensity Map
            cv::Mat intensityMap = tofCamera_.getIntensityMap();

            // Intensity Correction
            cv::Mat intensityRectMap = intensityMap.clone();
            cv::remap(intensityMap, intensityRectMap, cameraObject_.map1IR, cameraObject_.map2IR, cv::INTER_LINEAR);

            // For calibration store image
            calibrationPreviewMutex_.lock();
            calibrationPreviewImage_ = intensityRectMap;
            cv::resize(calibrationPreviewImage_, calibrationPreviewImage_, cv::Size(0,0), 2.0, 2.0);
            cv::Mat convertImage;
            cv::cvtColor(calibrationPreviewImage_, convertImage, CV_GRAY2BGR);
            calibrationPreviewImage_ = convertImage;
            calibrationPreviewMutex_.unlock();

            // Create image messages
            sensor_msgs::ImagePtr intensityImageMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", intensityMap).toImageMsg();
            sensor_msgs::ImagePtr intensityImageRectMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", intensityRectMap).toImageMsg();
            sensor_msgs::ImagePtr depthRectImageMsg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, rangeMap).toImageMsg();
            depthRectImageMsg->header.frame_id=baslerName_+"_ir_optical_frame";
            intensityImageRectMsg->header.frame_id=baslerName_+"_ir_optical_frame";
            cameraObject_.cameraInfoIR.header.frame_id=baslerName_+"_ir_optical_frame";

            // Publish
            ros::Time publishTime = ros::Time::now();
            intensityImageMsg->header.stamp = publishTime;
            intensityImageRectMsg->header.stamp = publishTime;
            depthRectImageMsg->header.stamp = publishTime;
            if(trigger_){
                if(!publish_) continue;
            }
            pubIntensity_.publish(intensityImageMsg);
            pubIntensityRect_.publish(intensityImageRectMsg);
            pubDepthRect_.publish(depthRectImageMsg);
            cameraObject_.cameraInfoIR.header.stamp = publishTime;
            pubCameraInfo_.publish(cameraObject_.cameraInfoIR);
            camCount_++;
        }
    }
}

}

//// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (basler_bridge, basler_tof_bridge, basler_bridge::BaslerTOFBridgeNodelet, nodelet::Nodelet);

