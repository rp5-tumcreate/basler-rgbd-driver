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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread/locks.hpp>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <camera_msgs/Calibrate.h>
#include <camera_msgs/Clear.h>
#include <camera_msgs/ExtrinsicsCalibrateStart.h>
#include <camera_msgs/ExtrinsicsCalibrateSignal.h>
#include <boost/thread.hpp>

#include "CameraData.h"
#include "ExtrinsicCalibration.h"
#include "ParamsFunctions.h"

#include "../registration/kinect2_registration.h"

#include <iomanip> // setprecision
#include <sstream> // stringstream

using namespace std;

namespace depth_registration {

using namespace message_filters::sync_policies;
namespace enc = sensor_msgs::image_encodings;

class DepthRegistrationNodelet : public nodelet::Nodelet{

public:

    struct CalibrationData{
        cv::Mat calibrationPreview;
        cv::Mat cameraMatrixFrom, cameraMatrixTo;
        std::string fromFrame, toFrame;
        std::vector<cv::Point2f> cameraImageFromCB, cameraImageToCB;
        cv::Mat cameraImageFromExt, cameraImageToExt;
        Eigen::VectorXf cameraImageFromExtRPYXYZ, cameraImageToExtRPYXYZ;
        cv::Size boardSize = cv::Size(9,6);
        float squareSize = 0.108;
    };

    DepthRegistrationNodelet();
    ~DepthRegistrationNodelet();
    virtual void onInit();
    void connectCb();
    void sensorWorkerCallback();
    void tfWorkerCallback();
    void calibrationCb(const sensor_msgs::ImageConstPtr& cameraImageFromMsg, const sensor_msgs::ImageConstPtr& cameraImageToMsg, const sensor_msgs::CameraInfoConstPtr& cameraInfoFromMsg, const sensor_msgs::CameraInfoConstPtr& cameraInfoToMsg);
    void registrationCb(const sensor_msgs::ImageConstPtr& cameraImageFromMsg, const sensor_msgs::ImageConstPtr& cameraImageToMsg, const sensor_msgs::CameraInfoConstPtr& cameraInfoFromMsg, const sensor_msgs::CameraInfoConstPtr& cameraInfoToMsg);

    bool extrinsicsCalibrateStartCallback(camera_msgs::ExtrinsicsCalibrateStart::Request &req, camera_msgs::ExtrinsicsCalibrateStart::Response &res);
    bool extrinsicsCalibrateSignalCallback(camera_msgs::ExtrinsicsCalibrateSignal::Request &req, camera_msgs::ExtrinsicsCalibrateSignal::Response &res);
    ros::ServiceServer extrinsicsCalibrateStartService_, extrinsicsCalibrateSignalService_;

    // Params
    int queueSize_;
    bool calibrationMode_;
    std::string cameraNameFrom_, cameraNameTo_;
    std::string cameraSerialFrom_, cameraSerialTo_;
    bool republishedRgb_;

    boost::mutex connectMutex_;
    message_filters::Subscriber<sensor_msgs::Image> subCameraImageFrom_;
    message_filters::Subscriber<sensor_msgs::Image> subCameraImageTo_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> subCameraInfoFrom_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> subCameraInfoTo_;
    typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    boost::shared_ptr<Synchronizer> sync_;
    CalibrationData calibrationData_;
    ros::Publisher pubCalibrationPreview_, pubCalibratedPreview_;
    boost::mutex calibrationDataMutex_;
    boost::condition_variable calibrationDataCond_;
    bool signalFlag_ = false;
    bool saveFlag_ = false;

    DepthRegistration *depthReg_ = NULL;
    ros::Publisher pubDepthRegistered_, pubRGBRegistered_;
    ros::Publisher pubCameraInfo_;
    cv::Size registeredResolution_;

    void setCalibrationData(const CalibrationData& calibrationData){
        boost::mutex::scoped_lock lock(calibrationDataMutex_);
        calibrationData_ = calibrationData;
        calibrationDataCond_.notify_one();
    }

    bool getCalibrationData(CalibrationData& calibrationData){
        boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(20000);
        boost::mutex::scoped_lock lock(calibrationDataMutex_);
        if(!calibrationDataCond_.timed_wait(lock, timeout)) return false;
        calibrationData = calibrationData_;
        return true;
    }
};

DepthRegistrationNodelet::DepthRegistrationNodelet(){
    //    camState_ = false;
}

DepthRegistrationNodelet::~DepthRegistrationNodelet(){
    //    delete drServer_;
}

void DepthRegistrationNodelet::onInit(){

    // Setup node handles
    ros::NodeHandle& nh         = getMTNodeHandle();
    ros::NodeHandle& privateNh = getMTPrivateNodeHandle();

    // Read Params
    republishedRgb_ = false;
    privateNh.getParam("queue_size", queueSize_);
    privateNh.getParam("calibration_mode", calibrationMode_);
    if(privateNh.hasParam("republished_rgb")){
        privateNh.getParam("republished_rgb", republishedRgb_);
    }

    // Calibration Mode
    if(calibrationMode_){
        privateNh.getParam("camera_name_from", cameraNameFrom_);
        privateNh.getParam("camera_name_to", cameraNameTo_);
        privateNh.getParam("camera_serial_from", cameraSerialFrom_);
        privateNh.getParam("camera_serial_to", cameraSerialTo_);

        // Sync
        sync_.reset( new Synchronizer(SyncPolicy(queueSize_), subCameraImageFrom_, subCameraImageTo_, subCameraInfoFrom_, subCameraInfoTo_) );
        sync_->registerCallback(boost::bind(&DepthRegistrationNodelet::calibrationCb, this, _1, _2, _3, _4));
        pubCalibrationPreview_ = privateNh.advertise<sensor_msgs::Image>("calibration_preview", 1);
        pubCalibratedPreview_ = privateNh.advertise<sensor_msgs::Image>("calibrated_preview", 1);

        // Calibration
        extrinsicsCalibrateStartService_ = privateNh.advertiseService<camera_msgs::ExtrinsicsCalibrateStart::Request, camera_msgs::ExtrinsicsCalibrateStart::Response>("extrinsics_calibrate_start", boost::bind(&DepthRegistrationNodelet::extrinsicsCalibrateStartCallback, this, _1, _2));
        extrinsicsCalibrateSignalService_ = privateNh.advertiseService<camera_msgs::ExtrinsicsCalibrateSignal::Request, camera_msgs::ExtrinsicsCalibrateSignal::Response>("extrinsics_calibrate_signal", boost::bind(&DepthRegistrationNodelet::extrinsicsCalibrateSignalCallback, this, _1, _2));

        // rosservice call /TOF0_depth_registration/extrinsics_calibrate_start "board_size: '9x6' square_size: 0.108"
    }
    // Non-Calibration Mode
    else{
        privateNh.getParam("camera_name_from", cameraNameFrom_);
        privateNh.getParam("camera_name_to", cameraNameTo_);
        std::vector<float> registeredResolutionFloat;
        privateNh.getParam("registered_resolution", registeredResolutionFloat);
        registeredResolution_.width = registeredResolutionFloat[0];
        registeredResolution_.height = registeredResolutionFloat[1];

        // Sync
        sync_.reset( new Synchronizer(SyncPolicy(queueSize_), subCameraImageFrom_, subCameraImageTo_, subCameraInfoFrom_, subCameraInfoTo_) );
        sync_->registerCallback(boost::bind(&DepthRegistrationNodelet::registrationCb, this, _1, _2, _3, _4));
        ros::SubscriberStatusCallback connect_cb = boost::bind(&DepthRegistrationNodelet::connectCb, this);
        boost::lock_guard<boost::mutex> lock(connectMutex_);
        pubCalibrationPreview_ = privateNh.advertise<sensor_msgs::Image>("calibration_preview", 1, connect_cb, connect_cb);
        pubDepthRegistered_ = nh.advertise<sensor_msgs::Image>(cameraNameTo_ + "/registered/depth_rect", 1, connect_cb, connect_cb);
        pubRGBRegistered_ = nh.advertise<sensor_msgs::Image>(cameraNameTo_ + "/registered/image_color_rect", 1, connect_cb, connect_cb);
        pubCameraInfo_ = nh.advertise<sensor_msgs::CameraInfo>(cameraNameTo_ + "/registered/camera_info", 1, connect_cb, connect_cb);
    }
}

//void DepthRegistrationNodelet::drCallback(basler_bridge::basler_rgb_bridgeConfig &config, uint32_t level){
//    ROS_INFO("drCallback");

//}

void DepthRegistrationNodelet::connectCb()
{
    ros::NodeHandle& nh         = getMTNodeHandle();
    ros::NodeHandle& privateNh = getMTPrivateNodeHandle();

    boost::lock_guard<boost::mutex> lock(connectMutex_);
    if(!calibrationMode_){
        if (pubCalibrationPreview_.getNumSubscribers() == 0 && pubDepthRegistered_.getNumSubscribers() == 0 && pubCameraInfo_.getNumSubscribers() == 0 && pubRGBRegistered_.getNumSubscribers() == 0)
        {
            ROS_INFO("Unsubscribing");
            subCameraImageFrom_.unsubscribe();
            subCameraImageTo_.unsubscribe();
            subCameraInfoFrom_.unsubscribe();
            subCameraInfoTo_.unsubscribe();
        }
        else
        {
            ROS_INFO("Subscribing");
            if(republishedRgb_)
                subCameraImageTo_.subscribe(nh, cameraNameTo_ + "/image_color_rect/republished", 1);
            else
                subCameraImageTo_.subscribe(nh, cameraNameTo_ + "/image_color_rect", 1);
            subCameraImageFrom_.subscribe(nh, cameraNameFrom_ + "/depth_rect", 1);
            subCameraInfoFrom_.subscribe(nh, cameraNameFrom_ + "/camera_info", 1);
            subCameraInfoTo_.subscribe(nh, cameraNameTo_ + "/camera_info", 1);
        }
    }
}

void DepthRegistrationNodelet::registrationCb(const sensor_msgs::ImageConstPtr& cameraImageFromMsg, const sensor_msgs::ImageConstPtr& cameraImageToMsg, const sensor_msgs::CameraInfoConstPtr& cameraInfoFromMsg, const sensor_msgs::CameraInfoConstPtr& cameraInfoToMsg)
{
    // Convert to CV Mat
    cv::Mat cameraImageFrom = ParamsFunctions::getMatFromROSMsg(cameraImageFromMsg);
    cv::Mat cameraImageTo = ParamsFunctions::getMatFromROSMsg(cameraImageToMsg);
    cv::Mat cameraMatrixFrom = ParamsFunctions::getCalibrationMatFromROSMsg(cameraInfoFromMsg);
    cv::Mat cameraMatrixTo = ParamsFunctions::getCalibrationMatFromROSMsg(cameraInfoToMsg);

    // Check to see if From is 16Bit and To is 8Bit RGB

    // Desired To Matrix
    cv::Size toResolution = cameraImageTo.size();
    cv::Mat cameraMatrixRegistered = cameraMatrixTo.clone();
    cameraMatrixRegistered.at<float>(0,0) = cameraMatrixTo.at<float>(0,0) * ((float)registeredResolution_.width/(float)toResolution.width);
    cameraMatrixRegistered.at<float>(0,2) = cameraMatrixTo.at<float>(0,2) * ((float)registeredResolution_.width/(float)toResolution.width);
    cameraMatrixRegistered.at<float>(1,1) = cameraMatrixTo.at<float>(1,1) * ((float)registeredResolution_.height/(float)toResolution.height);
    cameraMatrixRegistered.at<float>(1,2) = cameraMatrixTo.at<float>(1,2) * ((float)registeredResolution_.height/(float)toResolution.height);

    // Get Frame transform
    cv::Mat transform;
    try{
        transform = ParamsFunctions::getTransformTree(cameraInfoFromMsg->header.frame_id, cameraInfoToMsg->header.frame_id);
    }catch(...){
        ROS_ERROR("TF Error");
        return;
    }

    // Registration Module Setup Once
    if(depthReg_ == NULL){
        DepthRegistration::Method reg;
        reg = DepthRegistration::OPENCL;
        depthReg_ = DepthRegistration::New(reg);

        // Convert to required format
        cv::Mat rotation = cv::Mat(3,3,CV_64F);
        cv::Mat translation = cv::Mat(3,1,CV_64F);
        rotation.at<double>(0,0) = transform.at<float>(0,0); rotation.at<double>(0,1) = transform.at<float>(0,1); rotation.at<double>(0,2) = transform.at<float>(0,2);
        rotation.at<double>(1,0) = transform.at<float>(1,0); rotation.at<double>(1,1) = transform.at<float>(1,1); rotation.at<double>(1,2) = transform.at<float>(1,2);
        rotation.at<double>(2,0) = transform.at<float>(2,0); rotation.at<double>(2,1) = transform.at<float>(2,1); rotation.at<double>(2,2) = transform.at<float>(2,2);
        translation.at<double>(0,0) = transform.at<float>(0,3);
        translation.at<double>(1,0) = transform.at<float>(1,3);
        translation.at<double>(2,0) = transform.at<float>(2,3);
        cv::Mat cameraMatrixFromDouble;
        cameraMatrixFrom.convertTo(cameraMatrixFromDouble, CV_64F);
        cv::Mat cameraMatrixRegisteredDouble;
        cameraMatrixRegistered.convertTo(cameraMatrixRegisteredDouble, CV_64F);
        if(!depthReg_->init(cameraMatrixRegisteredDouble, registeredResolution_, cameraMatrixFromDouble, cameraImageFrom.size(), cv::Mat::zeros(8, 1, CV_64F), rotation, translation, 0.3f, 12.0, 0)){
            throw;
        }
    }

    // Register Depth
    cv::Mat depthRegistered;
    depthReg_->registerDepth(cameraImageFrom, depthRegistered);

    // Register RGB
    cv::Mat rgbRegistered;
    cv::resize(cameraImageTo, rgbRegistered, registeredResolution_);

    // Camera Info
    sensor_msgs::CameraInfo registeredCameraInfo;
    registeredCameraInfo.D = { 0, 0, 0, 0, 0 };
    registeredCameraInfo.K = {  cameraMatrixRegistered.at<float>(0,0), cameraMatrixRegistered.at<float>(0,1), cameraMatrixRegistered.at<float>(0,2),
                                cameraMatrixRegistered.at<float>(1,0), cameraMatrixRegistered.at<float>(1,1), cameraMatrixRegistered.at<float>(1,2),
                                cameraMatrixRegistered.at<float>(2,0), cameraMatrixRegistered.at<float>(2,1), cameraMatrixRegistered.at<float>(2,2),
                             };
    registeredCameraInfo.P = {  cameraMatrixRegistered.at<float>(0,0), cameraMatrixRegistered.at<float>(0,1), cameraMatrixRegistered.at<float>(0,2), 0,
                                cameraMatrixRegistered.at<float>(1,0), cameraMatrixRegistered.at<float>(1,1), cameraMatrixRegistered.at<float>(1,2), 0,
                                cameraMatrixRegistered.at<float>(2,0), cameraMatrixRegistered.at<float>(2,1), cameraMatrixRegistered.at<float>(2,2), 0,
                             };
    registeredCameraInfo.R = {  1,0,0,
                                0,1,0,
                                0,0,1,
                             };
    registeredCameraInfo.binning_x = 0;
    registeredCameraInfo.binning_y = 0;
    registeredCameraInfo.roi.x_offset = 0;
    registeredCameraInfo.roi.y_offset = 0;
    registeredCameraInfo.roi.height = 0;
    registeredCameraInfo.roi.width = 0;
    registeredCameraInfo.roi.do_rectify = false;
    registeredCameraInfo.distortion_model = "plumb_bob";
    registeredCameraInfo.width = registeredResolution_.width;
    registeredCameraInfo.height = registeredResolution_.height;

    // Publish
    ros::Time publishTime = ros::Time::now();
    sensor_msgs::ImagePtr registeredDepthMsg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, depthRegistered).toImageMsg();
    registeredDepthMsg->header.frame_id = cameraInfoToMsg->header.frame_id;
    registeredDepthMsg->header.stamp = publishTime;
    sensor_msgs::ImagePtr registeredRGBMsg = cv_bridge::CvImage(std_msgs::Header(), ParamsFunctions::getROSType(rgbRegistered), rgbRegistered).toImageMsg();
    registeredRGBMsg->header.frame_id = cameraInfoToMsg->header.frame_id;
    registeredRGBMsg->header.stamp = publishTime;
    registeredCameraInfo.header.frame_id = cameraInfoToMsg->header.frame_id;
    registeredCameraInfo.header.stamp = publishTime;
    pubDepthRegistered_.publish(registeredDepthMsg);
    pubRGBRegistered_.publish(registeredRGBMsg);
    pubCameraInfo_.publish(registeredCameraInfo);

    // Do the transform test
//    int v,u;
//    uint16_t* p;
//    for( v = 0; v < cameraImageFrom.rows; ++v)
//    {
//        p = cameraImageFrom.ptr<uint16_t>(v);
//        for ( u = 0; u < cameraImageFrom.cols; ++u)
//        {
//            float z = (float)p[u] * 0.001;
//            float y = (((float)v - cameraMatrixFrom.at<float>(1,2)) * z) / (cameraMatrixFrom.at<float>(1,1));
//            float x = (((float)u - cameraMatrixFrom.at<float>(0,2)) * z) / (cameraMatrixFrom.at<float>(0,0));
//            float nx, ny, nz;
//            nx = x*transform.at<float>(0,0) + y*transform.at<float>(0,1) + z*transform.at<float>(0,2) + transform.at<float>(0,3);
//            ny = x*transform.at<float>(1,0) + y*transform.at<float>(1,1) + z*transform.at<float>(1,2) + transform.at<float>(1,3);
//            nz = x*transform.at<float>(2,0) + y*transform.at<float>(2,1) + z*transform.at<float>(2,2) + transform.at<float>(2,3);
//            int nu, nv;
//            nu = (int)(((cameraMatrixRegistered.at<float>(0,0)*nx)/nz) + cameraMatrixRegistered.at<float>(0,2));
//            nv = (int)(((cameraMatrixRegistered.at<float>(1,1)*ny)/nz) + cameraMatrixRegistered.at<float>(1,2));
//            if(nu < 0 || nu >= rgbRegistered.cols || nv < 0 || nv >= rgbRegistered.rows) continue;
//            cv::Vec3b& color = rgbRegistered.at<cv::Vec3b>(cv::Point2i(nu,nv));
//            color[0] = 254;
//            color[1] = 0;
//            color[2] = 254;
//        }
//    }
//    sensor_msgs::ImagePtr previewMsg = cv_bridge::CvImage(std_msgs::Header(), ParamsFunctions::getROSType(rgbRegistered), rgbRegistered).toImageMsg();
//    pubCalibrationPreview_.publish(previewMsg);
}

void DepthRegistrationNodelet::calibrationCb(const sensor_msgs::ImageConstPtr& cameraImageFromMsg, const sensor_msgs::ImageConstPtr& cameraImageToMsg, const sensor_msgs::CameraInfoConstPtr& cameraInfoFromMsg, const sensor_msgs::CameraInfoConstPtr& cameraInfoToMsg)
{
    static ParamsFunctions::iterable_queue<float> eucDistanceQueue({0,0,0});

    // Convert to CV Mat
    cv::Mat cameraImageFrom = ParamsFunctions::getMatFromROSMsg(cameraImageFromMsg, 2.0);
    cv::Mat cameraImageTo = ParamsFunctions::getMatFromROSMsg(cameraImageToMsg);
    cv::Mat cameraMatrixFrom = ParamsFunctions::getCalibrationMatFromROSMsg(cameraInfoFromMsg, 2.0);
    cv::Mat cameraMatrixTo = ParamsFunctions::getCalibrationMatFromROSMsg(cameraInfoToMsg);

    // Checkerboard
    CalibrationData calibrationData;
    cv::Mat cameraImageFromDraw, cameraImageToDraw;
    try{
        cout << "cb" << endl;
        calibrationData.fromFrame = cameraImageFromMsg->header.frame_id;
        calibrationData.toFrame = cameraImageToMsg->header.frame_id;
        calibrationData.cameraMatrixFrom = cameraMatrixFrom;
        calibrationData.cameraMatrixTo = cameraMatrixTo;
        calibrationData.cameraImageFromCB = ParamsFunctions::findCheckerBoardPoints(cameraImageFrom, cameraImageFromDraw, calibrationData.boardSize);
        calibrationData.cameraImageToCB = ParamsFunctions::findCheckerBoardPoints(cameraImageTo, cameraImageToDraw, calibrationData.boardSize);
        calibrationData.cameraImageFromExt = ParamsFunctions::findTransform(calibrationData.cameraImageFromCB, calibrationData.cameraMatrixFrom, cv::Mat::zeros(8, 1, CV_64F), calibrationData.boardSize, calibrationData.squareSize);
        calibrationData.cameraImageFromExtRPYXYZ = ParamsFunctions::getRPYXYZ(ParamsFunctions::eigenFromMat(calibrationData.cameraImageFromExt));
        calibrationData.cameraImageToExt = ParamsFunctions::findTransform(calibrationData.cameraImageToCB, calibrationData.cameraMatrixTo, cv::Mat::zeros(8, 1, CV_64F), calibrationData.boardSize, calibrationData.squareSize);
        calibrationData.cameraImageToExtRPYXYZ = ParamsFunctions::getRPYXYZ(ParamsFunctions::eigenFromMat(calibrationData.cameraImageToExt));
        calibrationData.calibrationPreview = ParamsFunctions::imageSideBySide(cameraImageFromDraw, cameraImageToDraw);
        cout << "done" << endl;

        // Stability
        float distance = sqrt(
                    pow(calibrationData.cameraImageFromExtRPYXYZ[3],2) +
                pow(calibrationData.cameraImageFromExtRPYXYZ[4],2) +
                pow(calibrationData.cameraImageFromExtRPYXYZ[5],2)
                );
        eucDistanceQueue.pop();
        eucDistanceQueue.push(distance);
        float std = eucDistanceQueue.stdev();
        if(std > 0.005){
            ROS_WARN("Unstable pattern");
            return;
        }

        // Send Data
        setCalibrationData(calibrationData);

        // Publish Preview
        sensor_msgs::ImagePtr calibrationPreviewMsg = cv_bridge::CvImage(std_msgs::Header(), ParamsFunctions::getROSType(calibrationData.calibrationPreview), calibrationData.calibrationPreview).toImageMsg();
        pubCalibrationPreview_.publish(calibrationPreviewMsg);

        ROS_INFO("Found pattern");
    }catch(cv::Exception& e){
        cerr << e.what() << endl;
        ROS_WARN("Failed to find pattern");
        return;
    }
}

bool DepthRegistrationNodelet::extrinsicsCalibrateStartCallback(camera_msgs::ExtrinsicsCalibrateStart::Request &req, camera_msgs::ExtrinsicsCalibrateStart::Response &res){

    ros::NodeHandle& nh         = getMTNodeHandle();
    ros::NodeHandle& privateNh = getMTPrivateNodeHandle();

    // Subscribe
    subCameraImageFrom_.subscribe(nh, cameraNameFrom_ + "/intensity_rect", 1);
    subCameraImageTo_.subscribe(nh, cameraNameTo_ + "/image_color_rect", 1);
    subCameraInfoFrom_.subscribe(nh, cameraNameFrom_ + "/camera_info", 1);
    subCameraInfoTo_.subscribe(nh, cameraNameTo_ + "/camera_info", 1);

    std::vector<CalibrationData> calibrationDatas;
    float rollRange, pitchRange, yawRange, xRange, yRange, zRange;

    int64_t begin, end;
    begin = clock();
    while(1){

        boost::this_thread::sleep(boost::posix_time::milliseconds(100));

        // Check for stop service
        if(signalFlag_){
            signalFlag_ = false;

            // Save Data
            if(saveFlag_){
                saveFlag_ = false;

                if(calibrationDatas.size() < 1){
                    ROS_ERROR("Need at least 1 point to save");
                    continue;
                }

                CalibrationData& sampleCalibrationData = calibrationDatas[0];
                std::vector<std::vector<cv::Point3f> > pointsBoard = ParamsFunctions::getObjectPoints(sampleCalibrationData.boardSize, sampleCalibrationData.squareSize, calibrationDatas.size());
                std::vector<std::vector<cv::Point2f> > pointsFrom;
                std::vector<std::vector<cv::Point2f> > pointsTo;
                for(CalibrationData& existingCalibrationData : calibrationDatas){
                    pointsFrom.push_back(existingCalibrationData.cameraImageFromCB);
                    pointsTo.push_back(existingCalibrationData.cameraImageToCB);
                }

                ROS_INFO("Calibrating");
                cv::Mat rotation, translation, essential, fundamental, disparity;
                const cv::TermCriteria termCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON);
                double error;
                cv::Mat emptyDistortion = cv::Mat::zeros(8, 1, CV_64F);

                #if CV_MAJOR_VERSION == 2
                    error = cv::stereoCalibrate(pointsBoard, pointsFrom, pointsTo, sampleCalibrationData.cameraMatrixFrom, emptyDistortion, sampleCalibrationData.cameraMatrixTo, emptyDistortion, cv::Size(),
                                                rotation, translation, essential, fundamental, termCriteria, cv::CALIB_FIX_INTRINSIC);
                #elif CV_MAJOR_VERSION == 3
                    error = cv::stereoCalibrate(pointsBoard, pointsFrom, pointsTo, sampleCalibrationData.cameraMatrixFrom, emptyDistortion, sampleCalibrationData.cameraMatrixTo, emptyDistortion, cv::Size(),
                                                rotation, translation, essential, fundamental, cv::CALIB_FIX_INTRINSIC, termCriteria);
                #endif

                cout << "re-projection error: " << error << std::endl;
                cout << "Rotation:" << std::endl << rotation << std::endl;
                cout << "Translation:" << std::endl << translation << std::endl;
                cout << "Essential:" << std::endl << essential << std::endl;
                cout << "Fundamental:" << std::endl << fundamental << std::endl;

                cv::Mat extMat = cv::Mat(4, 4, CV_64FC1);
                extMat.at<double>(0, 0) = rotation.at<double>(0, 0); extMat.at<double>(0, 1) = rotation.at<double>(0, 1); extMat.at<double>(0, 2) = rotation.at<double>(0, 2); extMat.at<double>(0, 3) = translation.at<double>(0, 0);
                extMat.at<double>(1, 0) = rotation.at<double>(1, 0); extMat.at<double>(1, 1) = rotation.at<double>(1, 1); extMat.at<double>(1, 2) = rotation.at<double>(1, 2); extMat.at<double>(1, 3) = translation.at<double>(1, 0);
                extMat.at<double>(2, 0) = rotation.at<double>(2, 0); extMat.at<double>(2, 1) = rotation.at<double>(2, 1); extMat.at<double>(2, 2) = rotation.at<double>(2, 2); extMat.at<double>(2, 3) = translation.at<double>(2, 0);
                extMat.at<double>(3, 0) = 0.0; extMat.at<double>(3, 1) = 0.0; extMat.at<double>(3, 2) = 0.0; extMat.at<double>(3, 3) = 1.0;
                extMat = extMat.inv();

                CameraData cameraData;
                cameraData.saveTransforms("basler_bridge", cameraSerialFrom_, sampleCalibrationData.toFrame, extMat, 1);
                cout << extMat << endl;

                break;
            }
            // Get more data
            else{

                // Get Data
                CalibrationData newCalibrationData;
                if(!getCalibrationData(newCalibrationData)){
                    cout << "ignoring" << endl;
                    continue;
                }

                // Add data if not the same
                bool newMatch = true;
                if(!calibrationDatas.size()){
                    ROS_INFO("Adding data");
                    calibrationDatas.push_back(newCalibrationData);
                }else{
                    for(CalibrationData& existingCalibrationData : calibrationDatas){
                        float eucDistance = sqrt(
                                    pow(newCalibrationData.cameraImageFromExtRPYXYZ[3]-existingCalibrationData.cameraImageFromExtRPYXYZ[3],2) +
                                pow(newCalibrationData.cameraImageFromExtRPYXYZ[4]-existingCalibrationData.cameraImageFromExtRPYXYZ[4],2) +
                                pow(newCalibrationData.cameraImageFromExtRPYXYZ[5]-existingCalibrationData.cameraImageFromExtRPYXYZ[5],2)
                                );
                        float rotDistance = sqrt(
                                    pow(newCalibrationData.cameraImageFromExtRPYXYZ[0]-existingCalibrationData.cameraImageFromExtRPYXYZ[0],2) +
                                pow(newCalibrationData.cameraImageFromExtRPYXYZ[1]-existingCalibrationData.cameraImageFromExtRPYXYZ[1],2) +
                                pow(newCalibrationData.cameraImageFromExtRPYXYZ[2]-existingCalibrationData.cameraImageFromExtRPYXYZ[2],2)
                                );
                        if(eucDistance < 0.2 && rotDistance < 10) newMatch = false;
                    }
                    if(newMatch){
                        ROS_INFO("Adding data");
                        calibrationDatas.push_back(newCalibrationData);
                    }
                }

                // Test range
                if(newMatch){
                    if(calibrationDatas.size() >= 2){
                        float rollMax = -1000., pitchMax = -1000., yawMax = -1000., xMax = -1000., yMax = -1000., zMax = -1000.;
                        float rollMin = 1000., pitchMin = 1000., yawMin = 1000., xMin = 1000., yMin = 1000., zMin = 1000.;
                        for(CalibrationData& existingCalibrationData : calibrationDatas){
                            if(existingCalibrationData.cameraImageFromExtRPYXYZ[0] > rollMax) rollMax = existingCalibrationData.cameraImageFromExtRPYXYZ[0];
                            if(existingCalibrationData.cameraImageFromExtRPYXYZ[1] > pitchMax) pitchMax = existingCalibrationData.cameraImageFromExtRPYXYZ[1];
                            if(existingCalibrationData.cameraImageFromExtRPYXYZ[2] > yawMax) yawMax = existingCalibrationData.cameraImageFromExtRPYXYZ[2];
                            if(existingCalibrationData.cameraImageFromExtRPYXYZ[3] > xMax) xMax = existingCalibrationData.cameraImageFromExtRPYXYZ[3];
                            if(existingCalibrationData.cameraImageFromExtRPYXYZ[4] > yMax) yMax = existingCalibrationData.cameraImageFromExtRPYXYZ[4];
                            if(existingCalibrationData.cameraImageFromExtRPYXYZ[5] > zMax) zMax = existingCalibrationData.cameraImageFromExtRPYXYZ[5];
                            if(existingCalibrationData.cameraImageFromExtRPYXYZ[0] < rollMin) rollMin = existingCalibrationData.cameraImageFromExtRPYXYZ[0];
                            if(existingCalibrationData.cameraImageFromExtRPYXYZ[1] < pitchMin) pitchMin = existingCalibrationData.cameraImageFromExtRPYXYZ[1];
                            if(existingCalibrationData.cameraImageFromExtRPYXYZ[2] < yawMin) yawMin = existingCalibrationData.cameraImageFromExtRPYXYZ[2];
                            if(existingCalibrationData.cameraImageFromExtRPYXYZ[3] < xMin) xMin = existingCalibrationData.cameraImageFromExtRPYXYZ[3];
                            if(existingCalibrationData.cameraImageFromExtRPYXYZ[4] < yMin) yMin = existingCalibrationData.cameraImageFromExtRPYXYZ[4];
                            if(existingCalibrationData.cameraImageFromExtRPYXYZ[5] < zMin) zMin = existingCalibrationData.cameraImageFromExtRPYXYZ[5];
                        }
                        rollRange = rollMax - rollMin;
                        pitchRange = pitchMax - pitchMin;
                        yawRange = yawMax - yawMin;
                        xRange = xMax - xMin;
                        yRange = yMax - yMin;
                        zRange = zMax - zMin;

                        // Modify Image
                        stringstream stream;
                        stream << fixed << setprecision(2) << "RANGE: " << rollRange << " " << pitchRange << " " << yawRange << " " << xRange << " " << yRange << " " << zRange << " COUNT: " << calibrationDatas.size();
                        std::string outputString = stream.str();
                        cout << outputString << endl;
                        cv::putText(newCalibrationData.calibrationPreview, outputString, cv::Point(30,30), CV_FONT_HERSHEY_PLAIN, 2.0, CV_RGB(0,0,250), 2);
                    }

                    // Send Preview
                    sensor_msgs::ImagePtr calibratedPreviewMsg = cv_bridge::CvImage(std_msgs::Header(), ParamsFunctions::getROSType(newCalibrationData.calibrationPreview), newCalibrationData.calibrationPreview).toImageMsg();
                    pubCalibratedPreview_.publish(calibratedPreviewMsg);
                }
                else{
                    ROS_WARN("Move the camera");
                }

            }

        }

        double elapsed = ((double)(clock() - begin) / CLOCKS_PER_SEC);
        //cout << "data received " << elapsed << endl;
        //if(elapsed > 300) break;
    }

    ROS_INFO("Unsubscribing");
    subCameraImageFrom_.unsubscribe();
    subCameraImageTo_.unsubscribe();
    subCameraInfoFrom_.unsubscribe();
    subCameraInfoTo_.unsubscribe();

    return true;
}

bool DepthRegistrationNodelet::extrinsicsCalibrateSignalCallback(camera_msgs::ExtrinsicsCalibrateSignal::Request &req, camera_msgs::ExtrinsicsCalibrateSignal::Response &res){
    signalFlag_ = true;
    saveFlag_ = req.save;
    return true;
}

}

//// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (depth_registration, depth_registration, depth_registration::DepthRegistrationNodelet, nodelet::Nodelet);

