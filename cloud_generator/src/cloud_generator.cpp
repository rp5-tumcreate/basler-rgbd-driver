#include <string>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <cloud_generator/cloud_generatorConfig.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread/locks.hpp>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <camera_msgs/Calibrate.h>
#include <camera_msgs/Clear.h>
#include <boost/thread.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include "CameraData.h"
#include "ExtrinsicCalibration.h"
#include "ParamsFunctions.h"

#define __CL_ENABLE_EXCEPTIONS
#include <CL/cl.hpp>
#include <getopt.h>

using namespace std;

namespace cloud_generator {

typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloud;

class CloudGeneratorProcessing{
public:

    const std::string path = ros::package::getPath("cloud_generator");

    // Struct for GPU Processing
    typedef struct Params
    {
        int height, width;
        float fx,fy,cx,cy;
        float r00, r01, r02, t0,
        r10, r11, r12, t1,
        r20, r21, r22, t2;
        float xRange, yRange, zRange;
    } Params;

    // CL Params
    cl::Device device_;
    cl::Context context_;
    cl::CommandQueue queue_;

    // OpenCL Buffers for point cloud generation
    cl::Buffer bufferParams_;
    cl::Buffer bufferDepth_;
    cl::Buffer bufferEdge_;
    cl::Buffer bufferCloudOutput_;
    cl::Buffer bufferCloudEdgeOutput_;

    // CL Routines
    typedef cl::make_kernel <cl::Buffer&, cl::Buffer&, cl::Buffer&> kernelCloudType;
    cl::Program kernelCloudProgram_;
    std::function<kernelCloudType::type_> kernelCloud_;
    typedef cl::make_kernel <cl::Buffer&, cl::Buffer&, int, int> kernelGaussianType;
    cl::Program kernelGaussianProgram_;
    std::function<kernelGaussianType::type_> kernelGaussian_;
    typedef cl::make_kernel <cl::Buffer&, cl::Buffer&, cl::Buffer&, int, int> kernelSobelType;
    cl::Program kernelSobelProgram_;
    std::function<kernelSobelType::type_> kernelSobel_;
    typedef cl::make_kernel <cl::Buffer&, cl::Buffer&, cl::Buffer&, int, int> kernelNMSType;
    cl::Program kernelNMSProgram_;
    std::function<kernelNMSType::type_> kernelNMS_;
    typedef cl::make_kernel <cl::Buffer&, cl::Buffer&, int, int> kernelHystType;
    cl::Program kernelHystProgram_;
    std::function<kernelHystType::type_> kernelHyst_;
    typedef cl::make_kernel <cl::Buffer&, cl::Buffer&, cl::Buffer&, cl::Buffer&> kernelCloudEdgeType;
    cl::Program kernelCloudEdgeProgram_;
    std::function<kernelCloudEdgeType::type_> kernelCloudEdge_;

    // OpenCL Buffers for edge generation
    int workgroup_size_ = 16;
    size_t buffer_index_ = 0;
    cl::Buffer bufferTheta_;
    cl::Buffer bufferEdges_[2];
    inline cl::Buffer& NextEdgeBuff() { return bufferEdges_[buffer_index_]; }
    inline cl::Buffer& PrevEdgeBuff() { return bufferEdges_[buffer_index_ ^ 1]; }
    inline void AdvanceEdgeBuff() { buffer_index_ ^= 1; }

    // Variables
    cv::Mat depthImg_, rgbImg_, rgbImgDepthSize_, grayImg_, edgeImg_;
    Params cameraParams_;

    // Flags
    bool gpuCloud_ = true;
    bool gpuEdge_ = true;
    bool cpuOnly_ = false;

    unsigned getDeviceList(std::vector<cl::Device>& devices)
    {
        cl_int err;

        // Get list of platforms
        std::vector<cl::Platform> platforms;
        cl::Platform::get(&platforms);

        // Enumerate devices
        for (int i = 0; i < platforms.size(); i++)
        {
            cl_uint num = 0;
            std::vector<cl::Device> plat_devices;
            platforms[i].getDevices(CL_DEVICE_TYPE_ALL, &plat_devices);
            devices.insert(devices.end(), plat_devices.begin(), plat_devices.end());
        }

        return devices.size();
    }

    inline std::string loadProgram(std::string input)
    {
        std::ifstream stream(input.c_str());
        if (!stream.is_open()) {
            std::cout << "Cannot open file: " << input << std::endl;
            exit(1);
        }
         return std::string(
            std::istreambuf_iterator<char>(stream),
            (std::istreambuf_iterator<char>()));
    }

    void getDeviceName(cl::Device& device, std::string& name)
    {
        cl_device_info info = CL_DEVICE_NAME;

        // Special case for AMD
        #ifdef CL_DEVICE_BOARD_NAME_AMD
        device.getInfo(CL_DEVICE_VENDOR, &name);
        if (strstr(name.c_str(), "Advanced Micro Devices"))
            info = CL_DEVICE_BOARD_NAME_AMD;
        #endif

        device.getInfo(info, &name);
    }

    CloudGeneratorProcessing(){
        try{
            cl_uint deviceIndex = 0;
            std::vector<cl::Device> devices;
            unsigned numDevices = getDeviceList(devices);
            if (deviceIndex >= numDevices){
                std::cout << "Invalid device index (try '--list')\n";
                return;
            }
            this->device_ = devices[deviceIndex];

            // Find Device
            std::vector<cl::Device> chosen_device;
            chosen_device.push_back(device_);
            this->context_ = cl::Context(chosen_device);
            this->queue_ = cl::CommandQueue(context_, device_);
            std::string name;
            getDeviceName(this->device_, name);
            std::cout << "Using OpenCL device: " << name << "\n";

            // Compile GPU Code
            this->kernelCloudProgram_ = cl::Program(this->context_, loadProgram(path + "/src/cloud_kernel.cl"), true);
            this->kernelCloud_ = kernelCloudType(this->kernelCloudProgram_, "cloud_kernel");
            this->kernelGaussianProgram_ = cl::Program(this->context_, loadProgram(path + "/src/gaussian_kernel.cl"), true);
            this->kernelGaussian_ = kernelGaussianType(this->kernelGaussianProgram_, "gaussian_kernel");
            this->kernelSobelProgram_ = cl::Program(this->context_, loadProgram(path + "/src/sobel_kernel.cl"), true);
            this->kernelSobel_ = kernelSobelType(this->kernelSobelProgram_, "sobel_kernel");
            this->kernelNMSProgram_ = cl::Program(this->context_, loadProgram(path + "/src/non_max_supp_kernel.cl"), true);
            this->kernelNMS_ = kernelNMSType(this->kernelNMSProgram_, "non_max_supp_kernel");
            this->kernelHystProgram_ = cl::Program(this->context_, loadProgram(path + "/src/hyst_kernel.cl"), true);
            this->kernelHyst_ = kernelHystType(this->kernelHystProgram_, "hyst_kernel");
            this->kernelCloudEdgeProgram_ = cl::Program(this->context_, loadProgram(path + "/src/cloud_edge_kernel.cl"), true);
            this->kernelCloudEdge_ = kernelCloudEdgeType(this->kernelCloudEdgeProgram_, "cloud_edge_kernel");
            std::cout << "OpenCL Code Compiled" << "\n";

        } catch (cl::Error err)
        {
            std::cerr << "Exception: " << err.err() << endl;
            std::cerr << "Switching to CPU only" << std::endl;
            gpuCloud_ = false;
            gpuEdge_ = false;
            cpuOnly_ = true;
            throw;
        }
    }

    ~CloudGeneratorProcessing(){

    }

    void loadData(const cv::Mat &depthImg, const cv::Mat &rgbImg, Params cameraParams){
        cv::resize(rgbImg, this->rgbImgDepthSize_, cv::Size(depthImg.cols, depthImg.rows));
        cv::cvtColor(rgbImg,this->grayImg_,CV_BGR2GRAY);
        int rows = ((grayImg_.rows - 2) / workgroup_size_) * workgroup_size_ + 2;
        int cols = ((grayImg_.cols - 2) / workgroup_size_) * workgroup_size_ + 2;
        cv::resize(this->grayImg_, this->grayImg_, cv::Size(cols, rows));

        this->depthImg_ = depthImg;
        this->rgbImg_ = rgbImg;
        this->cameraParams_ = cameraParams;
        if(gpuCloud_ || gpuEdge_){
            this->bufferParams_ = cl::Buffer(this->context_, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(Params), &cameraParams_, NULL);
            this->bufferDepth_ = cl::Buffer(this->context_, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, depthImg_.rows * depthImg_.cols * sizeof(uint16_t), depthImg_.data, NULL);
            this->bufferCloudOutput_ = cl::Buffer(this->context_, CL_MEM_WRITE_ONLY, depthImg_.rows * depthImg_.cols * sizeof(float) * 3);
            this->bufferCloudEdgeOutput_ = cl::Buffer(this->context_, CL_MEM_WRITE_ONLY, depthImg_.rows * depthImg_.cols * sizeof(float) * 3);
            this->bufferTheta_ = cl::Buffer(context_, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR, grayImg_.rows * grayImg_.cols * grayImg_.elemSize());
            NextEdgeBuff() = cl::Buffer(this->context_, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR | CL_MEM_COPY_HOST_PTR, grayImg_.rows * grayImg_.cols * grayImg_.elemSize(), grayImg_.data);
            PrevEdgeBuff() = cl::Buffer(context_, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR, grayImg_.rows * grayImg_.cols * grayImg_.elemSize());
            AdvanceEdgeBuff();
        }
    }

    void generatePointCloud(PointCloud::Ptr &cloud){
        if(gpuCloud_){
            try{
                cl::NDRange global(cameraParams_.width, cameraParams_.height);
                this->kernelCloud_(cl::EnqueueArgs(this->queue_, global), this->bufferParams_, this->bufferDepth_, this->bufferCloudOutput_);
                this->queue_.finish();

                float* cloudOutput = new float[depthImg_.rows * depthImg_.cols * sizeof(float) * 3];
                this->queue_.enqueueReadBuffer(this->bufferCloudOutput_, CL_TRUE, 0, depthImg_.rows * depthImg_.cols * sizeof(float) * 3, cloudOutput);

                pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator pt_iter = cloud->begin();
                for(int v=0; v<depthImg_.rows; v++){
                    for(int u=0; u<depthImg_.cols; u++){
                        pcl::PointXYZRGBNormal& pt = *pt_iter++;
                        int index = v*cameraParams_.width + u;
                        pt.x = cloudOutput[index*3 + 0];
                        pt.y = cloudOutput[index*3 + 1];
                        pt.z = cloudOutput[index*3 + 2];
                        pt.r = rgbImgDepthSize_.data[index*3 + 2];
                        pt.g = rgbImgDepthSize_.data[index*3 + 1];
                        pt.b = rgbImgDepthSize_.data[index*3 + 0];
                    }
                }

                delete cloudOutput;
            }catch (cl::Error err)
            {
                std::cerr << "Exception: " << err.err() << endl;
            }

        }else{

            float bad_point = std::numeric_limits<float>::quiet_NaN ();
            const ushort* depth_data = reinterpret_cast<const ushort*>(depthImg_.data);
            PointCloud::iterator pt_iter = cloud->begin();

            for(int v=0; v<cameraParams_.height; v++){
                for(int u=0; u<cameraParams_.width; u++){
                    pcl::PointXYZRGBNormal& pt = *pt_iter++;
                    int index = v*cameraParams_.width + u;
                    const ushort d = depth_data[index];
                    if(d != 0){
                        const float z = (float)d*0.001;
                        const float y = (((float)v - cameraParams_.cy) * z) / (cameraParams_.fy);
                        const float x = (((float)u - cameraParams_.cx) * z) / (cameraParams_.fx);
                        const float xt = x*cameraParams_.r00 + y*cameraParams_.r01 + z*cameraParams_.r02 + cameraParams_.t0;
                        const float yt = x*cameraParams_.r10 + y*cameraParams_.r11 + z*cameraParams_.r12 + cameraParams_.t1;
                        const float zt = x*cameraParams_.r20 + y*cameraParams_.r21 + z*cameraParams_.r22 + cameraParams_.t2;
                        if(xt < -cameraParams_.xRange || xt > cameraParams_.xRange || yt < -cameraParams_.yRange || yt > cameraParams_.yRange || zt < cameraParams_.zRange){
                            pt.x = pt.y = pt.z = bad_point;
                        }else{
                            pt.x = xt;
                            pt.y = yt;
                            pt.z = zt;
                            pt.r = rgbImgDepthSize_.data[index*3 + 2];
                            pt.g = rgbImgDepthSize_.data[index*3 + 1];
                            pt.b = rgbImgDepthSize_.data[index*3 + 0];
                        }
                    }else{
                        pt.x = pt.y = pt.z = bad_point;
                    }
                }
            }

        }
    }

    void edgeOperator(cv::Mat& sobelImg){
        if(gpuEdge_){

            try{
                this->kernelGaussian_(
                    cl::EnqueueArgs(this->queue_,
                        cl::NDRange(1, 1),
                        cl::NDRange(grayImg_.rows - 2, grayImg_.cols - 2),
                        cl::NDRange(workgroup_size_, workgroup_size_)
                    ),
                    PrevEdgeBuff(), NextEdgeBuff(), grayImg_.rows, grayImg_.cols
                );
                this->queue_.finish();
                AdvanceEdgeBuff();

                this->kernelSobel_(
                    cl::EnqueueArgs(this->queue_,
                        cl::NDRange(1, 1),
                        cl::NDRange(grayImg_.rows - 2, grayImg_.cols - 2),
                        cl::NDRange(workgroup_size_, workgroup_size_)
                    ),
                    PrevEdgeBuff(), NextEdgeBuff(), bufferTheta_, grayImg_.rows, grayImg_.cols
                );
                this->queue_.finish();
                AdvanceEdgeBuff();

                sobelImg = cv::Mat(grayImg_.rows, grayImg_.cols, CV_8UC1, cv::Scalar(0));
                this->queue_.enqueueReadBuffer(PrevEdgeBuff(), CL_TRUE, 0, grayImg_.rows * grayImg_.cols * sizeof(uint8_t), sobelImg.data);
                cv::resize(sobelImg, sobelImg, cv::Size(rgbImg_.cols, rgbImg_.rows));

            }catch (cl::Error err)
            {
                std::cerr << "Exception: " << err.err() << endl;
                sobelImg = cv::Mat(rgbImg_.rows, rgbImg_.cols, CV_8UC1, cv::Scalar(0));
            }
        }else{
            cv::Mat blurredImg;
            cv::GaussianBlur(grayImg_, blurredImg, cv::Size(3,3), 0, 0);
            cv::Mat grad_x, grad_y, grad;
            cv::Mat abs_grad_x, abs_grad_y;
            cv::Sobel( blurredImg, grad_x, CV_16S, 1, 0, 5, 0.1, 0, cv::BORDER_DEFAULT );
            cv::convertScaleAbs( grad_x, abs_grad_x );
            cv::Sobel( blurredImg, grad_y, CV_16S, 0, 1, 5, 0.1, 0, cv::BORDER_DEFAULT );
            cv::convertScaleAbs( grad_y, abs_grad_y );
            cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, sobelImg );
            cv::resize(sobelImg, sobelImg, cv::Size(rgbImg_.cols, rgbImg_.rows));
        }
    }

    void temporalDepth(cv::Mat& depthImage){


    }

};

using namespace message_filters::sync_policies;
namespace enc = sensor_msgs::image_encodings;

class CloudGeneratorNodelet : public nodelet::Nodelet{

public:

    CloudGeneratorNodelet();
    ~CloudGeneratorNodelet();
    virtual void onInit();
    void connectCb();
    void cloudCb(const sensor_msgs::ImageConstPtr& depthMsg, const sensor_msgs::ImageConstPtr& rgbMsg, const sensor_msgs::CameraInfoConstPtr& depthInfoMsg, const sensor_msgs::CameraInfoConstPtr& rgbInfoMsg);
    void drCallback(cloud_generator::cloud_generatorConfig &config, uint32_t level);

    // Params
    int queueSize_;
    bool temporalFilter_;
    float temporalError_;
    bool unorganize_;
    bool republish_;
    dynamic_reconfigure::Server<cloud_generator::cloud_generatorConfig>* drServer_;

    boost::mutex connectMutex_;
    message_filters::Subscriber<sensor_msgs::Image> subDepth_;
    message_filters::Subscriber<sensor_msgs::Image> subRgb_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> subDepthInfo_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> subRgbInfo_;
    typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    boost::shared_ptr<Synchronizer> sync_;
    cloud_generator::cloud_generatorConfig config_;

    ros::Publisher pubDepthRepublished_, pubRgbRepublished_;
    ros::Publisher pubEdge_, pubCloud_;

    std::string subDepthTopic_, subRgbTopic_, subDepthInfoTopic_, subRgbInfoTopic_;
    std::string pubDepthRepublishedTopic_, pubRgbRepublishedTopic_;
    std::string pubEdgeTopic_, pubCloudTopic_;

    std::vector<cv::Mat> depthQueue_;
    CloudGeneratorProcessing cloudGeneratorProcessing_;
    image_geometry::PinholeCameraModel model_;
    std::string tfTarget_, tfSource_;

    std::map<int, boost::posix_time::ptime> clocks;
    void clockStart(int key = 0) { clocks[key] = boost::posix_time::microsec_clock::local_time(); }
    double clockEnd(int key = 0) { return fabs((float)(boost::posix_time::microsec_clock::local_time() - clocks[key]).total_milliseconds())/1000; }
};

CloudGeneratorNodelet::CloudGeneratorNodelet(){
    //    camState_ = false;
}

CloudGeneratorNodelet::~CloudGeneratorNodelet(){
    delete drServer_;
}

void CloudGeneratorNodelet::onInit(){

    // Setup node handles
    ros::NodeHandle& nh         = getMTNodeHandle();
    ros::NodeHandle& privateNh = getMTPrivateNodeHandle();

    // Read Params
    privateNh.getParam("queue_size", queueSize_);
    privateNh.getParam("temporal_filter", temporalFilter_);
    privateNh.getParam("temporal_error", temporalError_);
    privateNh.getParam("unorganize", unorganize_);
    privateNh.getParam("republish", republish_);
    privateNh.getParam("tf_target", tfTarget_);
    privateNh.getParam("sub_depth_topic", subDepthTopic_);
    privateNh.getParam("sub_rgb_topic", subRgbTopic_);
    privateNh.getParam("sub_depth_info_topic", subDepthInfoTopic_);
    privateNh.getParam("sub_rgb_info_topic", subRgbInfoTopic_);
    privateNh.getParam("pub_edge_topic", pubEdgeTopic_);
    privateNh.getParam("pub_cloud_topic", pubCloudTopic_);

    // Republish Topics
    if(republish_){
        privateNh.getParam("pub_depth_republish_topic", pubDepthRepublishedTopic_);
        privateNh.getParam("pub_rgb_republish_topic", pubRgbRepublishedTopic_);
    }

    // Dynamic Reconfigure
    drServer_ = new dynamic_reconfigure::Server<cloud_generator::cloud_generatorConfig>(privateNh);
    dynamic_reconfigure::Server<cloud_generator::cloud_generatorConfig>::CallbackType f;
    f = boost::bind(&CloudGeneratorNodelet::drCallback, this, _1, _2);
    drServer_->setCallback(f);

    // Publisher
    SyncPolicy sp(queueSize_);
    //sp.setMaxIntervalDuration(ros::Duration(100.f / 1000));
    //sp.setAgePenalty(0.5);
    //sp.setInterMessageLowerBound(0, ros::Duration(0.1));
    sync_.reset( new Synchronizer(SyncPolicy(sp), subDepth_, subRgb_, subDepthInfo_, subRgbInfo_) );
    sync_->registerCallback(boost::bind(&CloudGeneratorNodelet::cloudCb, this, _1, _2, _3, _4));
    ros::SubscriberStatusCallback connect_cb = boost::bind(&CloudGeneratorNodelet::connectCb, this);
    boost::lock_guard<boost::mutex> lock(connectMutex_);
    if(republish_){
        pubDepthRepublished_ = nh.advertise<sensor_msgs::Image>(pubDepthRepublishedTopic_, 1, connect_cb, connect_cb);
        pubRgbRepublished_ = nh.advertise<sensor_msgs::Image>(pubRgbRepublishedTopic_, 1, connect_cb, connect_cb);
    }
    pubEdge_ = nh.advertise<sensor_msgs::Image>(pubEdgeTopic_, 1, connect_cb, connect_cb);
    pubCloud_ = nh.advertise<PointCloud>(pubCloudTopic_, 1, connect_cb, connect_cb);
}

void CloudGeneratorNodelet::drCallback(cloud_generator::cloud_generatorConfig &config, uint32_t level){
    ROS_INFO("drCallback");
    config_ = config;
}

void CloudGeneratorNodelet::connectCb()
{
    ros::NodeHandle& nh         = getMTNodeHandle();
    ros::NodeHandle& privateNh = getMTPrivateNodeHandle();

    boost::lock_guard<boost::mutex> lock(connectMutex_);
    if(republish_){
        if (pubEdge_.getNumSubscribers() == 0 && pubCloud_.getNumSubscribers() == 0 && pubDepthRepublished_.getNumSubscribers() == 0 && pubRgbRepublished_.getNumSubscribers() == 0)
        {
            ROS_INFO("Unsubscribing");
            subDepth_.unsubscribe();
            subRgb_.unsubscribe();
            subDepthInfo_.unsubscribe();
            subRgbInfo_.unsubscribe();
            depthQueue_.clear();
        }
        else
        {
            ROS_INFO("Subscribing");
            subDepth_.subscribe(nh, subDepthTopic_, 1);
            subRgb_.subscribe(nh, subRgbTopic_, 1);
            subDepthInfo_.subscribe(nh, subDepthInfoTopic_, 1);
            subRgbInfo_.subscribe(nh, subRgbInfoTopic_, 1);
        }
    }else{
        if (pubEdge_.getNumSubscribers() == 0 && pubCloud_.getNumSubscribers() == 0)
        {
            ROS_INFO("Unsubscribing");
            subDepth_.unsubscribe();
            subRgb_.unsubscribe();
            subDepthInfo_.unsubscribe();
            subRgbInfo_.unsubscribe();
            depthQueue_.clear();
        }
        else
        {
            ROS_INFO("Subscribing");
            subDepth_.subscribe(nh, subDepthTopic_, 1);
            subRgb_.subscribe(nh, subRgbTopic_, 1);
            subDepthInfo_.subscribe(nh, subDepthInfoTopic_, 1);
            subRgbInfo_.subscribe(nh, subRgbInfoTopic_, 1);
        }
    }
}

void CloudGeneratorNodelet::cloudCb(const sensor_msgs::ImageConstPtr& depthMsg, const sensor_msgs::ImageConstPtr& rgbMsg, const sensor_msgs::CameraInfoConstPtr& depthInfoMsg, const sensor_msgs::CameraInfoConstPtr& rgbInfoMsg)
{
    //clockStart();

    // Check for bad inputs
    if (depthMsg->header.frame_id != rgbMsg->header.frame_id)
    {
        NODELET_ERROR_THROTTLE(5, "Depth image frame id [%s] doesn't match RGB image frame id [%s]",
                               depthMsg->header.frame_id.c_str(), rgbMsg->header.frame_id.c_str());
        return;
    }

    // Update camera model
    model_.fromCameraInfo(depthInfoMsg);

    // Images
    cv::Mat rgbImg, depthImg, sobelImg, cannyImg;

    // Get RGB Image and get Depth Image
    rgbImg = cv_bridge::toCvCopy(rgbMsg, enc::TYPE_8UC3)->image;
    depthImg = cv_bridge::toCvCopy(depthMsg, enc::TYPE_16UC1)->image;

    // Get transform world
    tfSource_ = depthMsg->header.frame_id;
    static tf::TransformListener listener;
    tf::StampedTransform transform;
    bool s = listener.waitForTransform(tfTarget_, tfSource_, ros::Time(0), ros::Duration(10));
    if(s) listener.lookupTransform(tfTarget_, tfSource_, ros::Time(0), transform); else{
        cout << "Frame Error" << endl;
    }
    tf::Quaternion q = transform.getRotation();
    tf::Vector3 t = transform.getOrigin();
    t.setX(t.getX());
    t.setY(t.getY());
    t.setZ(t.getZ());
    tf::Matrix3x3 rot;
    rot.setRotation(q);

    // Cloud Gen Params
    if(!cloudGeneratorProcessing_.cpuOnly_){
        cloudGeneratorProcessing_.gpuCloud_ = config_.gpuCloud;
        cloudGeneratorProcessing_.gpuEdge_ = config_.gpuEdge;
    }

    // Hack (Temporal Filter) (Should be on GPU!)
    if(temporalFilter_){
        cv::Mat depthImgCleaned = depthImg.clone();
        static const int queueSize = 4;
        if(depthQueue_.size() < queueSize-1){
            depthQueue_.push_back(depthImg);
            return;
        }else{
            depthQueue_.push_back(depthImg);
            #pragma omp parallel for collapse(2) shared(depthImgCleaned)
            for(int v=0; v<depthImg.rows; ++v){
                for(int u=0; u<depthImg.cols; ++u){

                    // New Algo
                    float arr[queueSize];
                    for(int i=0; i<queueSize; i++){
                        arr[i] = ((float)depthQueue_[i].at<ushort>(v,u))*0.001;
                    }
                    std::sort(std::begin(arr), std::end(arr));
                    float median = arr[queueSize/2];
                    if(!median){
                        depthImgCleaned.at<ushort>(v,u) = 0;
                        continue;
                    }
                    float meansum = 0;
                    int meancount = 0;
                    for(int i=0; i<queueSize; i++){
                        if(fabs(arr[i]-median) < temporalError_){
                            meancount++;
                            meansum+=arr[i];
                        }
                    }
                    if(meancount <= 2){
                        depthImgCleaned.at<ushort>(v,u) = 0;
                        continue;
                    }
                    float mean = meansum/meancount;
                    depthImgCleaned.at<ushort>(v,u) = (ushort)(mean*1000);

                }
            }
            depthQueue_.erase(depthQueue_.begin());
        }
        depthImg = depthImgCleaned;
    }

    // Load data into processing
    CloudGeneratorProcessing::Params cameraParams;
    cameraParams.height = depthMsg->height;
    cameraParams.width = depthMsg->width;
    cameraParams.fx = (float)model_.fx();
    cameraParams.fy = (float)model_.fy();
    cameraParams.cx = (float)model_.cx();
    cameraParams.cy = (float)model_.cy();
    cameraParams.r00 = (float)rot[0][0]; cameraParams.r01 = (float)rot[0][1]; cameraParams.r02 = (float)rot[0][2]; cameraParams.t0 = (float)t[0] + config_.xShift;
    cameraParams.r10 = (float)rot[1][0]; cameraParams.r11 = (float)rot[1][1]; cameraParams.r12 = (float)rot[1][2]; cameraParams.t1 = (float)t[1] + config_.yShift;
    cameraParams.r20 = (float)rot[2][0]; cameraParams.r21 = (float)rot[2][1]; cameraParams.r22 = (float)rot[2][2]; cameraParams.t2 = (float)t[2] + config_.zShift;
    cameraParams.xRange = config_.xRange;
    cameraParams.yRange = config_.yRange;
    cameraParams.zRange = config_.zRange;
    cloudGeneratorProcessing_.loadData(depthImg, rgbImg, cameraParams);

    // Allocate new point cloud messages
    PointCloud::Ptr cloudMsg (new PointCloud);
    cloudMsg->header = pcl_conversions::toPCL(depthMsg->header);
    cloudMsg->height = depthMsg->height;
    cloudMsg->width  = depthMsg->width;
    cloudMsg->is_dense = false;
    cloudMsg->points.resize (cloudMsg->height * cloudMsg->width);
    cloudMsg->header.frame_id = tfTarget_;

    // Generate Point Cloud
    cloudGeneratorProcessing_.generatePointCloud(cloudMsg);

    // Make unorganized
    if(unorganize_){
        PointCloud::Ptr cloudMsgUnorganized (new PointCloud);
        cloudMsgUnorganized->header = pcl_conversions::toPCL(depthMsg->header);
        cloudMsgUnorganized->header.frame_id = tfTarget_;
        for(pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator it = cloudMsg->begin(); it!= cloudMsg->end(); it++){
            pcl::PointXYZRGBNormal& point = *it;
            if(pcl::isFinite(point)) cloudMsgUnorganized->points.push_back(point);
        }
        cloudMsgUnorganized->is_dense = true;
        cloudMsg = cloudMsgUnorganized;
    }

    // Generate Sobel
    //cloudGeneratorProcessing_.edgeOperator(sobelImg);
    //sensor_msgs::ImagePtr edgeImageMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", sobelImg).toImageMsg();

    // Publish
    ros::Time publishTime = ros::Time::now();
    pcl_conversions::toPCL(publishTime, cloudMsg->header.stamp);
    //edgeImageMsg->header.stamp = publishTime;
    //edgeImageMsg->header.frame_id = tfSource_;
    pubCloud_.publish (cloudMsg);
    //pubEdge_.publish(edgeImageMsg);

    // Republish
    if(republish_){
        sensor_msgs::ImagePtr depthImageMsg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depthImg).toImageMsg();
        sensor_msgs::ImagePtr rgbImageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgbImg).toImageMsg();
        depthImageMsg->header.frame_id = tfSource_;
        depthImageMsg->header.stamp = publishTime;
        rgbImageMsg->header.frame_id = tfSource_;
        rgbImageMsg->header.stamp = publishTime;
        pubDepthRepublished_.publish(depthImageMsg);
        pubRgbRepublished_.publish(rgbImageMsg);
    }

    //cout << clockEnd() << endl;
}

}

//// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (cloud_generator, cloud_generator, cloud_generator::CloudGeneratorNodelet, nodelet::Nodelet);

//                    // Old Algo
//                    float largestError = 0.;
//                    float agree = true;
//                    for(int i=0; i<depthQueue_.size(); ++i){
//                        for(int j=i; j<depthQueue_.size(); ++j){
//                            if(i==j) continue;
//                            float error = (float)(abs(depthQueue_[i].at<ushort>(v,u) - depthQueue_[j].at<ushort>(v,u)))*0.001;
//                            if(error > largestError) largestError = error;
//                            if(error > temporalError_) agree = false;
//                        }
//                    }
//                    if(!agree && largestError > 0.5){
//                        depthImgCleaned.at<ushort>(v,u) = 0;
//                    }else{
//                        int sum = 0;
//                        for(int i=0; i<depthQueue_.size(); i++){
//                            sum+=depthQueue_[i].at<ushort>(v,u);
//                        }
//                        depthImgCleaned.at<ushort>(v,u) = sum/depthQueue_.size();
//                    }

