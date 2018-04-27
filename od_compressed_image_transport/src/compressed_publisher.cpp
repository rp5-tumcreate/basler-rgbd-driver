
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/make_shared.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <dynamic_reconfigure/server.h>
#include <od_compressed_image_transport/compressed_publisherConfig.h>
#include <boost/thread/locks.hpp>
#include <message_filters/subscriber.h>

#include <vector>
#include <sstream>

using namespace cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings;

namespace od_compressed_image_transport
{

// Compression formats
enum compressionFormat
{
    UNDEFINED = -1, JPEG, PNG
};

class CompressedPublisherNodelet : public nodelet::Nodelet{

public:

    int queueSize_;
    std::string rawTopicInput_, compressedTopicOutput_;

    dynamic_reconfigure::Server<od_compressed_image_transport::compressed_publisherConfig>* drServer_;
    od_compressed_image_transport::compressed_publisherConfig config_;

    boost::mutex connectMutex_;
    message_filters::Subscriber<sensor_msgs::Image> rawTopicSub_;
    ros::Publisher compressedTopicPub_;
    bool retime_;

    CompressedPublisherNodelet(){

    }

    ~CompressedPublisherNodelet(){
        delete drServer_;
    }

    virtual void onInit(){

        // Setup node handles
        ros::NodeHandle& nh         = getMTNodeHandle();
        ros::NodeHandle& privateNh = getMTPrivateNodeHandle();

        // Read Params
        retime_ = false;
        privateNh.getParam("queue_size", queueSize_);
        privateNh.getParam("raw_topic_input", rawTopicInput_);
        privateNh.getParam("compressed_topic_output", compressedTopicOutput_);
        if(privateNh.hasParam("retime")){
            privateNh.getParam("retime", retime_);
        }

        // Dynamic Reconfigure
        drServer_ = new dynamic_reconfigure::Server<od_compressed_image_transport::compressed_publisherConfig>(privateNh);
        dynamic_reconfigure::Server<od_compressed_image_transport::compressed_publisherConfig>::CallbackType f;
        f = boost::bind(&CompressedPublisherNodelet::drCallback, this, _1, _2);
        drServer_->setCallback(f);

        // Connection
        ros::SubscriberStatusCallback connect_cb = boost::bind(&CompressedPublisherNodelet::connectCb, this);
        boost::lock_guard<boost::mutex> lock(connectMutex_);
        rawTopicSub_.registerCallback(&CompressedPublisherNodelet::callback, this);
        compressedTopicPub_ = nh.advertise<sensor_msgs::CompressedImage>(compressedTopicOutput_, 1, connect_cb, connect_cb);
    }

    void drCallback(od_compressed_image_transport::compressed_publisherConfig &config, uint32_t level){
        config_ = config;
    }

    void connectCb(){
        ros::NodeHandle& nh = getMTNodeHandle();
        boost::lock_guard<boost::mutex> lock(connectMutex_);
        if (compressedTopicPub_.getNumSubscribers() == 0)
        {
            rawTopicSub_.unsubscribe();
        }
        else
        {
            rawTopicSub_.subscribe(nh, rawTopicInput_, 1);
        }
    }

    void callback(const sensor_msgs::Image& message){

        // Compressed image message
        sensor_msgs::CompressedImage compressed;
        compressed.header = message.header;
        compressed.format = message.encoding;

        // Compression settings
        std::vector<int> params;
        params.resize(3, 0);

        // Get codec configuration
        compressionFormat encodingFormat = UNDEFINED;
        if (config_.format == "jpeg")
            encodingFormat = JPEG;
        if (config_.format == "png")
            encodingFormat = PNG;

        // Bit depth of image encoding
        int bitDepth = enc::bitDepth(message.encoding);
        int numChannels = enc::numChannels(message.encoding);

        switch (encodingFormat)
        {
        // JPEG Compression
        case JPEG:
        {
            params[0] = CV_IMWRITE_JPEG_QUALITY;
            params[1] = config_.jpeg_quality;

            // Update ros message format header
            compressed.format += "; jpeg compressed";

            // Check input format
            if ((bitDepth == 8) && // JPEG only works on 8bit images
                    ((numChannels == 1) || (numChannels == 3)))
            {

                // Target image format
                stringstream targetFormat;
                if (enc::isColor(message.encoding))
                {
                    // convert color images to RGB domain
                    targetFormat << "rgb" << bitDepth;
                }

                // OpenCV-ros bridge
                try
                {
                    boost::shared_ptr<CompressedPublisherNodelet> tracked_object;
                    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(message, tracked_object, targetFormat.str());

                    // Compress image
                    if (cv::imencode(".jpg", cv_ptr->image, compressed.data, params))
                    {

                        float cRatio = (float)(cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize())
                                / (float)compressed.data.size();
                        ROS_DEBUG("Compressed Image Transport - Codec: jpg, Compression: 1:%.2f (%lu bytes)", cRatio, compressed.data.size());
                    }
                    else
                    {
                        ROS_ERROR("cv::imencode (jpeg) failed on input image");
                    }
                }
                catch (cv_bridge::Exception& e)
                {
                    ROS_ERROR("%s", e.what());
                }
                catch (cv::Exception& e)
                {
                    ROS_ERROR("%s", e.what());
                }

                // Publish message
                if(retime_) compressed.header.stamp = ros::Time::now();
                compressedTopicPub_.publish(compressed);
            }
            else
                ROS_ERROR("Compressed Image Transport - JPEG compression requires 8-bit, 1/3-channel images (input format is: %s)", message.encoding.c_str());

            break;
        }
            // PNG Compression
        case PNG:
        {
            params[0] = CV_IMWRITE_PNG_COMPRESSION;
            params[1] = config_.png_level;

            // Update ros message format header
            compressed.format += "; png compressed";

            // Check input format
            if (((bitDepth == 16) || (bitDepth == 8)) && ((numChannels == 1) || (numChannels == 3)))
            {

                // Target image format
                stringstream targetFormat;
                if (enc::isColor(message.encoding))
                {
                    // convert color images to RGB domain
                    targetFormat << "rgb" << bitDepth;
                }

                // OpenCV-ros bridge
                try
                {
                    boost::shared_ptr<CompressedPublisherNodelet> tracked_object;
                    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(message, tracked_object, targetFormat.str());

                    // Compress image
                    if (cv::imencode(".png", cv_ptr->image, compressed.data, params))
                    {

                        float cRatio = (float)(cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize())
                                / (float)compressed.data.size();
                        ROS_DEBUG("Compressed Image Transport - Codec: png, Compression: 1:%.2f (%lu bytes)", cRatio, compressed.data.size());
                    }
                    else
                    {
                        ROS_ERROR("cv::imencode (png) failed on input image");
                    }
                }
                catch (cv_bridge::Exception& e)
                {
                    ROS_ERROR("%s", e.what());
                }
                catch (cv::Exception& e)
                {
                    ROS_ERROR("%s", e.what());
                }

                // Publish message
                if(retime_) compressed.header.stamp = ros::Time::now();
                compressedTopicPub_.publish(compressed);
            }
            else
                ROS_ERROR("Compressed Image Transport - PNG compression requires 8/16-bit, 1/3-channel images (input format is: %s)", message.encoding.c_str());
            break;
        }

        default:
            ROS_ERROR("Unknown compression type '%s', valid options are 'jpeg' and 'png'", config_.format.c_str());
            break;
        }
    }
};

} //namespace compressed_image_transport

//// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (od_compressed_image_transport, compressed_publisher, od_compressed_image_transport::CompressedPublisherNodelet, nodelet::Nodelet);
