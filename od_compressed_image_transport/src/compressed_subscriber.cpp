
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
#include <od_compressed_image_transport/compressed_subscriberConfig.h>
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

class CompressedSubscriberNodelet : public nodelet::Nodelet{

public:

    int queueSize_;
    std::string compressedTopicInput_, rawTopicOutput_;

    dynamic_reconfigure::Server<od_compressed_image_transport::compressed_subscriberConfig>* drServer_;
    od_compressed_image_transport::compressed_subscriberConfig config_;

    boost::mutex connectMutex_;
    message_filters::Subscriber<sensor_msgs::CompressedImage> compressedTopicSub_;
    ros::Publisher rawTopicPub_;
    int imdecode_flag_;
    bool retime_;

    CompressedSubscriberNodelet(){

    }

    ~CompressedSubscriberNodelet(){
        delete drServer_;
    }

    virtual void onInit(){

        // Setup node handles
        ros::NodeHandle& nh         = getMTNodeHandle();
        ros::NodeHandle& privateNh = getMTPrivateNodeHandle();

        // Read Params
        retime_ = false;
        privateNh.getParam("queue_size", queueSize_);
        privateNh.getParam("compressed_topic_input", compressedTopicInput_);
        privateNh.getParam("raw_topic_output", rawTopicOutput_);
        if(privateNh.hasParam("retime")){
            privateNh.getParam("retime", retime_);
        }

        // Dynamic Reconfigure
        drServer_ = new dynamic_reconfigure::Server<od_compressed_image_transport::compressed_subscriberConfig>(privateNh);
        dynamic_reconfigure::Server<od_compressed_image_transport::compressed_subscriberConfig>::CallbackType f;
        f = boost::bind(&CompressedSubscriberNodelet::drCallback, this, _1, _2);
        drServer_->setCallback(f);

        // Connection
        ros::SubscriberStatusCallback connect_cb = boost::bind(&CompressedSubscriberNodelet::connectCb, this);
        boost::lock_guard<boost::mutex> lock(connectMutex_);
        compressedTopicSub_.registerCallback(&CompressedSubscriberNodelet::callback, this);
        rawTopicPub_ = nh.advertise<sensor_msgs::Image>(rawTopicOutput_, 1, connect_cb, connect_cb);
    }

    void drCallback(od_compressed_image_transport::compressed_subscriberConfig &config, uint32_t level){
        config_ = config;
        if (config_.mode == "gray") {
            imdecode_flag_ = cv::IMREAD_GRAYSCALE;
        } else if (config_.mode == "color") {
            imdecode_flag_ = cv::IMREAD_COLOR;
        } else /*if (config_.mode == compressed_image_transport::CompressedSubscriber_unchanged)*/ {
            imdecode_flag_ = cv::IMREAD_UNCHANGED;
        }
    }

    void connectCb(){
        ros::NodeHandle& nh = getMTNodeHandle();
        boost::lock_guard<boost::mutex> lock(connectMutex_);
        if (rawTopicPub_.getNumSubscribers() == 0)
        {
            compressedTopicSub_.unsubscribe();
        }
        else
        {
            compressedTopicSub_.subscribe(nh, compressedTopicInput_, 1);
        }
    }

    void callback(const sensor_msgs::CompressedImageConstPtr& message){
        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

        // Copy message header
        cv_ptr->header = message->header;

        // Decode color/mono image
        try
        {
            cv_ptr->image = cv::imdecode(cv::Mat(message->data), imdecode_flag_);

            // Assign image encoding string
            const size_t split_pos = message->format.find(';');
            if (split_pos==std::string::npos)
            {
                // Older version of compressed_image_transport does not signal image format
                switch (cv_ptr->image.channels())
                {
                case 1:
                    cv_ptr->encoding = enc::MONO8;
                    break;
                case 3:
                    cv_ptr->encoding = enc::BGR8;
                    break;
                default:
                    ROS_ERROR("Unsupported number of channels: %i", cv_ptr->image.channels());
                    break;
                }
            } else
            {
                std::string image_encoding = message->format.substr(0, split_pos);

                cv_ptr->encoding = image_encoding;

                if ( enc::isColor(image_encoding))
                {
                    std::string compressed_encoding = message->format.substr(split_pos);
                    bool compressed_bgr_image = (compressed_encoding.find("compressed bgr") != std::string::npos);

                    // Revert color transformation
                    if (compressed_bgr_image)
                    {
                        // if necessary convert colors from bgr to rgb
                        if ((image_encoding == enc::RGB8) || (image_encoding == enc::RGB16))
                            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGB);

                        if ((image_encoding == enc::RGBA8) || (image_encoding == enc::RGBA16))
                            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGBA);

                        if ((image_encoding == enc::BGRA8) || (image_encoding == enc::BGRA16))
                            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2BGRA);
                    } else
                    {
                        // if necessary convert colors from rgb to bgr
                        if ((image_encoding == enc::BGR8) || (image_encoding == enc::BGR16))
                            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGR);

                        if ((image_encoding == enc::BGRA8) || (image_encoding == enc::BGRA16))
                            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGRA);

                        if ((image_encoding == enc::RGBA8) || (image_encoding == enc::RGBA16))
                            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2RGBA);
                    }
                }
            }
        }
        catch (cv::Exception& e)
        {
            ROS_ERROR("%s", e.what());
        }

        size_t rows = cv_ptr->image.rows;
        size_t cols = cv_ptr->image.cols;

        if ((rows > 0) && (cols > 0)){
            // Publish message to user callback
            sensor_msgs::ImagePtr publish = cv_ptr->toImageMsg();
            if(retime_) publish->header.stamp = ros::Time::now();
            rawTopicPub_.publish(publish);
        }

    }
};

} //namespace compressed_image_transport

//// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (od_compressed_image_transport, compressed_subscriber, od_compressed_image_transport::CompressedSubscriberNodelet, nodelet::Nodelet);
