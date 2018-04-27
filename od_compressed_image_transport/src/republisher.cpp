
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

class RepublisherNodelet : public nodelet::Nodelet{

public:

    int queueSize_;
    std::string topicInput_, topicOutput_;

    boost::mutex connectMutex_;
    message_filters::Subscriber<sensor_msgs::Image> inputTopicSub_;
    ros::Publisher outputTopicPub_;
    int imdecode_flag_;

    RepublisherNodelet(){

    }

    ~RepublisherNodelet(){
    }

    virtual void onInit(){

        // Setup node handles
        ros::NodeHandle& nh         = getMTNodeHandle();
        ros::NodeHandle& privateNh = getMTPrivateNodeHandle();

        // Read Params
        privateNh.getParam("queue_size", queueSize_);
        privateNh.getParam("topic_input", topicInput_);
        privateNh.getParam("topic_output", topicOutput_);

        // Connection
        ros::SubscriberStatusCallback connect_cb = boost::bind(&RepublisherNodelet::connectCb, this);
        boost::lock_guard<boost::mutex> lock(connectMutex_);
        inputTopicSub_.registerCallback(&RepublisherNodelet::callback, this);
        outputTopicPub_ = nh.advertise<sensor_msgs::Image>(topicOutput_, 1, connect_cb, connect_cb);
    }

    void connectCb(){
        ros::NodeHandle& nh = getMTNodeHandle();
        boost::lock_guard<boost::mutex> lock(connectMutex_);
        if (outputTopicPub_.getNumSubscribers() == 0)
        {
            inputTopicSub_.unsubscribe();
        }
        else
        {
            inputTopicSub_.subscribe(nh, topicInput_, 1);
        }
    }

    void callback(const sensor_msgs::ImageConstPtr& message){
        sensor_msgs::Image outputMessage = *message;
        ros::Time publishTime = ros::Time::now();
        outputMessage.header.stamp = publishTime;
        outputTopicPub_.publish(outputMessage);
    }
};

} //namespace compressed_image_transport

//// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (od_compressed_image_transport, republisher, od_compressed_image_transport::RepublisherNodelet, nodelet::Nodelet);
