#ifndef PARAMS_FUNCTIONS_H
#define PARAMS_FUNCTIONS_H

#include <string>
#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <image_geometry/pinhole_camera_model.h>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <queue>
#include <deque>
#include <numeric>


using namespace std;

class ParamsFunctions
{
public:
    template <class T, class container=std::deque<T> >
    class iterable_queue {
        container data;
    public:
        typedef typename container::iterator iterator;
        explicit iterable_queue(std::initializer_list<T> t) : data(t) {}
        void push(T t) { data.push_back(t); }
        void pop() { data.pop_front(); }
        T front() const { return data.front(); }
        bool empty() const { return data.empty(); }
        iterator begin() { return this->data.begin(); }
        iterator end() { return this->data.end(); }
        int size() { return data.size(); }
        T stdev(){
            T sum = std::accumulate(begin(), end(), 0.0);
            T mean = sum / size();
            std::vector<T> diff(size());
            std::transform(begin(), end(), diff.begin(), [mean](T x) { return x - mean; });
            T sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
            T stdev = std::sqrt(sq_sum / size());
            return stdev;
        }
    };

    static sensor_msgs::CameraInfo cameraInfoFromMat(cv::Mat K, cv::Mat D, cv::Mat R, cv::Mat P, cv::Size size){
        sensor_msgs::CameraInfo cameraInfo;
        if(!D.empty())
        cameraInfo.D = { D.at<float>(0,0), D.at<float>(0,1), D.at<float>(0,2), D.at<float>(0,3), D.at<float>(0,4) };
        if(!K.empty())
        cameraInfo.K = { K.at<float>(0,0), K.at<float>(0,1), K.at<float>(0,2),
                              K.at<float>(1,0), K.at<float>(1,1), K.at<float>(1,2),
                              K.at<float>(2,0), K.at<float>(2,1), K.at<float>(2,2),
                            };
        if(!R.empty())
        cameraInfo.R = { R.at<float>(0,0), R.at<float>(0,1), R.at<float>(0,2),
                              R.at<float>(1,0), R.at<float>(1,1), R.at<float>(1,2),
                              R.at<float>(2,0), R.at<float>(2,1), R.at<float>(2,2),
                            };
        if(!P.empty())
        cameraInfo.P = { P.at<float>(0,0), P.at<float>(0,1), P.at<float>(0,2), P.at<float>(0,3),
                              P.at<float>(1,0), P.at<float>(1,1), P.at<float>(1,2), P.at<float>(1,3),
                              P.at<float>(2,0), P.at<float>(2,1), P.at<float>(2,2), P.at<float>(2,3),
                            };
        cameraInfo.distortion_model = "plumb_bob";
        cameraInfo.height = size.height;
        cameraInfo.width = size.width;
        return cameraInfo;
    }

    static tf::StampedTransform tfFromMat(cv::Mat transformMatrix){
        tf::StampedTransform transform;
        transform.setOrigin(tf::Vector3(transformMatrix.at<float>(0, 3), transformMatrix.at<float>(1, 3), transformMatrix.at<float>(2, 3)));
        tf::Quaternion q;
        tf::Matrix3x3 tf3d;
        tf3d.setValue(
            transformMatrix.at<float>(0, 0), transformMatrix.at<float>(0, 1), transformMatrix.at<float>(0, 2),
            transformMatrix.at<float>(1, 0), transformMatrix.at<float>(1, 1), transformMatrix.at<float>(1, 2),
            transformMatrix.at<float>(2, 0), transformMatrix.at<float>(2, 1), transformMatrix.at<float>(2, 2)
        );
        tf3d.getRotation(q);
        transform.setRotation(q);
        return transform;
    }

    static cv::Mat matFromTF(tf::StampedTransform transform){
        tf::Quaternion q = transform.getRotation();
        tf::Vector3 t = transform.getOrigin();
        tf::Matrix3x3 rot;
        rot.setRotation(q);
        cv::Mat transformMatrix = (cv::Mat_<float>(4,4) <<
                                    rot[0][0], rot[0][1], rot[0][2], t[0],
                                    rot[1][0], rot[1][1], rot[1][2], t[1],
                                    rot[2][0], rot[2][1], rot[2][2], t[2],
                                    0, 0, 0, 1
                                  );
        return transformMatrix;
    }

    static cv::Mat getTransformTree(std::string fromFrame, std::string toFrame){
        static tf::TransformListener listener;
        tf::StampedTransform transform;
        bool s = listener.waitForTransform(toFrame, fromFrame, ros::Time(0), ros::Duration(10));
        if(s) {
            listener.lookupTransform(toFrame, fromFrame, ros::Time(0), transform);
        }else{
            ROS_WARN("Failed to get transform. Returning identity");
            Eigen::Matrix4f i; i.setIdentity();
            return matFromEigen(i);
        }
        return matFromTF(transform);
    }

    void sendTransformTree(cv::Mat transform, std::string fromFrame, std::string toFrame){
        static tf::TransformBroadcaster br;
        tf::StampedTransform tfTransform = tfFromMat(transform);
        br.sendTransform(tf::StampedTransform(tfTransform, ros::Time::now(), toFrame, fromFrame));
    }

    static cv::Mat getMatFromROSMsg(const sensor_msgs::ImageConstPtr& inputImage, float scalar = 1.0){
        cv::Mat img = cv_bridge::toCvShare(inputImage, inputImage->encoding)->image;
        if(scalar != 1.0) cv::resize(img, img, cv::Size(0,0), scalar, scalar);
        return img;
    }

    static cv::Mat getCalibrationMatFromROSMsg(sensor_msgs::CameraInfoConstPtr msg, float scalar=1.0){
        image_geometry::PinholeCameraModel model_;
        model_.fromCameraInfo(msg);
        cv::Mat paddedMatrix = (cv::Mat_<float>(3, 3) <<
                                model_.fx()*scalar, 0, model_.cx()*scalar,
                                0, model_.fy()*scalar, model_.cy()*scalar,
                                0, 0, 1
                                );
        return paddedMatrix;
    }

    static std::vector<std::vector<cv::Point3f> > getObjectPoints(cv::Size boardSize, float squareSize, int count){
        std::vector<std::vector<cv::Point3f> > objectPointsSet;
        for(int x = 0;  x < count; x++){
            std::vector<cv::Point3f> objectPoints;
            for (int i = 0; i < boardSize.height; i++)
                for (int j = 0; j < boardSize.width; j++)
                    objectPoints.push_back(cv::Point3f(float(j*squareSize), float(i*squareSize), 0));
            objectPointsSet.push_back(objectPoints);
        }
        return objectPointsSet;
    }

    static std::string getType(cv::Mat& img){
        int type = img.type();
        std::string r;

        uchar depth = type & CV_MAT_DEPTH_MASK;
        uchar chans = 1 + (type >> CV_CN_SHIFT);

        switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
        }

        r += "C";
        r += (chans+'0');

        return r;
    }

    static std::string getROSType(cv::Mat& img){
        std::string type = getType(img);
        std::string rosType;
        if(type == "8UC1") return "mono8";
        if(type == "16UC1") return "mono16";
        if(type == "8UC3") return "bgr8";
        if(type == "8UC4") return "bgra8";
        return "User";
    }

    static Eigen::Vector3f getRPY(Eigen::Matrix3f rm){
        Eigen::Vector3f ea;
        tf::Matrix3x3 m(
                    rm(0,0), rm(0,1), rm(0,2),
                    rm(1,0), rm(1,1), rm(1,2),
                    rm(2,0), rm(2,1), rm(2,2)
                    );
        double roll, pitch, yaw = 0.0;
        m.getEulerYPR(yaw, pitch, roll, 1);
        ea(0) = (float)roll* (180.0 / M_PI);
        ea(1) = (float)pitch* (180.0 / M_PI);
        ea(2) = (float)yaw* (180.0 / M_PI);
        return ea;
    }

    static Eigen::VectorXf getRPYXYZ(Eigen::Matrix4f transformMatrix){
        Eigen::VectorXf output(6);
        output(3) = transformMatrix(0,3);
        output(4) = transformMatrix(1,3);
        output(5) = transformMatrix(2,3);
        Eigen::Matrix3f rotationMatrix = transformMatrix.block(0,0,3,3);
        Eigen::Vector3f ea = getRPY(rotationMatrix);
        output(0) = ea(0);
        output(1) = ea(1);
        output(2) = ea(2);
        return output;
    }

    static cv::Mat matFromEigen(Eigen::Matrix4f transformMatrix){
        cv::Mat transform;
        cv::eigen2cv(transformMatrix, transform);
        return transform;
    }

    static Eigen::Matrix4f eigenFromMat(cv::Mat transformMatrix){
        Eigen::Matrix4f transform;
        cv::cv2eigen(transformMatrix, transform);
        return transform;
    }

    static std::vector<cv::Point2f> findCheckerBoardPoints(const cv::Mat& img, cv::Mat& drawImage, cv::Size boardSize){
        if(img.channels() == 1) cv::cvtColor(img, drawImage, cv::COLOR_GRAY2BGR); else drawImage = img;
        cv::Mat calibrationImage;
        if(img.channels() == 3) cv::cvtColor(img, calibrationImage, cv::COLOR_BGR2GRAY); else calibrationImage = img;
        std::vector<cv::Point2f> pointbuf;
        bool found = cv::findChessboardCorners(calibrationImage, boardSize, pointbuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
        if(!found){
            found = cv::findChessboardCorners(calibrationImage, boardSize, pointbuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        }
        if(found){
            cv::cornerSubPix(calibrationImage, pointbuf, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
            cv::drawChessboardCorners(drawImage, boardSize, cv::Mat(pointbuf), found);
            return pointbuf;
        }else{
            throw cv::Exception();
        }
    }

    static cv::Mat findTransform(const std::vector<cv::Point2f>& checkerboardPoints, cv::Mat cameraMatrix, cv::Mat distCoeffs, cv::Size boardSize, float squareSize){
        std::vector<cv::Point3f> objectPoints = getObjectPoints(boardSize, squareSize, 1)[0];
        int n = checkerboardPoints.size();
        cv::Mat rvecs, rt;
        cv::Mat tvecs, tr;
        rvecs.resize(n); tvecs.resize(n);
        cv::solvePnP(objectPoints, checkerboardPoints, cameraMatrix, distCoeffs, rvecs, tvecs);
        cv::Rodrigues(rvecs, rt);
        tvecs.convertTo(tr, CV_64F);

        // Generate matrix
        cv::Mat extMat = cv::Mat(4, 4, CV_64FC1);
        extMat.at<double>(0, 0) = rt.at<double>(0, 0); extMat.at<double>(0, 1) = rt.at<double>(0, 1); extMat.at<double>(0, 2) = rt.at<double>(0, 2); extMat.at<double>(0, 3) = tr.at<double>(0, 0);
        extMat.at<double>(1, 0) = rt.at<double>(1, 0); extMat.at<double>(1, 1) = rt.at<double>(1, 1); extMat.at<double>(1, 2) = rt.at<double>(1, 2); extMat.at<double>(1, 3) = tr.at<double>(1, 0);
        extMat.at<double>(2, 0) = rt.at<double>(2, 0); extMat.at<double>(2, 1) = rt.at<double>(2, 1); extMat.at<double>(2, 2) = rt.at<double>(2, 2); extMat.at<double>(2, 3) = tr.at<double>(2, 0);
        extMat.at<double>(3, 0) = 0.0; extMat.at<double>(3, 1) = 0.0; extMat.at<double>(3, 2) = 0.0; extMat.at<double>(3, 3) = 1.0;
        cv::Mat t1 = (cv::Mat_<double>(4,4) <<
                      1, 0, 0, 0,
                      0, 1, 0, squareSize * (boardSize.height - 1),
                      0, 0, 1, 0,
                      0, 0, 0, 1
                      );
        cv::Mat t2 = (cv::Mat_<double>(4,4) <<
                      1, 0, 0, 0,
                      0, cos(M_PI), -sin(M_PI), 0,
                      0, sin(M_PI), cos(M_PI), 0,
                      0, 0, 0, 1
                      );
        extMat = extMat * t1 * t2;
        return extMat;
    }

    static cv::Mat imageSideBySide(const cv::Mat& a, const cv::Mat& b, cv::Size size = cv::Size(640,480)){
        cv::Mat a_conv, b_conv;
        if(a.channels() == 1) cv::cvtColor(a, a_conv, cv::COLOR_GRAY2BGR); else a_conv = a;
        if(b.channels() == 1) cv::cvtColor(b, b_conv, cv::COLOR_GRAY2BGR); else b_conv = b;
        cv::resize(a_conv, a_conv, size);
        cv::resize(b_conv, b_conv, size);
        cv::Mat create;
        cv::hconcat(a_conv, b_conv, create);
        return create;
    }


};


#endif
