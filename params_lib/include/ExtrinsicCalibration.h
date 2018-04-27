#ifndef EXTRINSIC_CALIBRATION_H
#define EXTIRNSIC_CALIBRATION_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <Eigen/Eigen>

using namespace std;
# define M_PI 3.14159265358979323846  /* pi */

class ExtrinsicCalibration
{
public:
    enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

    ExtrinsicCalibration(){}
    ExtrinsicCalibration(cv::Size boardSize, float squareSize, Pattern pattern);
    bool calibrateExtrinsics(cv::Mat& img, cv::Mat& extMat, int squareShift = 0, float heightShift = 0., Eigen::Vector3f shift = Eigen::Vector3f(0,0,0));
    void checkCalibration(cv::Mat& img, cv::Mat extMat);
    void setBoardSize(cv::Size boardSize);
    void setSquareSize(float squareSize);
    void setCameraMatrix(cv::Mat cameraMatrix);
    void setDistCoeffs(cv::Mat distCoeffs);

private:

    cv::Mat cameraMatrix_, distCoeffs_;
    cv::Size boardSize_, imageSize_;
    float squareSize_;
    int pattern_;
};

#endif // EXTRINSIC_CALIBRATION_H
