#include "ExtrinsicCalibration.h"

ExtrinsicCalibration::ExtrinsicCalibration(cv::Size boardSize, float squareSize, Pattern pattern){
    boardSize_ = boardSize;
    squareSize_ = squareSize;
    pattern_ = pattern;
    distCoeffs_ = cv::Mat::zeros(8, 1, CV_64F);
    cameraMatrix_ = cv::Mat::eye(3, 3, CV_64F);
}

void ExtrinsicCalibration::setBoardSize(cv::Size boardSize){
    boardSize_ = boardSize;
}

void ExtrinsicCalibration::setSquareSize(float squareSize){
    squareSize_ = squareSize;
}

void ExtrinsicCalibration::setCameraMatrix(cv::Mat cameraMatrix){
    cameraMatrix_ = cameraMatrix;
}

void ExtrinsicCalibration::setDistCoeffs(cv::Mat distCoeffs){
    distCoeffs_ = distCoeffs;
}

bool ExtrinsicCalibration::calibrateExtrinsics (cv::Mat &img, cv::Mat& extMat, int squareShift, float heightShift, Eigen::Vector3f shift)
{
    // Setup data structures
    imageSize_ = img.size();
    std::vector<cv::Point2f> pointbuf;
    std::vector<cv::Point2f> imagePoints;

    // Convert image to grey
    cv::Mat imgGray;
    cv::cvtColor(img, imgGray, CV_BGR2GRAY);

    // Find checkboard patterns
    bool found = cv::findChessboardCorners(img, boardSize_, pointbuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
    if (pattern_ == CHESSBOARD && found) {

        // Draw onto image
        cv::cornerSubPix(imgGray, pointbuf, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        imagePoints=pointbuf;
        cv::drawChessboardCorners(img, boardSize_, cv::Mat(pointbuf), found);

        // Check if enough points found
        if (imagePoints.size() > 0) {

            // Setup 3D points
            std::vector<cv::Point3f> objectPoints;
            switch (pattern_)
            {
                case CHESSBOARD:
                case CIRCLES_GRID:
                    for (int i = 0; i < boardSize_.height; i++)
                        for (int j = 0; j < boardSize_.width; j++)
                            objectPoints.push_back(cv::Point3f(float(j*squareSize_),
                                                      float(i*squareSize_), 0));
                    break;

                case ASYMMETRIC_CIRCLES_GRID:
                    for (int i = 0; i < boardSize_.height; i++)
                        for (int j = 0; j < boardSize_.width; j++)
                            objectPoints.push_back(cv::Point3f(float((2 * j + i % 2)*squareSize_),
                                                      float(i*squareSize_), 0));
                    break;
            }

            // Run optimization
            double totalAvgErr = 0.0;
            int n = imagePoints.size();
            cv::Mat rvecs, rt;
            cv::Mat tvecs, tr;
            rvecs.resize(n); tvecs.resize(n);
            cv::solvePnP(objectPoints, imagePoints, cameraMatrix_, distCoeffs_, rvecs, tvecs);
            bool ok = cv::checkRange(cameraMatrix_) && cv::checkRange(distCoeffs_);
            cv::Rodrigues(rvecs, rt);
            tvecs.convertTo(tr, CV_64F);

            // Generate matrix
            extMat = cv::Mat(4, 4, CV_64FC1);
            extMat.at<double>(0, 0) = rt.at<double>(0, 0); extMat.at<double>(0, 1) = rt.at<double>(0, 1); extMat.at<double>(0, 2) = rt.at<double>(0, 2); extMat.at<double>(0, 3) = tr.at<double>(0, 0);
            extMat.at<double>(1, 0) = rt.at<double>(1, 0); extMat.at<double>(1, 1) = rt.at<double>(1, 1); extMat.at<double>(1, 2) = rt.at<double>(1, 2); extMat.at<double>(1, 3) = tr.at<double>(1, 0);
            extMat.at<double>(2, 0) = rt.at<double>(2, 0); extMat.at<double>(2, 1) = rt.at<double>(2, 1); extMat.at<double>(2, 2) = rt.at<double>(2, 2); extMat.at<double>(2, 3) = tr.at<double>(2, 0);
            extMat.at<double>(3, 0) = 0.0; extMat.at<double>(3, 1) = 0.0; extMat.at<double>(3, 2) = 0.0; extMat.at<double>(3, 3) = 1.0;
            cv::Mat t1 = (cv::Mat_<double>(4,4) <<
                1, 0, 0, 0,
                0, 1, 0, squareSize_ * (boardSize_.height - 1),
                0, 0, 1, 0,
                0, 0, 0, 1
            );
            cv::Mat t2 = (cv::Mat_<double>(4,4) <<
                1, 0, 0, 0,
                0, cos(M_PI), -sin(M_PI), 0,
                0, sin(M_PI), cos(M_PI), 0,
                0, 0, 0, 1
            );
            cv::Mat t3 = (cv::Mat_<double>(4,4) <<
                          1, 0, 0, (squareSize_ * squareShift) + shift(0),
                          0, 1, 0, squareSize_ * squareShift + shift(1),
                          0, 0, 1, heightShift + shift(2),
                          0, 0, 0, 1
                      );
            extMat = extMat * t1 * t2 * t3;

            // Check calibration
            checkCalibration(img, extMat);
        }
        // Not enough points found
        else{
            std::cout << "imagePoints.size() <= 0 " << std::endl;
            return false;
        }

    }
    // No checkboard pattern found
    else{
        std::cerr << "no points found in findChessboardCorners" << std::endl;
        return false;
    }

    return true;

}

void ExtrinsicCalibration::checkCalibration(cv::Mat& img, cv::Mat extMat)
{
    cv::Mat pr_mat(4, 4, CV_64FC1);
    cv::Mat cam_mat_padded(3, 4, CV_64FC1);

    for (int r = 0; r < cameraMatrix_.rows; ++r) {
        for (int c = 0; c < cameraMatrix_.cols; ++c) {
            cam_mat_padded.at<double>(r, c) = cameraMatrix_.at<double>(r, c);
        }
    }

    cam_mat_padded.at<double>(0, 3) = 0.0; cam_mat_padded.at<double>(1, 3) = 0.0; cam_mat_padded.at<double>(2, 3) = 0.0;

    pr_mat = cam_mat_padded*extMat;

    cv::Mat point_3d(4, 1, CV_64FC1);
    cv::Mat point_2d(3, 1, CV_64FC1);

    point_3d.at<double>(0, 0) = 0.0; point_3d.at<double>(1, 0) = 0.0; point_3d.at<double>(2, 0) = 0.001;point_3d.at<double>(3, 0) = 1.0;
    point_2d = pr_mat*point_3d;
    cv::Point px_o; cv::Point px_x; cv::Point px_y; cv::Point px_z;

    px_o.x = (int)(point_2d.at<double>(0, 0) / point_2d.at<double>(2, 0));
    px_o.y = (int)(point_2d.at<double>(1, 0) / point_2d.at<double>(2, 0));

    point_3d.at<double>(0, 0) = (squareSize_ * (boardSize_.width - 1)); point_3d.at<double>(1, 0) = 0.0; point_3d.at<double>(2, 0) = 0.001;point_3d.at<double>(3, 0) = 1.0;
    point_2d = pr_mat*point_3d;

    px_x.x = (int)(point_2d.at<double>(0, 0) / point_2d.at<double>(2, 0));
    px_x.y = (int)(point_2d.at<double>(1, 0) / point_2d.at<double>(2, 0));

    point_3d.at<double>(0, 0) = 0.0; point_3d.at<double>(1, 0) = (squareSize_ * (boardSize_.height - 1)); point_3d.at<double>(2, 0) = 0.001;point_3d.at<double>(3, 0) = 1.0;
    point_2d = pr_mat*point_3d;

    px_y.x = (int)(point_2d.at<double>(0, 0) / point_2d.at<double>(2, 0));
    px_y.y = (int)(point_2d.at<double>(1, 0) / point_2d.at<double>(2, 0));

    point_3d.at<double>(0, 0) = 0.0; point_3d.at<double>(1, 0) = 0.0; point_3d.at<double>(2, 0) = 0.5;point_3d.at<double>(3, 0) = 1.0;
    point_2d = pr_mat*point_3d;

    px_z.x = (int)(point_2d.at<double>(0, 0) / point_2d.at<double>(2, 0));
    px_z.y = (int)(point_2d.at<double>(1, 0) / point_2d.at<double>(2, 0));

    cv::circle(img, px_o, 5, cv::Scalar(0, 255, 0), 3);
    cv::circle(img, px_x, 5, cv::Scalar(0, 255, 0), 3);
    cv::circle(img, px_y, 5, cv::Scalar(0, 255, 0), 3);
    cv::circle(img, px_z, 5, cv::Scalar(0, 255, 0), 3);

    cv::line(img, px_o, px_x, cv::Scalar(0, 255, 0), 3);
    cv::line(img, px_o, px_y, cv::Scalar(0, 255, 0), 3);
    cv::line(img, px_o, px_z, cv::Scalar(0, 255, 0), 3);

    cv::putText(img, "origin", px_o, cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0), 2, 2);
    cv::putText(img, "X-Axis", px_x, cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0), 2, 2);
    cv::putText(img, "Y-Axis", px_y, cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0), 2, 2);
    cv::putText(img, "Z-Axis", px_z, cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0), 2, 2);
}
