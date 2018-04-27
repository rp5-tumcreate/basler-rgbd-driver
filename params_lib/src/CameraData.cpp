#include "CameraData.h"

CameraData::CameraData(){

}

bool CameraData::getCameraObject(std::string package, std::string serialNumber, CameraObject& c, cv::Size colorSize, cv::Size irSize){
    std::string path = ros::package::getPath(package)+"/data";
    DIR *dir;
    struct dirent *ent;
    std::vector<std::string> folders;
    if ((dir = opendir (path.c_str())) != NULL) {
      /* print all the files and directories within directory */
      while ((ent = readdir (dir)) != NULL) {
        folders.push_back(ent->d_name);
        //cout << ent->d_name << endl;
      }
      closedir (dir);
    } else {
      perror ("");
      throw;
    }

    for(int i=0; i<folders.size(); i++){
        std::string folderName = folders[i];

        if(folderName == serialNumber){
            c.clear();
            cv::FileStorage fs;

            // IR Resolution
            std::string filename = path + "/" + folderName + "/" + K2_CALIB_IR_RESOLUTION;
            if (fs.open(filename, cv::FileStorage::READ))
            {
                cv::Mat resolution;
                fs[K2_CALIB_RESOLUTION] >> resolution;
                irSize.width = resolution.at<double>(0);
                irSize.height = resolution.at<double>(1);
                fs.release();
            }
            else
            {
                std::cout << "can't open ir resolution file: " << filename << ", using default" << std::endl;
            }

            // Color Resolution
            filename = path + "/" + folderName + "/" + K2_CALIB_COLOR_RESOLUTION;
            if (fs.open(filename, cv::FileStorage::READ))
            {
                cv::Mat resolution;
                fs[K2_CALIB_RESOLUTION] >> resolution;
                colorSize.width = resolution.at<double>(0);
                colorSize.height = resolution.at<double>(1);
                fs.release();
            }
            else
            {
                std::cout << "can't open color resolution file: " << filename << ", using default" << std::endl;
            }

            // Color camera
            filename = path + "/" + folderName + "/" + K2_CALIB_COLOR;
            if (fs.open(filename, cv::FileStorage::READ))
            {
                // Camera Matrix
                cv::Mat K,D,R,P;
                fs[K2_CALIB_CAMERA_MATRIX] >> K;
                fs[K2_CALIB_DISTORTION] >> D;
                fs[K2_CALIB_ROTATION] >> R;
                fs[K2_CALIB_PROJECTION] >> P;
                c.cameraInfoColor.D = { D.at<double>(0,0), D.at<double>(0,1), D.at<double>(0,2), D.at<double>(0,3), D.at<double>(0,4) };
                c.cameraInfoColor.K = { K.at<double>(0,0), K.at<double>(0,1), K.at<double>(0,2),
                                      K.at<double>(1,0), K.at<double>(1,1), K.at<double>(1,2),
                                      K.at<double>(2,0), K.at<double>(2,1), K.at<double>(2,2),
                                    };
                c.cameraInfoColor.R = { R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                                      R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                                      R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2),
                                    };
                c.cameraInfoColor.P = { P.at<double>(0,0), P.at<double>(0,1), P.at<double>(0,2), P.at<double>(0,3),
                                      P.at<double>(1,0), P.at<double>(1,1), P.at<double>(1,2), P.at<double>(1,3),
                                      P.at<double>(2,0), P.at<double>(2,1), P.at<double>(2,2), P.at<double>(2,3),
                                    };
                c.cameraInfoColor.distortion_model = "plumb_bob";
                c.cameraInfoColor.height = colorSize.height;
                c.cameraInfoColor.width = colorSize.width;
                fs[K2_CALIB_DISTORTION] >> c.distortionColor;
                c.cameraMatrixColor = (cv::Mat_<double>(3, 3) <<
                P.at<double>(0,0), P.at<double>(0,1), P.at<double>(0,2),
                P.at<double>(1,0), P.at<double>(1,1), P.at<double>(1,2),
                P.at<double>(2,0), P.at<double>(2,1), P.at<double>(2,2)
                );
                c.cameraMatrixColorPadded = (cv::Mat_<double>(3, 4) <<
                c.cameraMatrixColor.at<double>(0, 0), c.cameraMatrixColor.at<double>(0, 1), c.cameraMatrixColor.at<double>(0, 2), 0,
                c.cameraMatrixColor.at<double>(1, 0), c.cameraMatrixColor.at<double>(1, 1), c.cameraMatrixColor.at<double>(1, 2), 0,
                c.cameraMatrixColor.at<double>(2, 0), c.cameraMatrixColor.at<double>(2, 1), c.cameraMatrixColor.at<double>(2, 2), 0
                );
                cv::Mat cameraMatrixP = (cv::Mat_<double>(3, 3) <<
                                         P.at<double>(0,0), P.at<double>(0,1), P.at<double>(0,2),
                                         P.at<double>(1,0), P.at<double>(1,1), P.at<double>(1,2),
                                         P.at<double>(2,0), P.at<double>(2,1), P.at<double>(2,2)
                                         );
                cv::Mat cameraMatrixK = (cv::Mat_<double>(3, 3) <<
                                         K.at<double>(0,0), K.at<double>(0,1), K.at<double>(0,2),
                                         K.at<double>(1,0), K.at<double>(1,1), K.at<double>(1,2),
                                         K.at<double>(2,0), K.at<double>(2,1), K.at<double>(2,2)
                                         );
                cv::initUndistortRectifyMap(cameraMatrixK, c.distortionColor, cv::Mat(), cameraMatrixP, colorSize, CV_16SC2, c.map1Color, c.map2Color);
                fs.release();
            }
            else
            {
                std::cout << "can't open color calibration file: " << filename << std::endl;
            }

            // IR camera
            filename = path + "/" + folderName + "/" + K2_CALIB_IR;
            if (fs.open(filename, cv::FileStorage::READ))
            {
                // Camera Matrix
                cv::Mat K,D,R,P;
                fs[K2_CALIB_CAMERA_MATRIX] >> K;
                fs[K2_CALIB_DISTORTION] >> D;
                fs[K2_CALIB_ROTATION] >> R;
                fs[K2_CALIB_PROJECTION] >> P;
                c.cameraInfoIR.D = { D.at<double>(0,0), D.at<double>(0,1), D.at<double>(0,2), D.at<double>(0,3), D.at<double>(0,4) };
                c.cameraInfoIR.K = { K.at<double>(0,0), K.at<double>(0,1), K.at<double>(0,2),
                                      K.at<double>(1,0), K.at<double>(1,1), K.at<double>(1,2),
                                      K.at<double>(2,0), K.at<double>(2,1), K.at<double>(2,2),
                                    };
                c.cameraInfoIR.R = { R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                                      R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                                      R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2),
                                    };
                c.cameraInfoIR.P = { P.at<double>(0,0), P.at<double>(0,1), P.at<double>(0,2), P.at<double>(0,3),
                                      P.at<double>(1,0), P.at<double>(1,1), P.at<double>(1,2), P.at<double>(1,3),
                                      P.at<double>(2,0), P.at<double>(2,1), P.at<double>(2,2), P.at<double>(2,3),
                                    };
                c.cameraInfoIR.distortion_model = "plumb_bob";
                c.cameraInfoIR.height = irSize.height;
                c.cameraInfoIR.width = irSize.width;
                fs[K2_CALIB_DISTORTION] >> c.distortionIR;
                c.cameraMatrixIR = (cv::Mat_<double>(3, 3) <<
                P.at<double>(0,0), P.at<double>(0,1), P.at<double>(0,2),
                P.at<double>(1,0), P.at<double>(1,1), P.at<double>(1,2),
                P.at<double>(2,0), P.at<double>(2,1), P.at<double>(2,2)
                );
                c.cameraMatrixIRPadded = (cv::Mat_<double>(3, 4) <<
                c.cameraMatrixIR.at<double>(0, 0), c.cameraMatrixIR.at<double>(0, 1), c.cameraMatrixIR.at<double>(0, 2), 0,
                c.cameraMatrixIR.at<double>(1, 0), c.cameraMatrixIR.at<double>(1, 1), c.cameraMatrixIR.at<double>(1, 2), 0,
                c.cameraMatrixIR.at<double>(2, 0), c.cameraMatrixIR.at<double>(2, 1), c.cameraMatrixIR.at<double>(2, 2), 0
                );
                cv::Mat cameraMatrixP = (cv::Mat_<double>(3, 3) <<
                                         P.at<double>(0,0), P.at<double>(0,1), P.at<double>(0,2),
                                         P.at<double>(1,0), P.at<double>(1,1), P.at<double>(1,2),
                                         P.at<double>(2,0), P.at<double>(2,1), P.at<double>(2,2)
                                         );
                cv::Mat cameraMatrixK = (cv::Mat_<double>(3, 3) <<
                                         K.at<double>(0,0), K.at<double>(0,1), K.at<double>(0,2),
                                         K.at<double>(1,0), K.at<double>(1,1), K.at<double>(1,2),
                                         K.at<double>(2,0), K.at<double>(2,1), K.at<double>(2,2)
                                         );
                cv::initUndistortRectifyMap(cameraMatrixK, c.distortionIR, cv::Mat(), cameraMatrixP, irSize, CV_16SC2, c.map1IR, c.map2IR);
                fs.release();
            }
            else
            {
                std::cout << "can't open ir calibration file: " << filename << std::endl;
            }

            // Transform
            filename = path + "/" + folderName + "/" + K2_CALIB_POSE;
            cv::Mat rotation, translation;
            if (fs.open(filename, cv::FileStorage::READ))
            {
                fs[K2_CALIB_ROTATION] >> rotation;
                fs[K2_CALIB_TRANSLATION] >> translation;
                fs.release();
                c.depthToColor = (cv::Mat_<double>(4, 4) <<
                rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2), translation.at<double>(0, 0),
                rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2), translation.at<double>(1, 0),
                rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2), translation.at<double>(2, 0),
                0, 0, 0, 1
                );
                c.colorToDepth = c.depthToColor.inv();
                c.colorToDepthTF = tfFromMat(c.colorToDepth);
                c.depthToColorTF = tfFromMat(c.depthToColor);
            }
            else
            {
                std::cerr << "can't open pose file: " << filename << std::endl;
            }

            // World
            filename = path + "/" + folderName + "/" + K2_CALIB_WORLD;
            if (fs.open(filename, cv::FileStorage::READ))
            {
                cv::FileNode fn = fs.root();
                for(cv::FileNodeIterator fit = fn.begin(); fit != fn.end(); ++fit)
                {
                  cv::FileNode item = *fit;
                  std::string key = item.name();
                  cv::Mat cameraToFrame;
                  fs[key] >> cameraToFrame;
                  bool direction = (bool)cameraToFrame.at<double>(3,3);
                  cameraToFrame.at<double>(3,3) = 1.;
                  c.cameraToFramesTF[key] = tfFromMat(cameraToFrame);
                  c.cameraToFramesDir[key] = direction;
                }

                fs.release();
            }
            else
            {
                std::cerr << "can't open world file: " << filename << std::endl;
                //cv::Mat cameraToFrame = cv::Mat::eye(4,4,CV_32FC1);
                //c.cameraToFramesTF["default_frame"] = tfFromMat(cameraToFrame);
                //c.cameraToFramesDir["default_frame"] = 1;
            }

            // Offset
            filename = path + "/" + folderName + "/" + K2_CALIB_DEPTH;
            if (fs.open(filename, cv::FileStorage::READ))
            {
                fs[K2_CALIB_DEPTH_SHIFT] >> c.depthShift;
                fs.release();
            }
            else
            {
                std::cerr << "can't open depth file: " << filename << std::endl;
            }

            return true;
        }
    }

    return false;
}

void CameraData::saveTransforms(std::string package, std::string serialNumber, std::string frame, cv::Mat extMat, bool direction, cv::Size colorSize, cv::Size irSize){
    std::string path = ros::package::getPath(package)+"/data";
    std::string filename = path + "/" + serialNumber + "/" + K2_CALIB_WORLD;

    CameraObject c;
    if(!getCameraObject(package, serialNumber, c, colorSize, irSize)){
        std::cout << "serial number not found" << std::endl;
        throw;
    }

    c.cameraToFramesTF[frame] = tfFromMat(extMat);
    c.cameraToFramesDir[frame] = direction;

    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if(!fs.isOpened()){
        std::cout << "failed to open --> " << filename << std::endl;
        throw;
    }

    for(std::map<std::string, tf::Transform>::iterator iterator = c.cameraToFramesTF.begin(); iterator != c.cameraToFramesTF.end(); iterator++) {
        cv::Mat mat = matFromTF(iterator->second);
        std::string key = iterator->first;
        bool direction = c.cameraToFramesDir[key];
        mat.at<double>(3,3) = direction;
        fs << key << mat;
    }

    fs.release();
}

void CameraData::clearExtrinsics(std::string package, std::string serialNumber){
    std::string path = ros::package::getPath(package)+"/data";
    std::string filename = path + "/" + serialNumber + "/" + K2_CALIB_WORLD;
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if(!fs.isOpened()){
        std::cout << "failed to open --> " << filename << std::endl;
        throw;
    }

    fs.release();
}

void CameraData::saveCameraMatrixColor(std::string package, std::string serialNumber, sensor_msgs::CameraInfo cameraInfo, cv::Size colorSize, cv::Size irSize){
    std::string path = ros::package::getPath(package)+"/data";
    std::string filename = path + "/" + serialNumber + "/" + K2_CALIB_COLOR;

    CameraObject c;
    if(!getCameraObject(package, serialNumber, c, colorSize, irSize)){
        std::cout << "serial number not found" << std::endl;
        throw;
    }

    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if(!fs.isOpened()){
        std::cout << "failed to open --> " << filename << std::endl;
        throw;
    }

    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(cameraInfo);

    fs << K2_CALIB_CAMERA_MATRIX << cv::Mat(3, 3, CV_64F, &cameraInfo.K);
    fs << K2_CALIB_DISTORTION << model.distortionCoeffs();
    fs << K2_CALIB_ROTATION << cv::Mat(3, 3, CV_64F, &cameraInfo.R);
    fs << K2_CALIB_PROJECTION << cv::Mat(4, 4, CV_64F, &cameraInfo.P);

    fs.release();
}

void CameraData::saveCameraMatrixIR(std::string package, std::string serialNumber, sensor_msgs::CameraInfo cameraInfo, cv::Size colorSize, cv::Size irSize){
    std::string path = ros::package::getPath(package)+"/data";
    std::string filename = path + "/" + serialNumber + "/" + K2_CALIB_IR;

    CameraObject c;
    if(!getCameraObject(package, serialNumber, c, colorSize, irSize)){
        std::cout << "serial number not found" << std::endl;
        throw;
    }

    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if(!fs.isOpened()){
        std::cout << "failed to open --> " << filename << std::endl;
        throw;
    }

    image_geometry::PinholeCameraModel model_;
    model_.fromCameraInfo(cameraInfo);

    fs << K2_CALIB_CAMERA_MATRIX << cv::Mat(3, 3, CV_64F, &cameraInfo.K);
    fs << K2_CALIB_DISTORTION << model_.distortionCoeffs();
    fs << K2_CALIB_ROTATION << cv::Mat(3, 3, CV_64F, &cameraInfo.R);
    fs << K2_CALIB_PROJECTION << cv::Mat(4, 4, CV_64F, &cameraInfo.P);

    fs.release();
}

