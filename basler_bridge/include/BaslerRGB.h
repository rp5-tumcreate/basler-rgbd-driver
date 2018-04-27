#include <string>
#include <iostream>
#include <vector>

#include <pylon/PylonIncludes.h>
#include <opencv2/opencv.hpp>

#ifndef __BASLERRGB__
#define __BASLERRGB__

using namespace std;
using namespace Pylon;

namespace basler_bridge {

class BaslerRGB {
public:
    BaslerRGB() {
        PylonInitialize();
    }
    ~BaslerRGB(){
        PylonTerminate();
    }

//    template<typename T>
//    bool setSetting(std::string key, T value){
//        std::map<std::string, int> settingsMap;
//        GenApi::NodeList_t nodes;
//        tofCamera_.GetDeviceNodeMap()->GetNodes(nodes);
//        for(GenApi::INode* inode : nodes){
//            settingsMap[std::string(inode->GetName())] = inode->GetPrincipalInterfaceType();
//        }
//        if (settingsMap.find(key) == settingsMap.end()){
//            cerr << "No such setting: " << key << endl;
//            return false;
//        }

//        switch(settingsMap[key]){
//        case 2:{
//            if(!std::is_same<T, int>::value){
//                cerr << "Wrong Type: " << key << " ,Use int" << endl;
//                return false;
//            }
//            GenApi::CIntegerPtr ptrInt(tofCamera_.GetParameter(key.c_str()));
//            ptrInt->SetValue(value);
//            break;
//        }
//        case 3:{
//            if(!std::is_same<T, bool>::value){
//                cerr << "Wrong Type: " << key << " ,Use bool" << endl;
//                return false;
//            }
//            GenApi::CBooleanPtr ptrBool(tofCamera_.GetParameter(key.c_str()));
//            ptrBool->SetValue(value);
//            break;
//        }
//        case 5:{
//            if(!std::is_same<T, float>::value){
//                cerr << "Wrong Type: " << key << " ,Use float" << endl;
//                return false;
//            }
//            GenApi::CFloatPtr ptrFloat(tofCamera_.GetParameter(key.c_str()));
//            ptrFloat->SetValue(value);
//            break;
//        }
//        default:{
//            cerr << "Type not implemented: " << settingsMap[key] << endl;
//            return false;
//        }
//        }

//        return true;
//    }

//    void loadFromFile(const char* parameterFileName){
//        std::ifstream strm(parameterFileName);
//        if ( ! strm )
//        {
//            std::ostringstream s;
//            throw RUNTIME_EXCEPTION( static_cast<std::stringstream&>(s << "Failed to open file "<< parameterFileName ).str());
//        }

//        GenApi::CFeatureBag FeatureBag;
//        strm >> FeatureBag;
//        {
//            FeatureBag.LoadFromBag(rgbCamera_.GetDeviceNodeMap());
//            FeatureBag.LoadFromBag(tofCamera_.GetDeviceNodeMap());
//        }
//    }

//    void saveToFile(const char* parameterFileName){
//        std::ofstream strm(parameterFileName);
//        if ( ! strm )
//        {
//            std::ostringstream s;
//            throw RUNTIME_EXCEPTION( static_cast<std::stringstream&>(s << "Failed to open file "<< parameterFileName ).str());
//        }

//        GenApi::CFeatureBag FeatureBag;
//        FeatureBag.StoreToBag(tofCamera_.GetDeviceNodeMap());
//        strm << FeatureBag;
//    }

    void addSettings(std::string key, float value){
        settingsMap_[key] = value;
        if(rgbCamera_->IsOpen()){
            applySettings();
        }
    }

    void applySettings(){
        for(auto const &setting : settingsMap_) {
            std::string key = setting.first;
            float value = setting.second;
        }
    }

    void setParams(std::string serialNumber, std::string settingsPath){
        serialNumber_ = serialNumber;
        settingsPath_ = settingsPath;

        // Enumerate Cameras
        DeviceInfoList_t list;
        CTlFactory::GetInstance().EnumerateDevices(list);
        bool camFound = false;
        for(int i=0; i<list.size(); i++){
            CDeviceInfo& deviceInfo = list[i];
            if(deviceInfo.GetSerialNumber() == serialNumber_.c_str()){
                cout << "Camera Found" << endl;
                rgbCamera_ = new CInstantCamera(CTlFactory::GetInstance().CreateDevice(deviceInfo));
                camFound = true;
                break;
            }
        }

        // Stop if list is empty or serial number mismatch
        if (list.empty())
        {
           throw RUNTIME_EXCEPTION("No cameras found.");
        }
        if (!camFound)
        {
           throw RUNTIME_EXCEPTION("Serial No not found");
        }

        // Settings
        formatConverter_.OutputPixelFormat = PixelType_BGR8packed;

    }

    void start(){
        if(!rgbCamera_->IsOpen()){
            rgbCamera_->Open();

            // Load Setting
            CFeaturePersistence::Load( settingsPath_.c_str(), &rgbCamera_->GetNodeMap(), true );

            // Start Camera
            rgbCamera_->StartGrabbing();
            cout << "Camera Started" << endl;
        }
    }

    void stop(){
        if(rgbCamera_->IsOpen()){
            cout << "Camera Stopped" << endl;
            // Stop the camera
            rgbCamera_->StopGrabbing();
            rgbCamera_->Close();
        }
    }

    bool grabImages(){
        CGrabResultPtr ptrGrabResult;
        if(rgbCamera_->IsGrabbing()){

            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            rgbCamera_->RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);

            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
                // Access the image data.
                formatConverter_.Convert(pylonImage_, ptrGrabResult);
                rgbMap_ = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *)pylonImage_.GetBuffer());
                return true;
            }
            else
            {
                cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
                return false;
            }
        }else return false;
    }

    cv::Mat getRGBMap(){
        return rgbMap_;
    }

private:
    CInstantCamera* rgbCamera_;
    std::string serialNumber_;
    std::string settingsPath_;
    std::map<std::string, float> settingsMap_;

    cv::Mat rgbMap_;
    cv::Size imageSize_;

    CImageFormatConverter formatConverter_;
    CPylonImage pylonImage_;
};

}
#endif

