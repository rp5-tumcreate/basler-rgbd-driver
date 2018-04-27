#include <string>
#include <iostream>
#include <vector>

#include <ConsumerImplHelper/ToFCamera.h>
#include <opencv2/opencv.hpp>

#ifndef __BASLERTOF__
#define __BASLERTOF__

using namespace std;
using namespace GenTLConsumerImplHelper;

namespace basler_bridge {

class CustomAllocator : public BufferAllocator
{
public:
    virtual void* AllocateBuffer(size_t size_by ) { return new char[size_by]; }
    virtual void FreeBuffer( void* pBuffer ) { delete[] static_cast<char*>(pBuffer); }
    virtual void Delete() { delete this; }
};

class BaslerTOF {
public:
    BaslerTOF() {
        CToFCamera::InitProducer();
    }
    ~BaslerTOF(){
        if ( CToFCamera::IsProducerInitialized() )
            CToFCamera::TerminateProducer();
    }

    template<typename T>
    bool setSetting(std::string key, T value){
        std::map<std::string, int> settingsMap;
        GenApi::NodeList_t nodes;
        tofCamera_.GetDeviceNodeMap()->GetNodes(nodes);
        for(GenApi::INode* inode : nodes){
            settingsMap[std::string(inode->GetName())] = inode->GetPrincipalInterfaceType();
        }
        if (settingsMap.find(key) == settingsMap.end()){
            cerr << "No such setting: " << key << endl;
            return false;
        }

        switch(settingsMap[key]){
        case 2:{
            if(!std::is_same<T, int>::value){
                cerr << "Wrong Type: " << key << " ,Use int" << endl;
                return false;
            }
            GenApi::CIntegerPtr ptrInt(tofCamera_.GetParameter(key.c_str()));
            ptrInt->SetValue(value);
            break;
        }
        case 3:{
            if(!std::is_same<T, bool>::value){
                cerr << "Wrong Type: " << key << " ,Use bool" << endl;
                return false;
            }
            GenApi::CBooleanPtr ptrBool(tofCamera_.GetParameter(key.c_str()));
            ptrBool->SetValue(value);
            break;
        }
        case 5:{
            if(!std::is_same<T, float>::value){
                cerr << "Wrong Type: " << key << " ,Use float" << endl;
                return false;
            }
            GenApi::CFloatPtr ptrFloat(tofCamera_.GetParameter(key.c_str()));
            ptrFloat->SetValue(value);
            break;
        }
        default:{
            cerr << "Type not implemented: " << settingsMap[key] << endl;
            return false;
        }
        }

        return true;
    }

    void loadFromFile(const char* parameterFileName){
        std::ifstream strm(parameterFileName);
        if ( ! strm )
        {
            std::ostringstream s;
            throw RUNTIME_EXCEPTION( static_cast<std::stringstream&>(s << "Failed to open file "<< parameterFileName ).str());
        }

        GenApi::CFeatureBag FeatureBag;
        strm >> FeatureBag;
        {
            FeatureBag.LoadFromBag(tofCamera_.GetDeviceNodeMap());
            FeatureBag.LoadFromBag(tofCamera_.GetDeviceNodeMap());
        }
    }

    void saveToFile(const char* parameterFileName){
        std::ofstream strm(parameterFileName);
        if ( ! strm )
        {
            std::ostringstream s;
            throw RUNTIME_EXCEPTION( static_cast<std::stringstream&>(s << "Failed to open file "<< parameterFileName ).str());
        }

        GenApi::CFeatureBag FeatureBag;
        FeatureBag.StoreToBag(tofCamera_.GetDeviceNodeMap());
        strm << FeatureBag;
    }

    void addSettings(std::string key, float value){
        settingsMap_[key] = value;
        if(tofCamera_.IsOpen()){
            applySettings();
        }
    }

    void applySettings(){
        for(auto const &setting : settingsMap_) {
            std::string key = setting.first;
            float value = setting.second;
            if(key ==  "FilterSpatial") if(!setSetting<bool>("FilterSpatial", value)) throw RUNTIME_EXCEPTION("Setting Error");
            if(key ==  "FilterTemporal") if(!setSetting<bool>("FilterTemporal", value)) throw RUNTIME_EXCEPTION("Setting Error");
            if(key ==  "ConfidenceThreshold") if(!setSetting<int>("ConfidenceThreshold", ((int)(value/16)*16))) throw RUNTIME_EXCEPTION("Setting Error");
            if(key ==  "FilterStrength") if(!setSetting<int>("FilterStrength", ((int)(value/5)*5))) throw RUNTIME_EXCEPTION("Setting Error");
            if(key ==  "OutlierTolerance") if(!setSetting<int>("OutlierTolerance", ((int)(value/16)*16))) throw RUNTIME_EXCEPTION("Setting Error");
        }
    }

    void setParams(std::string serialNumber, std::string settingsPath){
        serialNumber_ = serialNumber;
        settingsPath_ = settingsPath;

        // Enumerate cameras
        CameraList lstCameras = tofCamera_.EnumerateCameras();
        bool camFound = false;
        for (CameraList::iterator it = lstCameras.begin(); it != lstCameras.end(); ++it) {
            CameraInfo& camInfo = *it;
            if(camInfo.strSerialNumber == serialNumber_){
                cout << "Camera Found" << endl;
                camFound = true;
                break;
            }
        }

        // Stop if list is empty or serial number mismatch
        if (lstCameras.empty())
        {
           throw RUNTIME_EXCEPTION("No cameras found.");
        }
        if (!camFound)
        {
           throw RUNTIME_EXCEPTION("Serial No not found");
        }
    }

    void start(){
        if(!tofCamera_.IsOpen()){

            // Enumerate cameras
            CameraList lstCameras = tofCamera_.EnumerateCameras();
            bool camFound = false;
            for (CameraList::iterator it = lstCameras.begin(); it != lstCameras.end(); ++it) {
                CameraInfo& camInfo = *it;
                if(camInfo.strSerialNumber == serialNumber_){
                    cout << "Camera Found" << endl;
                    tofCamera_.Open(camInfo);
                    camFound = true;
                    break;
                }
            }

            // Stop if list is empty or serial number mismatch
            if (lstCameras.empty())
            {
               throw RUNTIME_EXCEPTION("No cameras found.");
            }
            if (!camFound)
            {
               throw RUNTIME_EXCEPTION("Serial No not found");
            }

            // Settings
            applySettings();
            loadFromFile(settingsPath_.c_str());

            // Enable depth and intensity data
            GenApi::CEnumerationPtr ptrImageComponentSelector = tofCamera_.GetParameter("ImageComponentSelector");
            GenApi::CBooleanPtr ptrImageComponentEnable = tofCamera_.GetParameter("ImageComponentEnable");
            GenApi::CEnumerationPtr ptrPixelFormat = tofCamera_.GetParameter("PixelFormat");
            ptrImageComponentSelector->FromString("Range");
            ptrImageComponentEnable->SetValue(true);
            ptrImageComponentSelector->FromString("Intensity");
            ptrImageComponentEnable->SetValue(true);

            // Setup buffers and start acquisition
            tofCamera_.SetBufferAllocator( new CustomAllocator(), true); // m_Camera takes ownership and will clean-up allocator.
            tofCamera_.PrepareAcquisition( 2 );
            for ( size_t i = 0; i < 2; ++i )
            {
                tofCamera_.QueueBuffer( i );
            }
            tofCamera_.StartAcquisition();
            tofCamera_.IssueAcquisitionStartCommand();
        }
    }

    void stop(){
        if(tofCamera_.IsOpen()){
            cout << "Camera Stopped" << endl;
            // Stop the camera
            tofCamera_.IssueAcquisitionStopCommand();
            tofCamera_.FinishAcquisition();
            tofCamera_.Close();
        }
    }

    bool grabImages(){
        // Grab
        GrabResult grabResult;
        tofCamera_.GetGrabResult( grabResult, 10000 );

        // Check whether a buffer has been grabbed successfully.
        if ( grabResult.status == GrabResult::Timeout )
        {
            cerr << "Timeout occurred." << endl;
            return false;
        }
        if ( grabResult.status != GrabResult::Ok )
        {
            cerr << "Failed to grab image." << endl;
            return false;
        }

        // Get Buffer
        BufferParts parts;
        tofCamera_.GetBufferParts(grabResult, parts);

        // Get Dim
        imageSize_.width = (int) parts[0].width;
        imageSize_.height = (int) parts[0].height;
        int count = imageSize_.width * imageSize_.height;

        // Get Depth
        rangeMap_ = cv::Mat(imageSize_.height, imageSize_.width, CV_16UC1, parts[0].pData);

        // Get Intensity
        intensityMap_ = cv::Mat(imageSize_.height, imageSize_.width, CV_16UC1, parts[1].pData);
        double  max;
        cv::minMaxLoc( intensityMap_, NULL, &max);
        intensityMap_ /= ( max / std::numeric_limits<uint16_t>::max() );
        cv::convertScaleAbs(intensityMap_, intensityMap_, (255.0/65535.0));

        // Put the buffer back into the acquisition  queue to fill with new data
        tofCamera_.QueueBuffer( grabResult.hBuffer );

        return true;
    }

    cv::Mat getRangeMap(){
        return rangeMap_;
    }

    cv::Mat getIntensityMap(){
        return intensityMap_;
    }

private:
    CToFCamera  tofCamera_;
    std::string serialNumber_;
    std::string settingsPath_;
    std::map<std::string, float> settingsMap_;

    cv::Mat rangeMap_;
    cv::Mat intensityMap_;
    cv::Size imageSize_;
};

}
#endif

