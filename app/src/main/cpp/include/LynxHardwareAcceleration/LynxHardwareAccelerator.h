#pragma once

#include "KeyFrame.h"

#include <memory>
#include <opencv2/core/mat.hpp>
#include <android/hardware_buffer.h>
#include <map>


#include <GLES3/gl3.h>
#include <GLES2/gl2ext.h>
#include <EGL/egl.h>

#include "rpcmem.h"
#include "remote.h"
#include "orbslam3.h"
#include "os_defines.h"

#define DEFAULT_WIDTH (640) //Outside reflections (GO FOR EXTRA MEM) (should be divisible by 128, needs extra vectors on the side to align central vector)
#define DEFAULT_HEIGHT (400) //Outside reflections (GO FOR EXTRA MEM) (should be divisible by 32)

#define MAX_POINTS 20000

//TODO ADD Namespaces



namespace ORB_SLAM3 {

    constexpr int MATCHING_CACHE_SIZE = 3; //To avoid the new DSP Thread to overwrite old data used by the CPU thread immediately

    extern std::string LynxHardwareAccelerator_AndroidLibPath;

    class LynxHardwareAccelerator {

    public:
        static LynxHardwareAccelerator &GetInstance() {
            static LynxHardwareAccelerator instance;
            return instance;
        }

        static std::unique_ptr<LynxHardwareAccelerator> lynxHardwareAccelerator;

        LynxHardwareAccelerator();

        ~LynxHardwareAccelerator();

        void StoreInputBuffer(const uint8_t *frameData) const;
        int ExtractORB(int& leftKeypointsCount, int& rightKeypointsCount, std::vector<cv::KeyPoint>& leftKeyPoints, std::vector<cv::KeyPoint>& rightKeyPoints, cv::OutputArray& leftDescriptors, cv::OutputArray& rightDescriptors, int vLappingLeft0, int vLappingLeft1, int vLappingRight0, int vLappingRight1, int& monoLeft, int& monoRight);
        void BFMatchORB(int nIdMatchingData, const cv::Mat &leftDescriptors, const cv::Mat &rightDescriptors, std::vector<uint16_t>& indices, std::vector<uint16_t>& dist1, std::vector<uint16_t>& dist2) const;

        void DebugLogInputImage();
    private:

        vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint> &vToDistributeKeys, const int &minX,
                          const int &maxX, const int &minY, const int &maxY, const int &nFeatures,
                          const int &level, const int nfeatures);


        remote_handle64 mDSPHandle;
        uint8_t* inputImageRPCMem;
        uint32_t* outputPositionsRPCMem;

        uint32_t* leftKeypointsX;
        uint32_t* leftKeypointsY;
        uint32_t* leftKeypointsAngle;
        uint32_t* leftKeypointsLevels;
        uint8_t* leftKeypointsORB;

        uint32_t* rightKeypointsX;
        uint32_t* rightKeypointsY;
        uint32_t* rightKeypointsAngle;
        uint32_t* rightKeypointsLevels;
        uint8_t* rightKeypointsORB;

        int mFrameCounter = 0;
        int16_t* matchingCacheIndices[MATCHING_CACHE_SIZE];
        int16_t* matchingCacheDistances[MATCHING_CACHE_SIZE];
        int16_t* matchingCacheDistances2[MATCHING_CACHE_SIZE];
    };



}