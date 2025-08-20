#include "LynxHardwareAcceleration/LynxHardwareAccelerator.h"

#include "ORBextractor.h"
#include <android/log.h>


#include "rpcmem.h"
#include "remote.h"
#include "orbslam3.h"
#include "os_defines.h"

namespace ORB_SLAM3 {

    using namespace std;

    const char *TAG = "orbslam3";

    const int EDGE_THRESHOLD = 19; //19
    const int PATCH_SIZE = 31;
    const int HALF_PATCH_SIZE = 15;

    std::string LynxHardwareAccelerator_AndroidLibPath;

    std::unique_ptr<LynxHardwareAccelerator> LynxHardwareAccelerator::lynxHardwareAccelerator;

    LynxHardwareAccelerator::LynxHardwareAccelerator() {

        mFrameCounter = 0;


        //INIT DSP ENV
        //const char *path = "/sdcard/";

        std::cout << "LynxHardwareAccelerator_AndroidLibPath: " << LynxHardwareAccelerator_AndroidLibPath << std::endl;

        const char* path = LynxHardwareAccelerator_AndroidLibPath.c_str();
        if (setenv("DSP_LIBRARY_PATH", path, 1) != 0) {
            __android_log_print(ANDROID_LOG_DEBUG, TAG, "Failed to set DSP_LIBRARY_PATH");
        }

        //Init RPC Mem
        rpcmem_init();

        //Init DSP Skel Handle
        if(!remote_session_control){
            __android_log_print(ANDROID_LOG_ERROR, TAG, "Failed to get remote_session_control");
            exit(-1);
        }

        //We enable access to the DSP (SANDBOX MODE UNSIGNED PD)
//        struct remote_rpc_control_unsigned_module remote_rpc_control_mod;
//        remote_rpc_control_mod.enable = 1;
//        remote_rpc_control_mod.domain = CDSP_DOMAIN_ID;
//        if(0 != remote_session_control(DSPRPC_CONTROL_UNSIGNED_MODULE,
//                                                        (void *) &remote_rpc_control_mod, sizeof(remote_rpc_control_mod))){
//            __android_log_print(ANDROID_LOG_ERROR, TAG, "Failed to enable DSP");
//            exit(-1);
//        }

        //We open an handle towards our skeleton
        char* uri = (char *) orbslam3_URI "&_dom=cdsp";
        if(0 != orbslam3_open(uri, &mDSPHandle)){
            __android_log_print(ANDROID_LOG_ERROR, TAG, "Failed to open DSP");
            exit(-1);
        }

        __android_log_print(ANDROID_LOG_DEBUG, TAG, "DSP Handle : %d", mDSPHandle);

        //We pre-allocate the RPC Mem
        inputImageRPCMem = (uint8_t*) rpcmem_alloc(RPCMEM_HEAP_ID_SYSTEM, RPCMEM_DEFAULT_FLAGS, DEFAULT_WIDTH*DEFAULT_HEIGHT*2);
        assert(inputImageRPCMem != nullptr);
        outputPositionsRPCMem = (uint32_t*) rpcmem_alloc(RPCMEM_HEAP_ID_SYSTEM, RPCMEM_DEFAULT_FLAGS, MAX_POINTS*sizeof(uint32_t));
        assert(outputPositionsRPCMem != nullptr);

        leftKeypointsX = (uint32_t*) rpcmem_alloc(RPCMEM_HEAP_ID_SYSTEM, RPCMEM_DEFAULT_FLAGS, MAX_POINTS*sizeof(uint32_t));
        leftKeypointsY = (uint32_t*) rpcmem_alloc(RPCMEM_HEAP_ID_SYSTEM, RPCMEM_DEFAULT_FLAGS, MAX_POINTS*sizeof(uint32_t));
        leftKeypointsAngle = (uint32_t*) rpcmem_alloc(RPCMEM_HEAP_ID_SYSTEM, RPCMEM_DEFAULT_FLAGS, MAX_POINTS*sizeof(uint32_t));
        leftKeypointsLevels = (uint32_t*) rpcmem_alloc(RPCMEM_HEAP_ID_SYSTEM, RPCMEM_DEFAULT_FLAGS, MAX_POINTS*sizeof(uint32_t));
        leftKeypointsORB = (uint8_t*) rpcmem_alloc(RPCMEM_HEAP_ID_SYSTEM, RPCMEM_DEFAULT_FLAGS, MAX_POINTS*sizeof(uint8_t)*32);

        rightKeypointsX = (uint32_t*) rpcmem_alloc(RPCMEM_HEAP_ID_SYSTEM, RPCMEM_DEFAULT_FLAGS, MAX_POINTS*sizeof(uint32_t));
        rightKeypointsY = (uint32_t*) rpcmem_alloc(RPCMEM_HEAP_ID_SYSTEM, RPCMEM_DEFAULT_FLAGS, MAX_POINTS*sizeof(uint32_t));
        rightKeypointsAngle = (uint32_t*) rpcmem_alloc(RPCMEM_HEAP_ID_SYSTEM, RPCMEM_DEFAULT_FLAGS, MAX_POINTS*sizeof(uint32_t));
        rightKeypointsLevels = (uint32_t*) rpcmem_alloc(RPCMEM_HEAP_ID_SYSTEM, RPCMEM_DEFAULT_FLAGS, MAX_POINTS*sizeof(uint32_t));
        rightKeypointsORB = (uint8_t*) rpcmem_alloc(RPCMEM_HEAP_ID_SYSTEM, RPCMEM_DEFAULT_FLAGS, MAX_POINTS*sizeof(uint8_t)*32);

        for(int i = 0; i < MATCHING_CACHE_SIZE; i++){
            matchingCacheIndices[i] = (int16_t*) rpcmem_alloc(RPCMEM_HEAP_ID_SYSTEM, RPCMEM_DEFAULT_FLAGS, MAX_POINTS*sizeof(int16_t));
            matchingCacheDistances[i] = (int16_t*) rpcmem_alloc(RPCMEM_HEAP_ID_SYSTEM, RPCMEM_DEFAULT_FLAGS, MAX_POINTS*sizeof(int16_t));
            matchingCacheDistances2[i] = (int16_t*) rpcmem_alloc(RPCMEM_HEAP_ID_SYSTEM, RPCMEM_DEFAULT_FLAGS, MAX_POINTS*sizeof(int16_t));
        }

    }

    LynxHardwareAccelerator::~LynxHardwareAccelerator() {
        rpcmem_free(inputImageRPCMem);
        rpcmem_free(outputPositionsRPCMem);

        rpcmem_free(leftKeypointsX);
        rpcmem_free(leftKeypointsY);
        rpcmem_free(leftKeypointsAngle);
        rpcmem_free(leftKeypointsLevels);
        rpcmem_free(leftKeypointsORB);

        rpcmem_free(rightKeypointsX);
        rpcmem_free(rightKeypointsY);
        rpcmem_free(rightKeypointsAngle);
        rpcmem_free(rightKeypointsLevels);
        rpcmem_free(rightKeypointsORB);

        for(int i = 0; i < MATCHING_CACHE_SIZE; i++){
            rpcmem_free(matchingCacheIndices[i]);
            rpcmem_free(matchingCacheDistances[i]);
            rpcmem_free(matchingCacheDistances2[i]);
        }

        orbslam3_close(mDSPHandle);
        rpcmem_deinit();
    }

    void LynxHardwareAccelerator::StoreInputBuffer(const uint8_t *frameData) const {
        memcpy(inputImageRPCMem, frameData, DEFAULT_WIDTH*DEFAULT_HEIGHT*2);
    }

    void LynxHardwareAccelerator::DebugLogInputImage(){
        char path[256];
        sprintf(path, "/sdcard/input_%d.raw", rand());
        FILE* file = fopen(path, "w");
        fwrite(inputImageRPCMem, 1, DEFAULT_WIDTH*DEFAULT_HEIGHT*2, file);
        fclose(file);
    }

    int
    LynxHardwareAccelerator::ExtractORB(int& leftKeypointsCount, int& rightKeypointsCount, std::vector<cv::KeyPoint>& leftKeyPoints, std::vector<cv::KeyPoint>& rightKeyPoints, cv::OutputArray& leftDescriptors, cv::OutputArray& rightDescriptors, int vLappingLeft0, int vLappingLeft1, int vLappingRight0, int vLappingRight1, int& monoLeft, int& monoRight) {

        mFrameCounter++;
        int bufferId = (mFrameCounter) % MATCHING_CACHE_SIZE;

        //debug
        //bufferId = 0;

        int32_t totalPointsLeft = 0;
        int32_t totalPointsRight = 0;
        int32_t monoLeftDSP = 0;
        int32_t monoRightDSP = 0;
        orbslam3_extractFeatures(mDSPHandle, inputImageRPCMem, (int)DEFAULT_WIDTH*DEFAULT_HEIGHT*2, (int)DEFAULT_WIDTH, (int)DEFAULT_HEIGHT, DEFAULT_WIDTH*2, (int)20, vLappingLeft0, vLappingLeft1, vLappingRight0, vLappingRight1,
                         &totalPointsLeft, (int*)leftKeypointsX, MAX_POINTS, (int*)leftKeypointsY, MAX_POINTS, (int*)leftKeypointsAngle, MAX_POINTS, (int*)leftKeypointsLevels, MAX_POINTS, leftKeypointsORB, MAX_POINTS*32,
                         &totalPointsRight, (int*)rightKeypointsX, MAX_POINTS, (int*)rightKeypointsY, MAX_POINTS, (int*)rightKeypointsAngle, MAX_POINTS, (int*)rightKeypointsLevels, MAX_POINTS, rightKeypointsORB, MAX_POINTS*32,
                         &monoLeftDSP, &monoRightDSP,
                         matchingCacheIndices[bufferId], MAX_POINTS, matchingCacheDistances[bufferId], MAX_POINTS, matchingCacheDistances2[bufferId], MAX_POINTS); // TODO ADD THE MONO OUTPUT and INPUT

        monoLeft = monoLeftDSP;
        monoRight = monoRightDSP;

        leftKeypointsCount = totalPointsLeft-1;
        rightKeypointsCount = totalPointsRight-1;

        int finalPointsLeft = min(10000, totalPointsLeft-1);
        int finalPointsRight = min(10000, totalPointsRight-1);

        leftKeyPoints = std::vector<cv::KeyPoint>();
        rightKeyPoints = std::vector<cv::KeyPoint>();

        int debugMonoCountLeft = 0;
        int debugMonoCountRight = 0;

        //Convert the output to OpenCV
        for(int i = 0; i < finalPointsLeft; i++){
            cv::KeyPoint kp;
            kp.pt.x = leftKeypointsX[i];
            kp.pt.y = leftKeypointsY[i];
            kp.octave = leftKeypointsLevels[i];

            //Get the real angle
            int8_t cos = leftKeypointsAngle[i] & 0xFF;
            int8_t sin = (leftKeypointsAngle[i] >> 8) & 0xFF;
            kp.angle = atan2(((float)sin)/64.0f, ((float)cos)/64.0f) * 180.0f / M_PI;

            leftKeyPoints.push_back(kp);
        }

        for(int i = 0; i < finalPointsRight; i++){
            cv::KeyPoint kp;
            kp.pt.x = rightKeypointsX[i];
            kp.pt.y = rightKeypointsY[i];
            kp.octave = rightKeypointsLevels[i];

            //Get the real angle
            int8_t cos = rightKeypointsAngle[i] & 0xFF;
            int8_t sin = (rightKeypointsAngle[i] >> 8) & 0xFF;
            kp.angle = atan2(((float)sin)/64.0f, ((float)cos)/64.0f) * 180.0f / M_PI;

            rightKeyPoints.push_back(kp);
        }

        leftDescriptors.create(finalPointsLeft, 32, CV_8U);
        rightDescriptors.create(finalPointsRight, 32, CV_8U);

        //Create descriptors MAT -- Copy to avoid destruction of FASTRPC mem
        cv::Mat(finalPointsLeft, 32, CV_8UC1, leftKeypointsORB).copyTo(leftDescriptors);
        cv::Mat(finalPointsRight, 32, CV_8UC1, rightKeypointsORB).copyTo(rightDescriptors);

        return mFrameCounter;
    }

    void LynxHardwareAccelerator::BFMatchORB(int nIdMatchingData, const cv::Mat &leftDescriptors, const cv::Mat &rightDescriptors, std::vector<uint16_t>& indices, std::vector<uint16_t>& dist1, std::vector<uint16_t>& dist2) const {
        int bufferId = (nIdMatchingData+3) % MATCHING_CACHE_SIZE;

        //Todo make sure to give the previous frame data
        indices.assign(matchingCacheIndices[bufferId], matchingCacheIndices[bufferId] + rightDescriptors.rows*sizeof(uint16_t));
        dist1.assign(matchingCacheDistances[bufferId], matchingCacheDistances[bufferId] + rightDescriptors.rows*sizeof(uint16_t));
        dist2.assign(matchingCacheDistances2[bufferId], matchingCacheDistances2[bufferId] + rightDescriptors.rows*sizeof(uint16_t));
    }


}