/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "LynxHardwareAcceleration/FrameAHB.h"

#include "G2oTypes.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include "GeometricCamera.h"

#include <thread>
#include <include/CameraModels/Pinhole.h>
#include <include/CameraModels/KannalaBrandt8.h>

namespace ORB_SLAM3
{

std::atomic<int> FrameAHB::s_HardwareBufferUses;

FrameAHB::FrameAHB(AHardwareBuffer*& imagesBuffer, const double &timeStamp, ORB_SLAM3::ORBextractor* extractorLeft, ORB_SLAM3::ORBextractor* extractorRight, ORB_SLAM3::ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, ORB_SLAM3::GeometricCamera* pCamera, Frame* pPrevF, const ORB_SLAM3::IMU::Calib &ImuCalib)
{
    std::cout << "FRAMEAHB CONSTRUCTOR 1" << std::endl;

    mpcpi = nullptr;
    mpORBvocabulary = voc;
    mpORBextractorLeft = extractorLeft;
    mpORBextractorRight = extractorRight;
    mTimeStamp = timeStamp;
    mK = K.clone();
    mK_ = ORB_SLAM3::Converter::toMatrix3f(K);
    mDistCoef = distCoef.clone();
    mbf = bf;
    mThDepth = thDepth;
    mImuCalib = ImuCalib;
    mpImuPreintegrated = nullptr;
    mpPrevFrame = pPrevF;
    mpImuPreintegratedFrame = nullptr;
    mpReferenceKF = static_cast<ORB_SLAM3::KeyFrame*>(nullptr);
    mbIsSet = false;
    mbImuPreintegrated = false;
    mpCamera = pCamera;
    mpCamera2 = nullptr;
    mbHasPose = false;
    mbHasVelocity = false;


    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
//    thread threadLeft(&FrameAHB::ExtractORB,this,0,imagesBuffer,0,0);
//    thread threadRight(&FrameAHB::ExtractORB,this,1,imagesBuffer,0,0);
//    threadLeft.join();
//    threadRight.join();

    //STEREO CALL ON Both
    FrameAHB::ExtractORB(imagesBuffer,0,0,0,0);

    //Unlock the AHB we don't need it anymore...
    //TODO Unlock AHB After preprocess!

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

    mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndExtORB - time_StartExtORB).count();
#endif


    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        int height = DEFAULT_HEIGHT;
        int width = DEFAULT_WIDTH;

        ComputeImageBounds(width, height);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    N = mvKeys.size();
    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartStereoMatches = std::chrono::steady_clock::now();
#endif
    ComputeStereoMatches();
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndStereoMatches = std::chrono::steady_clock::now();

    mTimeStereoMatch = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndStereoMatches - time_StartStereoMatches).count();
#endif

    mvpMapPoints = vector<ORB_SLAM3::MapPoint*>(N,static_cast<ORB_SLAM3::MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);
    mmProjectPoints.clear();
    mmMatchedInImage.clear();

    mb = mbf/fx;

    if(pPrevF)
    {
        if(pPrevF->HasVelocity())
            SetVelocity(pPrevF->GetVelocity());
    }
    else
    {
        mVw.setZero();
    }

    mpMutexImu = new std::mutex();

    //Set no stereo fisheye information
    Nleft = -1;
    Nright = -1;
    mvLeftToRightMatch = vector<int>(0);
    mvRightToLeftMatch = vector<int>(0);
    mvStereo3Dpoints = vector<Eigen::Vector3f>(0);
    monoLeft = -1;
    monoRight = -1;

    AssignFeaturesToGrid();
}


void FrameAHB::ExtractORB(AHardwareBuffer* imagesBuffer, const int x0, const int x1, const int x0_1, const int x0_2)
{
    vector<int> vLapping_left = {x0,x1};
    vector<int> vLapping_right = {x0_1,x0_2};

    monoLeft = 0;
    monoRight = 0;

    mnIdMatchingData = (*mpORBextractorLeft)(imagesBuffer, mvKeys,mDescriptors, vLapping_left, mvKeysRight, mDescriptorsRight, vLapping_right, monoLeft, monoRight);
}


    FrameAHB::FrameAHB(AHardwareBuffer*& images, const double &timeStamp, ORB_SLAM3::ORBextractor* extractorLeft, ORB_SLAM3::ORBextractor* extractorRight, ORB_SLAM3::ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, ORB_SLAM3::GeometricCamera* pCamera, ORB_SLAM3::GeometricCamera* pCamera2, Sophus::SE3f& Tlr,Frame* pPrevF, const ORB_SLAM3::IMU::Calib &ImuCalib)
{
    std::cout << "FRAMEAHB CONSTRUCTOR 2" << std::endl;


    mpcpi = nullptr;
    mpORBvocabulary = voc;
    mpORBextractorLeft = extractorLeft;
    mpORBextractorRight = extractorRight;
    mTimeStamp = timeStamp;
    mK = K.clone();
    mK_ = ORB_SLAM3::Converter::toMatrix3f(K);
    mDistCoef = distCoef.clone();
    mbf = bf;
    mThDepth = thDepth;
    mImuCalib = ImuCalib;
    mpImuPreintegrated = nullptr;
    mpPrevFrame = pPrevF;
    mpImuPreintegratedFrame = nullptr;
    mpReferenceKF = static_cast<ORB_SLAM3::KeyFrame*>(nullptr);
    mbIsSet = false;
    mbImuPreintegrated = false;
    mpCamera = pCamera;
    mpCamera2 = pCamera2;
    mbHasPose = false;
    mbHasVelocity = false;



    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif

    s_HardwareBufferUses = 1; // STEREO SO ONLY ONE IS OK
//    thread threadLeft(&FrameAHB::ExtractORB,this,0,images,static_cast<ORB_SLAM3::KannalaBrandt8*>(mpCamera)->mvLappingArea[0],static_cast<ORB_SLAM3::KannalaBrandt8*>(mpCamera)->mvLappingArea[1]);
//    thread threadRight(&FrameAHB::ExtractORB,this,1,images,static_cast<ORB_SLAM3::KannalaBrandt8*>(mpCamera2)->mvLappingArea[0],static_cast<ORB_SLAM3::KannalaBrandt8*>(mpCamera2)->mvLappingArea[1]);
//    threadLeft.join();
//    threadRight.join();

    FrameAHB::ExtractORB(images,
     static_cast<ORB_SLAM3::KannalaBrandt8*>(mpCamera)->mvLappingArea[0], static_cast<ORB_SLAM3::KannalaBrandt8*>(mpCamera)->mvLappingArea[1],
     static_cast<ORB_SLAM3::KannalaBrandt8*>(mpCamera2)->mvLappingArea[0], static_cast<ORB_SLAM3::KannalaBrandt8*>(mpCamera2)->mvLappingArea[1]);


#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

    mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndExtORB - time_StartExtORB).count();
#endif

    Nleft = mvKeys.size();
    Nright = mvKeysRight.size();
    N = Nleft + Nright;

    if(N == 0)
        return;

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        int height = DEFAULT_HEIGHT;
        int width = DEFAULT_WIDTH;

        ComputeImageBounds(width, height);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf / fx;

    // Sophus/Eigen
    mTlr = Tlr;


}

void FrameAHB::ComputeImageBounds(int width, int height)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=width; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=height;
        mat.at<float>(3,0)=width; mat.at<float>(3,1)=height;

        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,static_cast<Pinhole*>(mpCamera)->toK(),mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        // Undistort corners
        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = width;
        mnMinY = 0.0f;
        mnMaxY = height;
    }

    std:cout << "Image bounds: " << mnMinX << ", " << mnMaxX << ", " << mnMinY << ", " << mnMaxY << std::endl;

}

void FrameAHB::FinishBuild(){
    mTrl = mTlr.inverse();
    mRlr = mTlr.rotationMatrix();
    mtlr = mTlr.translation();

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartStereoMatches = std::chrono::steady_clock::now();
#endif
    ComputeStereoFishEyeMatches();
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndStereoMatches = std::chrono::steady_clock::now();

    mTimeStereoMatch = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndStereoMatches - time_StartStereoMatches).count();
#endif

    //Put all descriptors in the same matrix

    if(mDescriptors.empty()){
        mDescriptors = mDescriptorsRight.clone();
    }
    else if(!mDescriptorsRight.empty()){
        cv::vconcat(mDescriptors,mDescriptorsRight,mDescriptors);
    }

    mvpMapPoints = vector<ORB_SLAM3::MapPoint*>(N,static_cast<ORB_SLAM3::MapPoint*>(nullptr));
    mvbOutlier = vector<bool>(N,false);

    AssignFeaturesToGrid();

    mpMutexImu = new std::mutex();

    UndistortKeyPoints();
}


} //namespace ORB_SLAM
