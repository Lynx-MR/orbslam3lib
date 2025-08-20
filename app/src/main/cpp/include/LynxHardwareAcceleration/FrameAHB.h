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


#ifndef FRAME_AHB_H
#define FRAME_AHB_H

#include<vector>

#include <DBoW2/BowVector.h>
#include <DBoW2/FeatureVector.h>

#include "Thirdparty/Sophus/sophus/geometry.hpp"

#include "ImuTypes.h"
#include "ORBVocabulary.h"

#include "Converter.h"
#include "Settings.h"
#include "Frame.h"

#include <mutex>
#include <opencv2/opencv.hpp>

#include <android/hardware_buffer.h>

#include "Eigen/Core"
#include "sophus/se3.hpp"



namespace ORB_SLAM3{


class FrameAHB : public ORB_SLAM3::Frame
{
public:
    // Constructor for stereo cameras.
    FrameAHB(AHardwareBuffer*& images, const double &timeStamp, ORB_SLAM3::ORBextractor* extractorLeft, ORB_SLAM3::ORBextractor* extractorRight, ORB_SLAM3::ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, ORB_SLAM3::GeometricCamera* pCamera,Frame* pPrevF = static_cast<Frame*>(NULL), const ORB_SLAM3::IMU::Calib &ImuCalib = ORB_SLAM3::IMU::Calib());


    // Extract ORB on the image. 0 for left image and 1 for right image. Contains both Images
    void ExtractORB(AHardwareBuffer* imagesBuffer, const int x0, const int x1, const int x0_1, const int x0_2);

    static std::atomic<int> s_HardwareBufferUses;

private:

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(int width, int height);


public:
    FrameAHB(AHardwareBuffer*& images, const double &timeStamp, ORB_SLAM3::ORBextractor* extractorLeft, ORB_SLAM3::ORBextractor* extractorRight, ORB_SLAM3::ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, ORB_SLAM3::GeometricCamera* pCamera, ORB_SLAM3::GeometricCamera* pCamera2, Sophus::SE3f& Tlr,Frame* pPrevF = static_cast<Frame*>(NULL), const ORB_SLAM3::IMU::Calib &ImuCalib = ORB_SLAM3::IMU::Calib());

    void FinishBuild();
};

}// namespace ORB_SLAM

#endif // FRAME_H
