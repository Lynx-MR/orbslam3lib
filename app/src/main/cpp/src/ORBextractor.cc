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

/**
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*/


#include <vector>
#include <iostream>

#include <android/log.h>

#include "ORBextractor.h"

using namespace cv;
using namespace std;

namespace ORB_SLAM3
{
    ORBextractor::ORBextractor(int _nfeatures, float _scaleFactor, int _nlevels,
                               int _iniThFAST, int _minThFAST):
            nfeatures(_nfeatures), scaleFactor(_scaleFactor), nlevels(_nlevels),
            iniThFAST(_iniThFAST), minThFAST(_minThFAST)
    {
        //RAW DSP CODED
        nlevels = 8;
        scaleFactor = 1.25f;

        mvScaleFactor.resize(nlevels);
        mvLevelSigma2.resize(nlevels);
        mvScaleFactor[0]=1.0f;
        mvLevelSigma2[0]=1.0f;

        mvScaleFactor[1] = mvScaleFactor[0] * 5.0f/4.0f;
        mvScaleFactor[2] = mvScaleFactor[1] * 4.0f/3.0f;
        mvScaleFactor[3] = mvScaleFactor[2] * 3.0f/2.44948974278;
        mvScaleFactor[4] = mvScaleFactor[3] * 2.44948974278f/2.0f;
        mvScaleFactor[5] = mvScaleFactor[4] * 2.0f/1.58740105197;
        mvScaleFactor[6] = mvScaleFactor[5] * 1.58740105197f/1.25992104989;
        mvScaleFactor[7] = mvScaleFactor[6] * 1.25992104989f/1.0f;
        for(int i=1; i<nlevels; i++)
        {
            mvLevelSigma2[i]=mvScaleFactor[i]*mvScaleFactor[i];
        }

        mvInvScaleFactor.resize(nlevels);
        mvInvLevelSigma2.resize(nlevels);
        for(int i=0; i<nlevels; i++)
        {
            mvInvScaleFactor[i]=1.0f/mvScaleFactor[i];
            mvInvLevelSigma2[i]=1.0f/mvLevelSigma2[i];
        }

        mvImagePyramid.resize(nlevels);

        mnFeaturesPerLevel.resize(nlevels);
        float factor = 1.0f / scaleFactor;
        float nDesiredFeaturesPerScale = nfeatures*(1 - factor)/(1 - (float)pow((double)factor, (double)nlevels));

        int sumFeatures = 0;
        for( int level = 0; level < nlevels-1; level++ )
        {
            mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
            sumFeatures += mnFeaturesPerLevel[level];
            nDesiredFeaturesPerScale *= factor;
        }
        mnFeaturesPerLevel[nlevels-1] = std::max(nfeatures - sumFeatures, 0);
    }

    //Accelerated version of ORBextractor, using STEREO IMAGES
    int ORBextractor::operator()( AHardwareBuffer* _image,
        std::vector<cv::KeyPoint>& _keypointsLeft,
        cv::OutputArray _descriptorsLeft, std::vector<int> &vLappingAreaLeft,
        std::vector<cv::KeyPoint>& _keypointsRight,
        cv::OutputArray _descriptorsRight, std::vector<int> &vLappingAreaRight,
        int& monoLeft, int& monoRight)
    {
        if(LynxHardwareAccelerator::lynxHardwareAccelerator == nullptr){
            //We init the DSP
            LynxHardwareAccelerator::lynxHardwareAccelerator = std::make_unique<LynxHardwareAccelerator>();
        }

        //Get current Time
        auto start = std::chrono::high_resolution_clock::now();

        AHardwareBuffer_Desc desc = {};
        AHardwareBuffer_describe(_image, &desc);

        int width = desc.width / 2;
        int height = desc.height;

        //Lock buffer
        uint8_t* frameData;
        AHardwareBuffer_lock(_image, AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN, -1, nullptr, (void**)&frameData);

        LynxHardwareAccelerator::lynxHardwareAccelerator->StoreInputBuffer(frameData);
        FrameAHB::s_HardwareBufferUses--;

        if(FrameAHB::s_HardwareBufferUses <= 0){
            AHardwareBuffer_unlock(_image, 0);
        }
        int keypointsLeftCount = 0;
        int keypointsRightCount = 0;

        int monoLeftDSP = 0;
        int monoRightDSP = 0;

        //Launch DSP Calculations
        cout << "Before DSPMain" << endl;
        int frameId = LynxHardwareAccelerator::lynxHardwareAccelerator->ExtractORB(keypointsLeftCount, keypointsRightCount, _keypointsLeft, _keypointsRight, _descriptorsLeft, _descriptorsRight,
                                                                        vLappingAreaLeft[0], vLappingAreaLeft[1], vLappingAreaRight[0], vLappingAreaRight[1], monoLeftDSP, monoRightDSP);
        cout << "After DSPMain : " << keypointsLeftCount << ", " << keypointsRightCount << endl;

        monoLeft = monoLeftDSP;
        monoRight = monoRightDSP;

        return frameId;
    }
}