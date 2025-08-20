/**
 * @file orbslam_dsp_matcher.h
 *
 * @author Gaston Rouquette (Lynx Mixed Reality)
 *
 * @brief Bruteforce matching for orb descriptors, using HVX Instructions
**/


#pragma once

#include "orbslam3.h"
#include "orbslam_dsp_pyramid.h"
#include "orbslam_dsp_rb.h"

#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wunused-variable"

#define MAX_MATCH 8196

union orb_descriptor{
    uint16_t ui16[16];
    uint8_t ui8[32];
};

/**
 * @brief Prepares the ORB descriptors vector for efficient matching computation
 * @param descriptorsVect : The vector to prepare
 * @param count : The count of descriptors
 * @param descriptors : The descriptors to put in the vector
 */
int prepareORBDescriptorsVector(orb_descriptors_vecs_t* descriptorsVect, int count, orb_descriptor* descriptors);

/**
 * @brief Matches the ORB descriptors of the left and right images using data from the DSP. Used for the final matching step.
 * @param leftDescriptor : ORB descriptors of the left image
 * @param rightDescriptors : ORB descriptors of the right image
 * @param leftCount : Length of the left descriptors
 * @param rightCount : Length of the right descriptors
 * @param indices : Matching indices (The most corresponding points)
 * @param distances : Matching distances 1 (Distance to the most corresponding points)
 * @param distances2s : Matching distances 2 (Distance to the second most corresponding points)
 * @param indTemp : Temporary memory for matching calculations
 * @param ind2Temp : Temporary memory for matching calculations
 * @param distTemp : Temporary memory for matching calculations
 * @param dist2Temp : Temporary memory for matching calculations
 * @retval 0 if success
 */
int knnMatchORB(const orb_descriptors_vecs_t* leftDescriptor, const orb_descriptors_vecs_t* rightDescriptors, int leftCount, int rightCount, uint16_t* indices, uint16_t* distances, uint16_t* distances2s, uint16_t* indTemp, uint16_t* ind2Temp, uint16_t* distTemp, uint16_t* dist2Temp);