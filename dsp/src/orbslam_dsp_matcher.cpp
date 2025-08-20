/**
 * @file orbslam_dsp_matcher.cpp
 *
 * @author Gaston Rouquette (Lynx Mixed Reality)
 *
 * @brief Bruteforce matching for orb descriptors, using HVX Instructions
**/



#include "orbslam_dsp_matcher.h"
#include "HAP_farf.h"
#include "HAP_vtcm_mgr.h"
#include "HAP_compute_res.h"
#include "AEEStdErr.h"
#include "orbslam3.h"
#include "hvx_internal.h"

/// @brief Perform k-Nearest Neighbor matching for ORB descriptors using HVX vectors
/// @param leftDescriptor Array of left ORB descriptors
/// @param rightDescriptors Array of right ORB descriptors
/// @param leftCount Number of left descriptors
/// @param rightCount Number of right descriptors
/// @param indices Output array of matched indices
/// @param distances Output array of minimum distances
/// @param distances2 Output array of second minimum distances
/// @param indTemp Temporary buffer for indices
/// @param ind2Temp Temporary buffer for second indices
/// @param distTemp Temporary buffer for distances
/// @param dist2Temp Temporary buffer for second distances
/// @return 0 on success, -1 if descriptor count exceeds MAX_MATCH
int knnMatchORB(const orb_descriptors_vecs_t* leftDescriptor, 
                const orb_descriptors_vecs_t* rightDescriptors, 
                int leftCount, 
                int rightCount, 
                uint16_t* indices, 
                uint16_t* distances, 
                uint16_t* distances2, 
                uint16_t* indTemp, 
                uint16_t* ind2Temp, 
                uint16_t* distTemp, 
                uint16_t* dist2Temp) {
    // --- L2 Cache Prefetch ---
    // Fetch left and right descriptors into L2 cache
    int leftVecCount = (leftCount - 1) / (VLEN / 2) + 1;
    l2fetch(leftDescriptor, leftCount * 2, leftVecCount * VLEN * 2, 16, 1);

    int rightVecCount = (rightCount - 1) / (VLEN / 2) + 1;
    l2fetch(rightDescriptors, rightCount * 2, rightVecCount * VLEN * 2, 16, 1);

    // --- Input Validation ---
    if (leftCount > MAX_MATCH || rightCount > MAX_MATCH) {
        return -1;
    }

    // --- Buffer Initialization ---
    HVX_Vector* minimumDistances = (HVX_Vector*)distTemp;
    HVX_Vector* minimumIndices = (HVX_Vector*)indTemp;
    HVX_Vector* minimumDistances2 = (HVX_Vector*)dist2Temp;
    HVX_Vector* minimumIndices2 = (HVX_Vector*)ind2Temp;

    // Initialize minimum distances and indices with maximum values
    HVX_Vector* minDistances = minimumDistances;
    HVX_Vector* minIndices = minimumIndices;
    HVX_Vector* minDistances2 = minimumDistances2;
    HVX_Vector* minIndices2 = minimumIndices2;
    for (int j = 0; j < 88; j++) {
        *minDistances++ = Q6_Vh_vsplat_R(512);
        *minIndices++ = Q6_Vh_vsplat_R(0);
        *minDistances2++ = Q6_Vh_vsplat_R(512);
        *minIndices2++ = Q6_Vh_vsplat_R(0);
    }

    // --- Main Matching Loop ---
    for (int i = 0; i < leftCount; i++) {
        // Load left descriptor into HVX vectors
        HVX_Vector leftVecDesc1 = Q6_Vh_vsplat_R(leftDescriptor[i / (VLEN / 2)].descriptors_lines[0].descriptor[i % (VLEN / 2)]);
        HVX_Vector leftVecDesc2 = Q6_Vh_vsplat_R(leftDescriptor[i / (VLEN / 2)].descriptors_lines[1].descriptor[i % (VLEN / 2)]);
        HVX_Vector leftVecDesc3 = Q6_Vh_vsplat_R(leftDescriptor[i / (VLEN / 2)].descriptors_lines[2].descriptor[i % (VLEN / 2)]);
        HVX_Vector leftVecDesc4 = Q6_Vh_vsplat_R(leftDescriptor[i / (VLEN / 2)].descriptors_lines[3].descriptor[i % (VLEN / 2)]);
        HVX_Vector leftVecDesc5 = Q6_Vh_vsplat_R(leftDescriptor[i / (VLEN / 2)].descriptors_lines[4].descriptor[i % (VLEN / 2)]);
        HVX_Vector leftVecDesc6 = Q6_Vh_vsplat_R(leftDescriptor[i / (VLEN / 2)].descriptors_lines[5].descriptor[i % (VLEN / 2)]);
        HVX_Vector leftVecDesc7 = Q6_Vh_vsplat_R(leftDescriptor[i / (VLEN / 2)].descriptors_lines[6].descriptor[i % (VLEN / 2)]);
        HVX_Vector leftVecDesc8 = Q6_Vh_vsplat_R(leftDescriptor[i / (VLEN / 2)].descriptors_lines[7].descriptor[i % (VLEN / 2)]);
        HVX_Vector leftVecDesc9 = Q6_Vh_vsplat_R(leftDescriptor[i / (VLEN / 2)].descriptors_lines[8].descriptor[i % (VLEN / 2)]);
        HVX_Vector leftVecDesc10 = Q6_Vh_vsplat_R(leftDescriptor[i / (VLEN / 2)].descriptors_lines[9].descriptor[i % (VLEN / 2)]);
        HVX_Vector leftVecDesc11 = Q6_Vh_vsplat_R(leftDescriptor[i / (VLEN / 2)].descriptors_lines[10].descriptor[i % (VLEN / 2)]);
        HVX_Vector leftVecDesc12 = Q6_Vh_vsplat_R(leftDescriptor[i / (VLEN / 2)].descriptors_lines[11].descriptor[i % (VLEN / 2)]);
        HVX_Vector leftVecDesc13 = Q6_Vh_vsplat_R(leftDescriptor[i / (VLEN / 2)].descriptors_lines[12].descriptor[i % (VLEN / 2)]);
        HVX_Vector leftVecDesc14 = Q6_Vh_vsplat_R(leftDescriptor[i / (VLEN / 2)].descriptors_lines[13].descriptor[i % (VLEN / 2)]);
        HVX_Vector leftVecDesc15 = Q6_Vh_vsplat_R(leftDescriptor[i / (VLEN / 2)].descriptors_lines[14].descriptor[i % (VLEN / 2)]);
        HVX_Vector leftVecDesc16 = Q6_Vh_vsplat_R(leftDescriptor[i / (VLEN / 2)].descriptors_lines[15].descriptor[i % (VLEN / 2)]);

        // Reset pointers for matching
        minDistances = minimumDistances;
        minIndices = minimumIndices;
        minDistances2 = minimumDistances2;
        minIndices2 = minimumIndices2;

        // Process right descriptors in chunks
        HVX_Vector* rightVecs = (HVX_Vector*)rightDescriptors;
        for (int j = rightCount; j >= 0; j -= VLEN / 2) {
            // Load right descriptors and compute XOR
            HVX_Vector desc1 = *rightVecs++;
            HVX_Vector xorVec1 = Q6_V_vxor_VV(leftVecDesc1, desc1);

            HVX_Vector desc2 = *rightVecs++;
            HVX_Vector xorVec2 = Q6_V_vxor_VV(leftVecDesc2, desc2);
            HVX_Vector vpopcount1 = Q6_Vh_vpopcount_Vh(xorVec1);

            HVX_Vector desc3 = *rightVecs++;
            HVX_Vector xorVec3 = Q6_V_vxor_VV(leftVecDesc3, desc3);
            HVX_Vector vpopcount2 = Q6_Vh_vpopcount_Vh(xorVec2);

            HVX_Vector desc4 = *rightVecs++;
            HVX_Vector xorVec4 = Q6_V_vxor_VV(leftVecDesc4, desc4);
            HVX_Vector vpopcount3 = Q6_Vh_vpopcount_Vh(xorVec3);

            HVX_Vector desc5 = *rightVecs++;
            HVX_Vector xorVec5 = Q6_V_vxor_VV(leftVecDesc5, desc5);
            HVX_Vector vpopcount4 = Q6_Vh_vpopcount_Vh(xorVec4);

            HVX_Vector desc6 = *rightVecs++;
            HVX_Vector xorVec6 = Q6_V_vxor_VV(leftVecDesc6, desc6);
            HVX_Vector vpopcount5 = Q6_Vh_vpopcount_Vh(xorVec5);

            HVX_Vector desc7 = *rightVecs++;
            HVX_Vector xorVec7 = Q6_V_vxor_VV(leftVecDesc7, desc7);
            HVX_Vector vpopcount6 = Q6_Vh_vpopcount_Vh(xorVec6);

            HVX_Vector desc8 = *rightVecs++;
            HVX_Vector xorVec8 = Q6_V_vxor_VV(leftVecDesc8, desc8);
            HVX_Vector vpopcount7 = Q6_Vh_vpopcount_Vh(xorVec7);

            HVX_Vector desc9 = *rightVecs++;
            HVX_Vector xorVec9 = Q6_V_vxor_VV(leftVecDesc9, desc9);
            HVX_Vector vpopcount8 = Q6_Vh_vpopcount_Vh(xorVec8);

            HVX_Vector desc10 = *rightVecs++;
            HVX_Vector xorVec10 = Q6_V_vxor_VV(leftVecDesc10, desc10);
            HVX_Vector vpopcount9 = Q6_Vh_vpopcount_Vh(xorVec9);

            HVX_Vector desc11 = *rightVecs++;
            HVX_Vector xorVec11 = Q6_V_vxor_VV(leftVecDesc11, desc11);
            HVX_Vector vpopcount10 = Q6_Vh_vpopcount_Vh(xorVec10);

            HVX_Vector desc12 = *rightVecs++;
            HVX_Vector xorVec12 = Q6_V_vxor_VV(leftVecDesc12, desc12);
            HVX_Vector vpopcount11 = Q6_Vh_vpopcount_Vh(xorVec11);

            HVX_Vector desc13 = *rightVecs++;
            HVX_Vector xorVec13 = Q6_V_vxor_VV(leftVecDesc13, desc13);
            HVX_Vector vpopcount12 = Q6_Vh_vpopcount_Vh(xorVec12);

            HVX_Vector desc14 = *rightVecs++;
            HVX_Vector xorVec14 = Q6_V_vxor_VV(leftVecDesc14, desc14);
            HVX_Vector vpopcount13 = Q6_Vh_vpopcount_Vh(xorVec13);

            HVX_Vector desc15 = *rightVecs++;
            HVX_Vector xorVec15 = Q6_V_vxor_VV(leftVecDesc15, desc15);
            HVX_Vector vpopcount14 = Q6_Vh_vpopcount_Vh(xorVec14);

            HVX_Vector desc16 = *rightVecs++;
            HVX_Vector xorVec16 = Q6_V_vxor_VV(leftVecDesc16, desc16);
            HVX_Vector vpopcount15 = Q6_Vh_vpopcount_Vh(xorVec15);

            HVX_Vector vpopcount16 = Q6_Vh_vpopcount_Vh(xorVec16);

            // Sum popcounts to compute total distance
            HVX_Vector vpopcount1_2 = Q6_Vh_vadd_VhVh(vpopcount1, vpopcount2);
            HVX_Vector vpopcount3_4 = Q6_Vh_vadd_VhVh(vpopcount3, vpopcount4);
            HVX_Vector vpopcount5_6 = Q6_Vh_vadd_VhVh(vpopcount5, vpopcount6);
            HVX_Vector vpopcount7_8 = Q6_Vh_vadd_VhVh(vpopcount7, vpopcount8);
            HVX_Vector vpopcount9_10 = Q6_Vh_vadd_VhVh(vpopcount9, vpopcount10);
            HVX_Vector vpopcount11_12 = Q6_Vh_vadd_VhVh(vpopcount11, vpopcount12);
            HVX_Vector vpopcount13_14 = Q6_Vh_vadd_VhVh(vpopcount13, vpopcount14);
            HVX_Vector vpopcount15_16 = Q6_Vh_vadd_VhVh(vpopcount15, vpopcount16);

            HVX_Vector vpopcount1_2_3_4 = Q6_Vh_vadd_VhVh(vpopcount1_2, vpopcount3_4);
            HVX_Vector vpopcount5_6_7_8 = Q6_Vh_vadd_VhVh(vpopcount5_6, vpopcount7_8);
            HVX_Vector vpopcount9_10_11_12 = Q6_Vh_vadd_VhVh(vpopcount9_10, vpopcount11_12);
            HVX_Vector vpopcount13_14_15_16 = Q6_Vh_vadd_VhVh(vpopcount13_14, vpopcount15_16);

            HVX_Vector vpopcount1_2_3_4_5_6_7_8 = Q6_Vh_vadd_VhVh(vpopcount1_2_3_4, vpopcount5_6_7_8);
            HVX_Vector vpopcount9_10_11_12_13_14_15_16 = Q6_Vh_vadd_VhVh(vpopcount9_10_11_12, vpopcount13_14_15_16);

            HVX_Vector vpopcount_total = Q6_Vh_vadd_VhVh(vpopcount1_2_3_4_5_6_7_8, vpopcount9_10_11_12_13_14_15_16);

            // Update minimum and second minimum distances
            HVX_Vector minDistances2Vec = *minDistances2;
            HVX_Vector minIndices2Vec = *minIndices2;
            HVX_VectorPred minPred2 = Q6_Q_vcmp_gt_VhVh(vpopcount_total, minDistances2Vec);
            minDistances2Vec = Q6_V_vmux_QVV(minPred2, minDistances2Vec, vpopcount_total);
            minIndices2Vec = Q6_V_vmux_QVV(minPred2, minIndices2Vec, Q6_Vh_vsplat_R(i));

            HVX_Vector minDistancesVec = *minDistances;
            HVX_Vector minIndicesVec = *minIndices;
            HVX_VectorPred minPred = Q6_Q_vcmp_gt_VhVh(minDistances2Vec, minDistancesVec);
            *minDistances++ = Q6_V_vmux_QVV(minPred, minDistancesVec, minDistances2Vec);
            *minIndices++ = Q6_V_vmux_QVV(minPred, minIndicesVec, minIndices2Vec);
            *minDistances2++ = Q6_V_vmux_QVV(minPred, minDistances2Vec, minDistancesVec);
            *minIndices2++ = Q6_V_vmux_QVV(minPred, minIndices2Vec, minIndicesVec);
        }
    }

    // --- Output Extraction ---
    // Extract indices and distances from HVX vectors
    for (int i = 0; i < rightCount / 2; i++) {
        indices[2 * i] = minimumIndices[(2 * i) / 64][i % 32] & 0xFFFF;
        distances[2 * i] = minimumDistances[(2 * i) / 64][i % 32] & 0xFFFF;
        distances2[2 * i] = minimumDistances2[(2 * i) / 64][i % 32] & 0xFFFF;

        indices[2 * i + 1] = (minimumIndices2[(2 * i + 1) / 64][i % 32] & 0xFFFF0000) >> 16;
        distances[2 * i + 1] = (minimumDistances[(2 * i + 1) / 64][i % 32] & 0xFFFF0000) >> 16;
        distances2[2 * i + 1] = (minimumDistances2[(2 * i + 1) / 64][i % 32] & 0xFFFF0000) >> 16;
    }

    return 0;
}

/// @brief Prepare ORB descriptors for vectorized processing
/// @param descriptorsVect Output vectorized descriptor array
/// @param count Number of descriptors to process
/// @param descriptors Input array of ORB descriptors
/// @return 0 on success
int prepareORBDescriptorsVector(orb_descriptors_vecs_t* descriptorsVect, 
                               int count, 
                               orb_descriptor* descriptors) {
    for (int i = 0; i < count; i++) {
        for (int j = 0; j < 16; j++) {
            descriptorsVect[i / (VLEN / 2)].descriptors_lines[j].descriptor[i % (VLEN / 2)] = descriptors[i].ui16[j];
        }
    }
    return 0;
}