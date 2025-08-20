/**
 * @file orbslam_dsp_fast.cpp
 *
 * @author Gaston Rouquette (Lynx Mixed Reality)
 *
 * @brief Legacy DSP Code to calculate features based on an image, should now be replaced by Hardware Accelerators
**/


#include "orbslam_dsp_fast.h"

#include "HAP_farf.h"
#include "HAP_vtcm_mgr.h"
#include "HAP_compute_res.h"

#include "AEEStdErr.h"

#include "orbslam3.h"

#include "hvx_internal.h"

//LEGACY METHOD, UNUSED WITH THE HWA
//Return the number of features found (scalar version for testing)
int calculate_fast_features_dumb(uint8_t* image, uint32_t width, uint32_t height, int stride, uint16_t** subImageFeatures, uint32_t subImageCount, uint8_t threshold, uint32_t maxFeaturesPerSubImage, uint32_t* featuresPerSubImage){

    uint32_t featureCount = 0;
    for(int i = 3; i < height-3; i++){
        for(int j = 3; j < width-3; j++){

            int internJ = j%VLEN;
            int colJ = j/VLEN;

            uint32_t index = i*stride + j;

            const int deltaIndices[16] = {-3*stride, -3*stride+1, -2*stride+2, -stride+3, 3, stride+3, 2*stride+2, 3*stride+1, 3*stride, 3*stride-1, 2*stride-2, stride-3, -3, -stride-3, -2*stride-2, -3*stride-1};

            bool isFeature = false;
            uint32_t diffCounter = 0;
            for(int k = 0; k < 24; k++){ //Checks contiguity
                int deltaIndex = deltaIndices[(k)%16];

                if(image[index] > image[index + deltaIndex] + threshold || image[index] < image[index + deltaIndex] - threshold){
                    diffCounter++;
                }
                else{
                    diffCounter = 0;
                }

                if(diffCounter >= 9){
                    isFeature = true;
                    break;
                }
            }

            if(isFeature){
                subImageFeatures[colJ][featureCount] = i*128 + internJ;
                featureCount++;
                if(featureCount >= maxFeaturesPerSubImage*5){
                    return featureCount;
                }
            }
        }
    }

    return featureCount;
}

//LEGACY METHOD, UNUSED WITH THE HWA
//Return the number of features found (HVX OPTIMIZED VERSION)
//This implementation does not detect some points on the sides of the sub-images
//LEGACY METHOD, UNUSED WITH THE HWA
int calculate_fast_features_hvx(uint8_t* image, uint32_t width, uint32_t height, int stride, uint16_t** subImageFeatures, uint32_t subImageCount, uint8_t threshold, uint8_t minThreshold, int minFeatures, uint32_t maxFeaturesPerSubImage, uint32_t* featuresPerSubImage){

    uint32_t featureCount = 0;

    uint32_t vecWidth = stride/VLEN;

    union {
        HVX_Vector vmask;
        uint8_t smask[VLEN];
    } borderFilter;
    borderFilter.vmask = Q6_Vb_vsplat_R(255);

    borderFilter.smask[VLEN-6] = 0;
    borderFilter.smask[VLEN-5] = 0;
    borderFilter.smask[VLEN-4] = 0;
    borderFilter.smask[VLEN-3] = 0;
    borderFilter.smask[VLEN-2] = 0;
    borderFilter.smask[VLEN-1] = 0;

    if(subImageCount == 1){
        //LEFT AND RIGHT BORDERS -- TODO OPTIMIZE THIS
        for(int i = 0 ; i < HALF_PATCH_SIZE; i++){
            borderFilter.smask[i] = 0;
            borderFilter.smask[VLEN-1-i] = 0;
        }


        featuresPerSubImage[0] = calculate_fast_features_hvx_colvec(image, height - (HALF_PATCH_SIZE+6), stride, subImageFeatures[0], threshold, maxFeaturesPerSubImage, 0, 0, borderFilter.vmask);
        if(featuresPerSubImage[0] < minFeatures){
            featuresPerSubImage[0] = calculate_fast_features_hvx_colvec(image, height - (HALF_PATCH_SIZE+6), stride, subImageFeatures[0], minThreshold, maxFeaturesPerSubImage, 0, 0, borderFilter.vmask);
        }
        
        featureCount += featuresPerSubImage[0];

        return featureCount;
    }

    int bottomOffset = 1+((height-1)%80) - (HALF_PATCH_SIZE-6);

    //LEFT PART
    for(int i = 0 ; i < HALF_PATCH_SIZE; i++){
        borderFilter.smask[i] = 0;
    }

    for(int i = 0; i < vecWidth; i++){
        featuresPerSubImage[vecWidth*i] = calculate_fast_features_hvx_colvec(image + i*VLEN, 80 + 3 - HALF_PATCH_SIZE, stride, subImageFeatures[vecWidth*i], threshold, maxFeaturesPerSubImage, 0, 0, borderFilter.vmask); //TODO USE HALF_PATCH_SIZE HERE TOO
        if(featuresPerSubImage[vecWidth*i] < minFeatures){
            featuresPerSubImage[vecWidth*i] = calculate_fast_features_hvx_colvec(image + i*VLEN, 80 + 3 - HALF_PATCH_SIZE, stride, subImageFeatures[vecWidth*i], minThreshold, maxFeaturesPerSubImage, 0, 0, borderFilter.vmask);
        }

        featureCount += featuresPerSubImage[vecWidth*i];
        
        for(int j = 1; j < subImageCount-1; j++){
            featuresPerSubImage[vecWidth*i+j] = calculate_fast_features_hvx_colvec(image + i*VLEN + (80*j-3)*stride, 80 + 2*3, stride, subImageFeatures[vecWidth*i+j], threshold, maxFeaturesPerSubImage, 0, -HALF_PATCH_SIZE, borderFilter.vmask); //80*(j)-3
            if(featureCount < minFeatures){
                featuresPerSubImage[vecWidth*i+j] = calculate_fast_features_hvx_colvec(image + i*VLEN + (80*j-3)*stride, 80 + 2*3, stride, subImageFeatures[vecWidth*i+j], minThreshold, maxFeaturesPerSubImage, 0, -HALF_PATCH_SIZE, borderFilter.vmask); //80*(j)-3
            }

            featureCount += featuresPerSubImage[vecWidth*i+j];
        }
        featuresPerSubImage[vecWidth*i+(subImageCount-1)] = calculate_fast_features_hvx_colvec(image + i*VLEN + (80*(subImageCount-1)-3)*stride, bottomOffset, stride, subImageFeatures[subImageCount*i+(subImageCount-1)], threshold, maxFeaturesPerSubImage, 0, -HALF_PATCH_SIZE, borderFilter.vmask); //80*(vecWidth-1)-3
        if(featureCount < minFeatures){
            featuresPerSubImage[vecWidth*i+(subImageCount-1)] = calculate_fast_features_hvx_colvec(image + i*VLEN + (80*(subImageCount-1)-3)*stride, bottomOffset, stride, subImageFeatures[subImageCount*i+(subImageCount-1)], minThreshold, maxFeaturesPerSubImage, 0, -HALF_PATCH_SIZE, borderFilter.vmask); //80*(vecWidth-1)-3
        }

        featureCount += featuresPerSubImage[vecWidth*i+(subImageCount-1)];

        if(i == 0){
            for(int i = 0 ; i < HALF_PATCH_SIZE; i++){
                borderFilter.smask[i] = 255;
            }
        }
        if(i == vecWidth-2){
            for(int i = 0 ; i < HALF_PATCH_SIZE; i++){
                borderFilter.smask[(width%VLEN)-1-i] = 0;
            }
        }
    }

    return featureCount;
}

//LEGACY METHOD, UNUSED WITH THE HWA
int calculate_fast_features_hvx_colvec(uint8_t* imageRow, uint32_t height, int stride, uint16_t* features, uint8_t threshold, uint32_t maxFeatures, uint32_t featureCount, uint32_t yoffset, HVX_Vector predmaskInit){


    int loopOffset = yoffset == 0 ? 0 : -3;

    HVX_Vector *inputCursorL = (HVX_Vector*)(imageRow);
    HVX_Vector *inputCursorR = (HVX_Vector*)(imageRow + VLEN);

    if(yoffset == 0){
        inputCursorL = (HVX_Vector*)(imageRow + HALF_PATCH_SIZE*stride);
        inputCursorR = (HVX_Vector*)(imageRow + VLEN + HALF_PATCH_SIZE*stride);
        yoffset = -HALF_PATCH_SIZE;
    }


    l2fetch(inputCursorL, stride, stride, 7, 1);

    uint32_t deltaLineVec = stride/VLEN;

    HVX_Vector vthreshold = Q6_Vb_vsplat_R(threshold); //USE THE RIGHT TYPE OF SPLAT!!!

    HVX_Vector stairs = Q6_Vb_vsub_VbVb(Q6_Vb_prefixsum_Q(Q6_Vb_vsplat_R(1)), Q6_Vb_vsplat_R(1)); //We create a vector with the values 0,1,2,3,4,5,6,7,...,VLEN-1

    //Stairs 16 bits
    HVX_VectorPair stairsHF = Q6_Wuh_vzxt_Vub(stairs);
    HVX_Vector stairsHi = Q6_V_hi_W(stairsHF);
    HVX_Vector stairsLo = Q6_V_lo_W(stairsHF);

    HVX_Vector inputT3L = *inputCursorL;
    inputCursorL += deltaLineVec;
    HVX_Vector inputT2L = *inputCursorL;
    inputCursorL += deltaLineVec;
    HVX_Vector inputT1L = *inputCursorL;
    inputCursorL += deltaLineVec;
    HVX_Vector inputCL = *inputCursorL;
    inputCursorL += deltaLineVec;
    HVX_Vector inputB1L = *inputCursorL;
    inputCursorL += deltaLineVec;
    HVX_Vector inputB2L = *inputCursorL;
    inputCursorL += deltaLineVec;
    HVX_Vector inputB3L = *inputCursorL;
    inputCursorL += deltaLineVec;

    //We don't use right values here

    HVX_Vector zeroVector = Q6_V_vzero();
    
    union {
        HVX_Vector vmask;
        uint8_t smask[VLEN];
    } borderFilter;
    borderFilter.vmask = predmaskInit;

    borderFilter.smask[VLEN-6] = 0;
    borderFilter.smask[VLEN-5] = 0;
    borderFilter.smask[VLEN-4] = 0;
    borderFilter.smask[VLEN-3] = 0;
    borderFilter.smask[VLEN-2] = 0;
    borderFilter.smask[VLEN-1] = 0;

    for(int j = 3 + loopOffset; j < height-3 + loopOffset; j++){
        l2fetch(inputCursorL, stride, stride, 1, 0);

        //32 REGISTRES VECTEURS DISPOS, ON ESSAYE D'EN UTILISER ENVIRON 16~24 en local
        HVX_Vector vc, v1, v5, v9, v13;
        HVX_Vector diff1, diff5, diff9, diff13;
        HVX_Vector premaskTest, tempMask1, tempMask2, tempMask3, tempMask4;
        HVX_Vector mask1, mask5, mask9, mask13;
        union {
            HVX_Vector vqbitsum;
            uint8_t sqbitsum_u8[VLEN];
        } tempOK;

        //First 4 pixels (Even if this is less powerful than for SIMT, we can still use the same approach, when we have a full vector of pixels that are clearly not features, we can skip them)

        //Align the vectors
        
        vc = Q6_V_valign_VVI(Q6_V_vzero(), inputCL, 3);
        v1 = Q6_V_valign_VVI(Q6_V_vzero(), inputT3L, 3);
        v5 = Q6_V_valign_VVI(Q6_V_vzero(), inputCL,  6);
        v9 = Q6_V_valign_VVI(Q6_V_vzero(), inputB3L, 3);
        v13 = Q6_V_valign_VVI(Q6_V_vzero(), inputCL, 0);

        diff1 = Q6_Vub_vabsdiff_VubVub(v1, vc);
        diff5 = Q6_Vub_vabsdiff_VubVub(v5, vc);
        diff9 = Q6_Vub_vabsdiff_VubVub(v9, vc);
        diff13 = Q6_Vub_vabsdiff_VubVub(v13, vc);

        mask1 = Q6_Q_vcmp_gt_VubVub(diff1, vthreshold);        
        mask5 = Q6_Q_vcmp_gt_VubVub(diff5, vthreshold);
        mask9 = Q6_Q_vcmp_gt_VubVub(diff9, vthreshold);
        mask13 = Q6_Q_vcmp_gt_VubVub(diff13, vthreshold);

        //Create mask for each combination of pixels

        tempMask1 = Q6_Q_and_QQ(mask5, mask9);
        tempMask2 = Q6_Q_and_QQ(mask9, mask13);
        tempMask3 = Q6_Q_and_QQ(mask13, mask1);

        premaskTest = Q6_Q_and_QQ(mask1, mask5);
        premaskTest = Q6_Q_or_QQ(premaskTest, tempMask1);
        premaskTest = Q6_Q_or_QQ(premaskTest, tempMask2);
        premaskTest = Q6_Q_or_QQ(premaskTest, tempMask3);
    
        tempOK.vqbitsum = Q6_Vb_prefixsum_Q(premaskTest);

        //If vector is less than threshold, we can skip it
        if(tempOK.sqbitsum_u8[VLEN-1] != 0){
            //1st contiguity check

            HVX_Vector v2, v6, v10, v14;
            HVX_Vector diff2, diff6, diff10, diff14;
            HVX_Vector mask2, mask6, mask10, mask14;

            HVX_Vector sumMask = Q6_Vb_vadd_VbVb(mask1, mask5);
            sumMask = Q6_Vb_vadd_VbVb(sumMask, mask9);
            sumMask = Q6_Vb_vadd_VbVb(sumMask, mask13);

            //Align the vectors
            v2 = Q6_V_valign_VVI(Q6_V_vzero(), inputT3L, 4);
            v6 = Q6_V_valign_VVI(Q6_V_vzero(), inputB1L, 6);
            v10 = Q6_V_valign_VVI(Q6_V_vzero(), inputB3L, 2);
            v14 = Q6_V_valign_VVI(Q6_V_vzero(), inputT1L, 0);

            diff2 = Q6_Vub_vabsdiff_VubVub(v2, vc);
            diff6 = Q6_Vub_vabsdiff_VubVub(v6, vc);
            diff10 = Q6_Vub_vabsdiff_VubVub(v10, vc);
            diff14 = Q6_Vub_vabsdiff_VubVub(v14, vc);        

            mask2 = Q6_Q_vcmp_gt_VubVub(diff2, vthreshold);
            mask6 = Q6_Q_vcmp_gt_VubVub(diff6, vthreshold);
            mask10 = Q6_Q_vcmp_gt_VubVub(diff10, vthreshold);
            mask14 = Q6_Q_vcmp_gt_VubVub(diff14, vthreshold);

            tempMask1 = Q6_Q_and_QQ(mask2, mask6);
            tempMask2 = Q6_Q_and_QQ(mask6, mask10);
            tempMask3 = Q6_Q_and_QQ(mask10, mask14);
            tempMask4 = Q6_Q_and_QQ(mask14, mask2);

            tempMask1 = Q6_Q_or_QQ(tempMask1, tempMask2);
            tempMask3 = Q6_Q_or_QQ(tempMask3, tempMask4);

            tempMask1 = Q6_Q_or_QQ(tempMask1, tempMask3);
            premaskTest = Q6_Q_and_QQ(premaskTest, tempMask1);

            tempOK.vqbitsum = Q6_Vb_prefixsum_Q(premaskTest);

            //---------------------------------------------- SECOND CONTIGUITY CHECKS ----------------------------------------------
            if(tempOK.sqbitsum_u8[VLEN-1] != 0){
                sumMask = Q6_Vb_vadd_VbVb(sumMask, mask2);
                sumMask = Q6_Vb_vadd_VbVb(sumMask, mask6);
                sumMask = Q6_Vb_vadd_VbVb(sumMask, mask10); 
                sumMask = Q6_Vb_vadd_VbVb(sumMask, mask14);

                HVX_Vector v3, v7, v11, v15;
                HVX_Vector diff3, diff7, diff11, diff15;
                HVX_Vector mask3, mask7, mask11, mask15;

                v3 = Q6_V_valign_VVI(Q6_V_vzero(), inputT2L, 5);
                v7 = Q6_V_valign_VVI(Q6_V_vzero(), inputB2L, 5);
                v11 = Q6_V_valign_VVI(Q6_V_vzero(), inputB2L, 1);
                v15 = Q6_V_valign_VVI(Q6_V_vzero(), inputT2L, 1);

                diff3 = Q6_Vub_vabsdiff_VubVub(v3, vc);
                diff7 = Q6_Vub_vabsdiff_VubVub(v7, vc);
                diff11 = Q6_Vub_vabsdiff_VubVub(v11, vc);
                diff15 = Q6_Vub_vabsdiff_VubVub(v15, vc);

                mask3 = Q6_Q_vcmp_gt_VubVub(diff3, vthreshold);
                mask7 = Q6_Q_vcmp_gt_VubVub(diff7, vthreshold);
                mask11 = Q6_Q_vcmp_gt_VubVub(diff11, vthreshold);
                mask15 = Q6_Q_vcmp_gt_VubVub(diff15, vthreshold);

                tempMask1 = Q6_Q_and_QQ(mask3, mask7);
                tempMask2 = Q6_Q_and_QQ(mask7, mask11);
                tempMask3 = Q6_Q_and_QQ(mask11, mask15);
                tempMask4 = Q6_Q_and_QQ(mask15, mask3);

                tempMask1 = Q6_Q_or_QQ(tempMask1, tempMask2);
                tempMask3 = Q6_Q_or_QQ(tempMask3, tempMask4);

                tempMask1 = Q6_Q_or_QQ(tempMask1, tempMask3);
                premaskTest = Q6_Q_and_QQ(premaskTest, tempMask1);

                tempOK.vqbitsum = Q6_Vb_prefixsum_Q(premaskTest);

                //THIRD CONTIGUITY CHECKS
                if(tempOK.sqbitsum_u8[VLEN-1] != 0){

                    sumMask = Q6_Vb_vadd_VbVb(sumMask, mask3);
                    sumMask = Q6_Vb_vadd_VbVb(sumMask, mask7);
                    sumMask = Q6_Vb_vadd_VbVb(sumMask, mask11);
                    sumMask = Q6_Vb_vadd_VbVb(sumMask, mask15);

                    HVX_Vector v4, v8, v12, v16;
                    HVX_Vector diff4, diff8, diff12, diff16;
                    HVX_Vector mask4, mask8, mask12, mask16;

                    v4 = Q6_V_valign_VVI(Q6_V_vzero(), inputT1L, 6);
                    v8 = Q6_V_valign_VVI(Q6_V_vzero(), inputB3L, 4);
                    v12 = Q6_V_valign_VVI(Q6_V_vzero(), inputB1L, 0);
                    v16 = Q6_V_valign_VVI(Q6_V_vzero(), inputT3L, 2);

                    diff4 = Q6_Vub_vabsdiff_VubVub(v4, vc);
                    diff8 = Q6_Vub_vabsdiff_VubVub(v8, vc);
                    diff12 = Q6_Vub_vabsdiff_VubVub(v12, vc);
                    diff16 = Q6_Vub_vabsdiff_VubVub(v16, vc);

                    mask4 = Q6_Q_vcmp_gt_VubVub(diff4, vthreshold);
                    mask8 = Q6_Q_vcmp_gt_VubVub(diff8, vthreshold);
                    mask12 = Q6_Q_vcmp_gt_VubVub(diff12, vthreshold);
                    mask16 = Q6_Q_vcmp_gt_VubVub(diff16, vthreshold);

                    tempMask1 = Q6_Q_and_QQ(mask4, mask8);
                    tempMask2 = Q6_Q_and_QQ(mask8, mask12);
                    tempMask3 = Q6_Q_and_QQ(mask12, mask16);
                    tempMask4 = Q6_Q_and_QQ(mask16, mask4);

                    tempMask1 = Q6_Q_or_QQ(tempMask1, tempMask2);
                    tempMask3 = Q6_Q_or_QQ(tempMask3, tempMask4);

                    tempMask1 = Q6_Q_or_QQ(tempMask1, tempMask3);
                    premaskTest = Q6_Q_and_QQ(premaskTest, tempMask1);

                    tempOK.vqbitsum = Q6_Vb_prefixsum_Q(premaskTest);

                    //FINAL CONTINUGITY CHECK
                    if(tempOK.sqbitsum_u8[VLEN-1] != 0){
                        sumMask = Q6_Vb_vadd_VbVb(sumMask, mask4);
                        sumMask = Q6_Vb_vadd_VbVb(sumMask, mask8);
                        sumMask = Q6_Vb_vadd_VbVb(sumMask, mask12);
                        sumMask = Q6_Vb_vadd_VbVb(sumMask, mask16);

                        //Store the features

                        HVX_Vector maskHi, maskLo, GT3Hi, GT3Lo;
                        HVX_VectorPair maskHF, GT3HF;
                        HVX_Vector sumGT3 = Q6_Q_and_QQ(Q6_Q_and_QQ(premaskTest, borderFilter.vmask), Q6_Q_vcmp_gt_VubVub(Q6_Vb_vsplat_R(256-8-1), sumMask - Q6_Vb_vsplat_R(1)));

                        //EXTRACT THE FEATURES

                        union {
                            HVX_Vector vqbitsum;
                            uint8_t sqbitsum_u8[VLEN];
                        } uu;
                        
                        
                        uu.vqbitsum = Q6_Vb_prefixsum_Q(sumGT3);            

                        uint8_t validBits = uu.sqbitsum_u8[VLEN-1];

                        uint32_t newFeaturesCount = featureCount + (uint32_t)validBits;
                        if(newFeaturesCount >= maxFeatures){
                            //FARF(HIGH, "Feature Count TOO MANY: %lu\n", featureCount);
                            return featureCount;
                        }

                        if(validBits){
                            maskHF = Q6_Wuh_vzxt_Vub(Q6_Vub_vsub_VubVub_sat(uu.vqbitsum, Q6_Vb_vsplat_R(1)));
                            maskHi = Q6_V_hi_W(maskHF);
                            maskLo = Q6_V_lo_W(maskHF);

                            GT3HF = Q6_Wh_vsxt_Vb(sumGT3);
                            GT3Hi = Q6_V_hi_W(GT3HF);
                            GT3Lo = Q6_V_lo_W(GT3HF);

                            HVX_Vector offsetVecHi = Q6_Vh_vasl_VhR(maskHi, 1);
                            Q6_vscatter_QRMVhV(GT3Hi, (uint32_t)features+featureCount*sizeof(uint16_t), (maxFeatures*sizeof(uint16_t))-1, offsetVecHi, stairsHi + Q6_Vh_vsplat_R(j*VLEN*2+VLEN*(1-2*yoffset) + 3)); //NEW CACHE HAS A CACHE OF 2VLEN, and a offset of 128 + 3 lines (may need to be updated for later)

                            HVX_Vector offsetVecLo = Q6_Vh_vasl_VhR(maskLo, 1);
                            Q6_vscatter_QRMVhV(GT3Lo, (uint32_t)features+featureCount*sizeof(uint16_t), (maxFeatures*sizeof(uint16_t))-1, offsetVecLo, stairsLo + Q6_Vh_vsplat_R(j*VLEN*2+VLEN*(1-2*yoffset) + 3)); //NEW CACHE HAS A CACHE OF 2VLEN

                            featureCount = newFeaturesCount;
                        }
                    }
                }
            }
        }
                


        //Translate and load
        inputT3L = inputT2L;
        inputT2L = inputT1L;
        inputT1L = inputCL;
        inputCL = inputB1L;
        inputB1L = inputB2L;
        inputB2L = inputB3L;

        inputB3L = *inputCursorL;
        inputCursorL+=deltaLineVec;
    }

    return featureCount;

}

int calculate_fast_scores(uint8_t* image, uint32_t imageHeight, uint32_t imageStride, pyramid_VTCM_cache_t* vtcmImageCache, uint16_t** subImageFeatures, uint16_t** subImageFeaturesScores, uint32_t subImageCount, uint32_t* featuresPerSubImage){
    uint32_t strideCount = imageStride/VLEN;

    if(subImageCount == 1){
        cache_stripe_vtcm_2_both_borders(image, vtcmImageCache, imageHeight, imageStride); 
        calculate_fast_scores_stride(vtcmImageCache, imageHeight, imageStride, subImageFeatures[0], subImageFeaturesScores[0], featuresPerSubImage[0], 0);
        return 0;
    }

    int bottomOffset = 1+((imageHeight-1)%80);

    cache_stripe_vtcm_2_border_left(image, vtcmImageCache, 80+HALF_PATCH_SIZE, imageStride); 
    calculate_fast_scores_stride(vtcmImageCache, 80+HALF_PATCH_SIZE, 2*VLEN, subImageFeatures[0], subImageFeaturesScores[0], featuresPerSubImage[0], 0);
    for(int j = 1; j < subImageCount-1; j++){
        cache_stripe_vtcm_2_border_left(image + (80*j-HALF_PATCH_SIZE)*imageStride, vtcmImageCache, 80+2*HALF_PATCH_SIZE, imageStride); 
        calculate_fast_scores_stride(vtcmImageCache, 80+2*HALF_PATCH_SIZE, 2*VLEN, subImageFeatures[j], subImageFeaturesScores[j], featuresPerSubImage[j], 0);
    }
    cache_stripe_vtcm_2_border_left(image + (80*(subImageCount-1)-HALF_PATCH_SIZE)*imageStride, vtcmImageCache, bottomOffset+HALF_PATCH_SIZE, imageStride); 
    calculate_fast_scores_stride(vtcmImageCache, bottomOffset+HALF_PATCH_SIZE, 2*VLEN, subImageFeatures[subImageCount-1], subImageFeaturesScores[subImageCount-1], featuresPerSubImage[subImageCount-1], 0);


    for(int i = 1; i < subImageCount-1; i++){
        cache_stripe_vtcm_2_no_border(image + i*VLEN, vtcmImageCache, 80+HALF_PATCH_SIZE, imageStride); 
        calculate_fast_scores_stride(vtcmImageCache, 80+HALF_PATCH_SIZE, 2*VLEN, subImageFeatures[subImageCount*i], subImageFeaturesScores[subImageCount*i], featuresPerSubImage[subImageCount*i], 0);
        for(int j = 1; j < subImageCount-1; j++){
            cache_stripe_vtcm_2_no_border(image + i*VLEN + (80*j-HALF_PATCH_SIZE)*imageStride, vtcmImageCache, 80+2*HALF_PATCH_SIZE, imageStride); 
            calculate_fast_scores_stride(vtcmImageCache, 80+2*HALF_PATCH_SIZE, 2*VLEN, subImageFeatures[subImageCount*i+j], subImageFeaturesScores[subImageCount*i+j], featuresPerSubImage[subImageCount*i+j], 0);
        }
        cache_stripe_vtcm_2_no_border(image + i*VLEN + (80*(subImageCount-1)-HALF_PATCH_SIZE)*imageStride, vtcmImageCache, bottomOffset+HALF_PATCH_SIZE, imageStride); 
        calculate_fast_scores_stride(vtcmImageCache, bottomOffset+HALF_PATCH_SIZE, 2*VLEN, subImageFeatures[subImageCount*i+subImageCount-1], subImageFeaturesScores[subImageCount*i+subImageCount-1], featuresPerSubImage[subImageCount*i+subImageCount-1], 0);
    }

    cache_stripe_vtcm_2_border_right(image + (subImageCount-1)*VLEN, vtcmImageCache, 80+HALF_PATCH_SIZE, imageStride); 
    calculate_fast_scores_stride(vtcmImageCache, 80+HALF_PATCH_SIZE, 2*VLEN, subImageFeatures[subImageCount*(subImageCount-1)], subImageFeaturesScores[subImageCount*(subImageCount-1)], featuresPerSubImage[subImageCount*(subImageCount-1)], 0);
    for(int j = 1; j < subImageCount-1; j++){
        cache_stripe_vtcm_2_border_right(image + (subImageCount-1)*VLEN + (80*j-HALF_PATCH_SIZE)*imageStride, vtcmImageCache, 80+2*HALF_PATCH_SIZE, imageStride); 
        calculate_fast_scores_stride(vtcmImageCache, 80+2*HALF_PATCH_SIZE, 2*VLEN, subImageFeatures[subImageCount*(subImageCount-1)+j], subImageFeaturesScores[subImageCount*(subImageCount-1)+j], featuresPerSubImage[subImageCount*(subImageCount-1)+j], 0);
    }
    cache_stripe_vtcm_2_border_right(image + (subImageCount-1)*VLEN + (80*(subImageCount-1)-HALF_PATCH_SIZE)*imageStride, vtcmImageCache, bottomOffset+HALF_PATCH_SIZE, imageStride); 
    calculate_fast_scores_stride(vtcmImageCache, bottomOffset+HALF_PATCH_SIZE, 2*VLEN, subImageFeatures[subImageCount*(subImageCount-1)+subImageCount-1], subImageFeaturesScores[subImageCount*(subImageCount-1)+subImageCount-1], featuresPerSubImage[subImageCount*(subImageCount-1)+subImageCount-1], 0);


    return 0;
}

int calculate_fast_scores_stride(pyramid_VTCM_cache_t* vtcmImageCache, uint32_t height, int stride, uint16_t* features, uint16_t* featuresScore, uint32_t featureCount, uint32_t yoffset){
    if(featureCount == 0){
        return 0;
    }
    
    uint32_t iterationsCount = (featureCount-1)/(VLEN/2) + 1; // /2 because 16 bits

    HVX_Vector *inputCursor = (HVX_Vector*)(features);
    HVX_Vector *outputCursor = (HVX_Vector*)(featuresScore);
    
    for(int i = 0; i < iterationsCount; i++){ //IGNORE PIXELS ON THE BORDERS (May be not a problem?)
        HVX_Vector centerIndices = (*inputCursor); 
        inputCursor++;

        //We have to calculate the indices of the 16 pixels around the center pixel
        HVX_Vector p1Indices, p2Indices, p3Indices, p4Indices, p5Indices, p6Indices, p7Indices, p8Indices, p9Indices, p10Indices, p11Indices, p12Indices, p13Indices, p14Indices, p15Indices, p16Indices; 
        HVX_Vector borderPixels[24];
        HVX_Vector centerPixels;

        //Stairs method VLIW Optimization
        p1Indices = Q6_Vh_vadd_VhVh(centerIndices, Q6_Vh_vsplat_R((-3*VLEN*2))); //Should not overflow !
        Q6_vgather_ARMVh(&vtcmImageCache->gatheredVector1, (uint32_t)vtcmImageCache->cacheData, height*VLEN*2-1, centerIndices);

        p2Indices = Q6_Vh_vadd_VhVh(centerIndices, Q6_Vh_vsplat_R((-3*VLEN*2 + 1)));
        p3Indices = Q6_Vh_vadd_VhVh(centerIndices, Q6_Vh_vsplat_R((-2*VLEN*2 + 2)));


        Q6_vgather_ARMVh(&vtcmImageCache->gatheredVector2, (uint32_t)vtcmImageCache->cacheData, height*VLEN*2-1, p1Indices);
        centerPixels = Q6_Vb_vshuffe_VbVb(Q6_V_vzero(), vtcmImageCache->gatheredVector1);
        p4Indices = Q6_Vh_vadd_VhVh(centerIndices, Q6_Vh_vsplat_R((-1*VLEN*2 + 3)));

        Q6_vgather_ARMVh(&vtcmImageCache->gatheredVector3, (uint32_t)vtcmImageCache->cacheData, height*VLEN*2-1, p2Indices);


        p5Indices = Q6_Vh_vadd_VhVh(centerIndices, Q6_Vh_vsplat_R((3)));
        p6Indices = Q6_Vh_vadd_VhVh(centerIndices, Q6_Vh_vsplat_R((1*VLEN*2 + 3)));

        p7Indices = Q6_Vh_vadd_VhVh(centerIndices, Q6_Vh_vsplat_R((2*VLEN*2 + 2)));

        p8Indices = Q6_Vh_vadd_VhVh(centerIndices, Q6_Vh_vsplat_R((3*VLEN*2 + 1)));
        p9Indices = Q6_Vh_vadd_VhVh(centerIndices, Q6_Vh_vsplat_R((3*VLEN*2)));
        p10Indices = Q6_Vh_vadd_VhVh(centerIndices, Q6_Vh_vsplat_R((3*VLEN*2 - 1)));

        p11Indices = Q6_Vh_vadd_VhVh(centerIndices, Q6_Vh_vsplat_R((2*VLEN*2 - 2)));

        p12Indices = Q6_Vh_vadd_VhVh(centerIndices, Q6_Vh_vsplat_R((1*VLEN*2 - 3)));
        p13Indices = Q6_Vh_vadd_VhVh(centerIndices, Q6_Vh_vsplat_R((-3)));
        p14Indices = Q6_Vh_vadd_VhVh(centerIndices, Q6_Vh_vsplat_R((-1*VLEN*2 - 3)));

        p15Indices = Q6_Vh_vadd_VhVh(centerIndices, Q6_Vh_vsplat_R((-2*VLEN*2 - 2)));        

        p16Indices = Q6_Vh_vadd_VhVh(centerIndices, Q6_Vh_vsplat_R((-3*VLEN*2 - 1)));     

        //Load everything
        Q6_vgather_ARMVh(&vtcmImageCache->gatheredVector4, (uint32_t)vtcmImageCache->cacheData, height*VLEN*2-1, p3Indices);


        borderPixels[0] = Q6_Vb_vshuffe_VbVb(Q6_V_vzero(), vtcmImageCache->gatheredVector2);
        borderPixels[1] = Q6_Vb_vshuffe_VbVb(Q6_V_vzero(), vtcmImageCache->gatheredVector3);

        Q6_vgather_ARMVh(&vtcmImageCache->gatheredVector1, (uint32_t)vtcmImageCache->cacheData, height*VLEN*2-1, p4Indices);
        borderPixels[2] = Q6_Vb_vshuffe_VbVb(Q6_V_vzero(), vtcmImageCache->gatheredVector4);
        borderPixels[3] = Q6_Vb_vshuffe_VbVb(Q6_V_vzero(), vtcmImageCache->gatheredVector1);

        Q6_vgather_ARMVh(&vtcmImageCache->gatheredVector2, (uint32_t)vtcmImageCache->cacheData, height*VLEN*2-1, p5Indices);
        borderPixels[4] = Q6_Vb_vshuffe_VbVb(Q6_V_vzero(), vtcmImageCache->gatheredVector2);

        Q6_vgather_ARMVh(&vtcmImageCache->gatheredVector3, (uint32_t)vtcmImageCache->cacheData, height*VLEN*2-1, p6Indices);
        borderPixels[5] = Q6_Vb_vshuffe_VbVb(Q6_V_vzero(), vtcmImageCache->gatheredVector3);

        Q6_vgather_ARMVh(&vtcmImageCache->gatheredVector4, (uint32_t)vtcmImageCache->cacheData, height*VLEN*2-1, p7Indices);
        borderPixels[6] = Q6_Vb_vshuffe_VbVb(Q6_V_vzero(), vtcmImageCache->gatheredVector4);

        Q6_vgather_ARMVh(&vtcmImageCache->gatheredVector1, (uint32_t)vtcmImageCache->cacheData, height*VLEN*2-1, p8Indices);
        borderPixels[7] = Q6_Vb_vshuffe_VbVb(Q6_V_vzero(), vtcmImageCache->gatheredVector1);

        Q6_vgather_ARMVh(&vtcmImageCache->gatheredVector2, (uint32_t)vtcmImageCache->cacheData, height*VLEN*2-1, p9Indices);
        borderPixels[8] = Q6_Vb_vshuffe_VbVb(Q6_V_vzero(), vtcmImageCache->gatheredVector2);

        Q6_vgather_ARMVh(&vtcmImageCache->gatheredVector3, (uint32_t)vtcmImageCache->cacheData, height*VLEN*2-1, p10Indices);
        borderPixels[9] = Q6_Vb_vshuffe_VbVb(Q6_V_vzero(), vtcmImageCache->gatheredVector3);

        Q6_vgather_ARMVh(&vtcmImageCache->gatheredVector4, (uint32_t)vtcmImageCache->cacheData, height*VLEN*2-1, p11Indices);
        borderPixels[10] = Q6_Vb_vshuffe_VbVb(Q6_V_vzero(), vtcmImageCache->gatheredVector4);

        Q6_vgather_ARMVh(&vtcmImageCache->gatheredVector1, (uint32_t)vtcmImageCache->cacheData, height*VLEN*2-1, p12Indices);
        borderPixels[11] = Q6_Vb_vshuffe_VbVb(Q6_V_vzero(), vtcmImageCache->gatheredVector1);

        Q6_vgather_ARMVh(&vtcmImageCache->gatheredVector2, (uint32_t)vtcmImageCache->cacheData, height*VLEN*2-1, p13Indices);
        borderPixels[12] = Q6_Vb_vshuffe_VbVb(Q6_V_vzero(), vtcmImageCache->gatheredVector2);

        Q6_vgather_ARMVh(&vtcmImageCache->gatheredVector3, (uint32_t)vtcmImageCache->cacheData, height*VLEN*2-1, p14Indices);
        borderPixels[13] = Q6_Vb_vshuffe_VbVb(Q6_V_vzero(), vtcmImageCache->gatheredVector3);

        Q6_vgather_ARMVh(&vtcmImageCache->gatheredVector4, (uint32_t)vtcmImageCache->cacheData, height*VLEN*2-1, p15Indices);
        borderPixels[14] = Q6_Vb_vshuffe_VbVb(Q6_V_vzero(), vtcmImageCache->gatheredVector4);

        Q6_vgather_ARMVh(&vtcmImageCache->gatheredVector1, (uint32_t)vtcmImageCache->cacheData, height*VLEN*2-1, p16Indices);
        borderPixels[15] = Q6_Vb_vshuffe_VbVb(Q6_V_vzero(), vtcmImageCache->gatheredVector1);



        //COPY border pixel 1 in border pixel 16 ect
        for(int i = 0; i < 8; i++){
            borderPixels[16+i] = borderPixels[i];
        }


        HVX_Vector score_b = Q6_V_vzero();
        HVX_Vector score_d = Q6_Vh_vsplat_R(255);

        for(int startPos = 0; startPos < 16; startPos+=2){  //Optimisation
            HVX_Vector v0 = borderPixels[(startPos+1)];
            HVX_Vector v1 = borderPixels[(startPos+2)];
            HVX_Vector a = Q6_Vh_vmin_VhVh(v0, v1);
            HVX_Vector b = Q6_Vh_vmax_VhVh(v0, v1);
            v0 = borderPixels[(startPos+3)];
            a = Q6_Vh_vmin_VhVh(a, v0);
            b = Q6_Vh_vmax_VhVh(b, v0);
            v0 = borderPixels[(startPos+4)];
            a = Q6_Vh_vmin_VhVh(a, v0);
            b = Q6_Vh_vmax_VhVh(b, v0);
            v0 = borderPixels[(startPos+5)];
            a = Q6_Vh_vmin_VhVh(a, v0);
            b = Q6_Vh_vmax_VhVh(b, v0);
            v0 = borderPixels[(startPos+6)];
            a = Q6_Vh_vmin_VhVh(a, v0);
            b = Q6_Vh_vmax_VhVh(b, v0);
            v0 = borderPixels[(startPos+7)];
            a = Q6_Vh_vmin_VhVh(a, v0);
            b = Q6_Vh_vmax_VhVh(b, v0);
            v0 = borderPixels[(startPos+8)];
            a = Q6_Vh_vmin_VhVh(a, v0);
            b = Q6_Vh_vmax_VhVh(b, v0);
            v0 = borderPixels[(startPos)];
            score_b = Q6_Vh_vmax_VhVh(score_b, Q6_Vh_vmin_VhVh(v0, a));
            score_d = Q6_Vh_vmin_VhVh(score_d, Q6_Vh_vmax_VhVh(v0, b));
            v0 = borderPixels[(startPos+9)];
            score_b = Q6_Vh_vmax_VhVh(score_b, Q6_Vh_vmin_VhVh(v0, a));
            score_d = Q6_Vh_vmin_VhVh(score_d, Q6_Vh_vmax_VhVh(v0, b));
        }

        HVX_Vector score = Q6_Vh_vmax_VhVh(Q6_Vh_vsub_VhVh_sat(score_b, centerPixels), Q6_Vh_vsub_VhVh_sat(centerPixels, score_d));

        score = Q6_Vh_vmax_VhVh(score, Q6_Vh_vsplat_R(0));

        *outputCursor++ = score;
    }

    return 0;
}

int calculate_non_maximal_suppression(uint32_t imageHeight, uint16_t** subImageFeatures, uint16_t** subImageFeaturesScores, uint32_t subImageCount, uint32_t* featuresPerSubImage, int maxFeaturesPerSubImage, int minimumThreshold){

    //Horizontal non-maximal suppression
    for(int i = 0; i < subImageCount*subImageCount; i++){
        featuresPerSubImage[i] = calculate_non_maximal_suppression_stride(featuresPerSubImage[i], imageHeight, subImageFeatures[i],  subImageFeaturesScores[i], maxFeaturesPerSubImage, minimumThreshold);
    }


    return 0;
}

int calculate_non_maximal_suppression_stride(uint32_t featuresCount, uint32_t imageHeight, uint16_t* subImageFeatures, uint16_t* subImageFeaturesScores, int maxFeaturesPerSubImage, int minimumThreshold){
    //Calculate non-maximal horizontal suppression

    if(featuresCount == 0){
        //FARF(LOW, "Skipping empty subimage (calculate_non_maximal_suppression_stride)\n");
        return 0;
    }

    calculate_non_maximal_horizontal_suppression_stride(featuresCount, imageHeight, subImageFeatures,  subImageFeaturesScores);

    //Maybe add a temporary memory for transposed positions?
    uint32_t iterationsCount = (featuresCount-1)/(VLEN/2) + 1; 

    HVX_Vector *inputPositionsCursor = (HVX_Vector*)(subImageFeatures);
    HVX_Vector *inputScoresCursor = (HVX_Vector*)(subImageFeaturesScores);

    for(int i = 0; i < iterationsCount; i++){
        HVX_Vector centerPositionsVector = (*inputPositionsCursor);
        /*
        * FOR EACH NUMBER
        * x = i*128 + j (0 <= j < 128)
        * We want y = j*512 + i (0 <= i < 512 (no image has more height than 512))
        * Finally we sort the indices
        * 
        * i = x >> 7
        * j = x & 1111111b = x & 0x7F
        * j*512 = j << 9
        * y = j*512 + i
        * 
        * SORT
        */
        
        HVX_Vector i_vector = Q6_Vh_vlsr_VhVh(centerPositionsVector, Q6_Vh_vsplat_R(8));
        HVX_Vector j_vector = Q6_V_vand_VV(centerPositionsVector, Q6_Vh_vsplat_R(0xFF));

        HVX_Vector j_256_vector = Q6_Vh_vasl_VhR(j_vector, 8);
        HVX_Vector y_vector = j_256_vector + i_vector;

        (*inputPositionsCursor) = y_vector; //Random point to avoid array out of bounds when future calculations occurs if this is at the end of the last vector as useless filling data.

        inputPositionsCursor++;
    }

    bitonic_sort_uint16_increasing_hvx(subImageFeaturesScores, subImageFeatures, (featuresCount-1)/(VLEN/2)+1);

    //Now indices are vertically sorted, so it becomes vertical non-maximal suppression
    calculate_non_maximal_horizontal_suppression_stride(featuresCount, imageHeight, subImageFeatures, subImageFeaturesScores);
    
    return featuresCount;
}

int calculate_non_maximal_horizontal_suppression_stride(uint32_t featuresCount, uint32_t imageHeight, uint16_t* subImageFeatures, uint16_t* subImageFeaturesScores){

    uint32_t iterationsCount = (featuresCount-1)/(VLEN/2) + 1; 

    HVX_Vector *inputPositionsCursor = (HVX_Vector*)(subImageFeatures);
    HVX_Vector *scoresCursor = (HVX_Vector*)(subImageFeaturesScores);

    for(int i = 0; i < iterationsCount; i++){
        HVX_Vector centerPositionsVector = (*inputPositionsCursor);
        inputPositionsCursor++;
        HVX_Vector leftPositionsVector = Q6_V_valign_VVR(centerPositionsVector, Q6_V_vzero(), VLEN-2);
        HVX_Vector rightPositionVector = Q6_V_valign_VVI(Q6_V_vzero(), centerPositionsVector, 2);

        HVX_Vector centerScoresVector = (*scoresCursor);
        HVX_Vector leftScoresVector = Q6_V_valign_VVR(centerScoresVector, Q6_V_vzero(), VLEN-2);
        HVX_Vector rightScoresVector = Q6_V_valign_VVI(Q6_V_vzero(), centerScoresVector, 2);

        HVX_Vector leftPositionsVectorPlusOne = leftPositionsVector + Q6_Vh_vsplat_R(1);
        HVX_Vector rightPositionsVectorMinusOne = rightPositionVector - Q6_Vh_vsplat_R(1);

        HVX_Vector Vnf1 = Q6_Q_vcmp_eq_VhVh(centerPositionsVector, leftPositionsVectorPlusOne);
        HVX_Vector Vnf2 = Q6_Q_vcmp_eq_VhVh(centerPositionsVector, rightPositionsVectorMinusOne);

        HVX_Vector Vsf1 = Q6_Q_vcmp_gt_VhVh(leftScoresVector, centerScoresVector);
        HVX_Vector Vsf2 = Q6_Q_vcmp_gt_VhVh(rightScoresVector, centerScoresVector);

        HVX_Vector Vf1 = Q6_Q_and_QQ(Vnf1, Vsf1);
        HVX_Vector Vf2 = Q6_Q_and_QQ(Vnf2, Vsf2);

        HVX_Vector Vf = Q6_Q_not_Q(Q6_Q_or_QQ(Vf1, Vf2));

        HVX_Vector final_score = Q6_Vuh_vmin_VuhVuh(centerScoresVector, Vf);
        
        *scoresCursor++ = final_score;
    }

    return 0;
}

int sort_and_clean_features(uint32_t imageHeight, uint16_t** subImageFeatures, uint16_t** subImageFeaturesScores, uint32_t subImageCount, uint32_t* featuresPerSubImage, uint32_t maxFeaturesPerSubImage, uint32_t minimumScore, bool verticalToHorizontalCoordinateReordering){
    //sort_and_clean_features
    for(int i = 0; i < subImageCount*subImageCount; i++){
        uint32_t currentFeatures = featuresPerSubImage[i];
        uint32_t finalFeatures = maxFeaturesPerSubImage < currentFeatures ? maxFeaturesPerSubImage : currentFeatures;
        finalFeatures = sort_and_clean_features_stride(currentFeatures, imageHeight, subImageFeatures[i],  subImageFeaturesScores[i], finalFeatures, minimumScore, verticalToHorizontalCoordinateReordering);
        featuresPerSubImage[i] = finalFeatures;
    }

    return 0;
}

//Please make maxFeatures multiple of 64
int sort_and_clean_features_stride(uint32_t featuresCount, uint32_t imageHeight, uint16_t* subImageFeatures, uint16_t* subImageFeaturesScores, uint32_t maxFeaturesPerSubImage, uint32_t minimumScore, bool verticalToHorizontalCoordinateReordering){
    if(featuresCount == 0){
        //FARF(LOW, "Skipping empty subimage (sort_and_clean_features_stride)\n");
        return 0;
    }
    
    uint32_t finalFeatures = maxFeaturesPerSubImage;

    uint32_t totalIterationsCount = (featuresCount-1)/(VLEN/2) + 1; 

    HVX_Vector* fillWithZeroVecPtr = (HVX_Vector*)(subImageFeaturesScores + (totalIterationsCount-1)*(VLEN/2));

    HVX_VectorPred maskFilter = Q6_Q_vsetq_R((totalIterationsCount*(VLEN/2) - featuresCount)*2);

    HVX_Vector lastVector = *fillWithZeroVecPtr;
    *fillWithZeroVecPtr = Q6_V_vmux_QVV(maskFilter, lastVector,  Q6_V_vzero());

    
    bitonic_sort_uint16_decreasing_hvx(subImageFeatures, subImageFeaturesScores, (featuresCount-1)/(VLEN/2)+1);

    if(verticalToHorizontalCoordinateReordering){
        HVX_Vector* scoresCursor = (HVX_Vector*)(subImageFeaturesScores);
        HVX_Vector* indicesCursor = (HVX_Vector*)(subImageFeatures);

        for(int i = 0; i < totalIterationsCount; i++){
            //Put back indices to horizontal (for easier gathering later)
            /*
            * FOR EACH NUMBER
            * y = j*512 + i (0 <= i < 512)
            * i = y & 111111111b = y & 0x1FF
            * j = y >> 9
            * i*128 = i << 7
            * y = i*128 + j
            */

            HVX_Vector y_vector = *indicesCursor;

            HVX_Vector j_vector = Q6_Vh_vlsr_VhVh(y_vector, Q6_Vh_vsplat_R(8));
            HVX_Vector i_vector = Q6_V_vand_VV(y_vector, Q6_Vh_vsplat_R(0xFF));


            HVX_Vector i_256_vector = Q6_Vh_vasl_VhR(i_vector, 8);
            HVX_Vector x_vector = i_256_vector + j_vector;

            *indicesCursor++ = x_vector;
        }
    }

    return maxFeaturesPerSubImage > featuresCount ? featuresCount : maxFeaturesPerSubImage;
}


int clean_features(uint32_t imageHeight, uint16_t** subImageFeatures, uint16_t** subImageFeaturesScores, uint32_t subImageCount, uint32_t* featuresPerSubImage, uint32_t maxFeaturesPerSubImage, uint32_t minimumScore){
    for(int i = 0; i < subImageCount*subImageCount; i++){
        uint32_t currentFeatures = featuresPerSubImage[i];
        uint32_t finalFeatures = maxFeaturesPerSubImage < currentFeatures ? maxFeaturesPerSubImage : currentFeatures;
        finalFeatures = clean_features_stride(currentFeatures, imageHeight, subImageFeatures[i],  subImageFeaturesScores[i], maxFeaturesPerSubImage, minimumScore);
        featuresPerSubImage[i] = finalFeatures;
    }

    return 0;
}

int clean_features_stride(uint32_t featuresCount, uint32_t imageHeight, uint16_t* subImageFeatures, uint16_t* subImageFeaturesScores, uint32_t maxFeaturesPerSubImage, uint32_t minimumScore){
    if(featuresCount == 0){
        return 0;
    }

    HVX_Vector* scoresCursor = (HVX_Vector*)(subImageFeaturesScores);
    HVX_Vector* indicesCursor = (HVX_Vector*)(subImageFeatures);

    int iterationsCount = (featuresCount-1)/(VLEN/2) + 1;
    int finalFeatures = 0;

    HVX_Vector thresholdVector = Q6_Vh_vsplat_R(minimumScore);

    for(int i = 0; i < iterationsCount; i++){
        //FILTER THRESHOLD
        HVX_Vector centerScore = (*scoresCursor++);
        HVX_Vector mask = Q6_Q_vcmp_gt_VhVh(centerScore, thresholdVector);
        HVX_Vector maskUnit = Q6_Q_and_QQ(mask, Q6_Vh_vsplat_R(1));

        HVX_Vector centerIndices = (*indicesCursor++);

        union {
            HVX_Vector vqbitsum;
            uint16_t sqbitsum_u8[VLEN/2];
        } uu;
        uu.vqbitsum = Q6_Vh_prefixsum_Q(maskUnit);


        if(finalFeatures + uu.sqbitsum_u8[VLEN/2-1] >= maxFeaturesPerSubImage){
            finalFeatures = maxFeaturesPerSubImage; 
            break;
        }
        
        HVX_Vector offsetVec = Q6_Vh_vasl_VhR(Q6_Vuh_vsub_VuhVuh_sat(uu.vqbitsum, Q6_Vh_vsplat_R(1)), 1);
        Q6_vscatter_QRMVhV(mask, (uint32_t)subImageFeaturesScores+finalFeatures*sizeof(uint16_t), (maxFeaturesPerSubImage*sizeof(uint16_t))-1, offsetVec, centerScore); //Check if needs shift
    
        Q6_vscatter_QRMVhV(mask, (uint32_t)subImageFeatures+finalFeatures*sizeof(uint16_t), (maxFeaturesPerSubImage*sizeof(uint16_t))-1, offsetVec, centerIndices); //Check if needs shift
    
        finalFeatures += uu.sqbitsum_u8[VLEN/2-1]; 

    }

    //CLEAR THE REST OF THE LAST VECTOR
    int finalVectorIndex = finalFeatures/(VLEN/2);
    int toKeep = finalFeatures%(VLEN/2);
    HVX_VectorPred mask = Q6_Q_vsetq_R(2*toKeep);
    scoresCursor = (HVX_Vector*)(subImageFeaturesScores + finalFeatures);
    indicesCursor = (HVX_Vector*)(subImageFeatures + finalFeatures);
    *scoresCursor = Q6_V_vmux_QVV(mask, *scoresCursor, Q6_V_vzero());
    *indicesCursor = Q6_V_vmux_QVV(mask, *indicesCursor, Q6_Vh_vsplat_R(0xFFFF));


    return finalFeatures;
}