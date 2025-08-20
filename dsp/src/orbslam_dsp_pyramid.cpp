/**
 * @file orbslam_dsp_pyramid.cpp
 *
 * @author Gaston Rouquette (Lynx Mixed Reality)
 *
 * @brief Image Pyramid construction algorithm and VTCM custom cache management
**/



#include "orbslam_dsp_pyramid.h"
#include "HAP_farf.h"
#include "HAP_vtcm_mgr.h"
#include "HAP_compute_res.h"
#include "AEEStdErr.h"
#include "orbslam3.h"
#include "hvx_internal.h"

// --- Data Structures ---

/// @brief Union representing a line of precomputed indices for horizontal resampling
union precomputed_line_indices_t {
    HVX_Vector vec[sizeof(uint16_t) * INPUT_WIDTH / VLEN]; ///< Vectorized indices (max memory allocated)
    uint16_t hw[INPUT_WIDTH];                             ///< Hardware indices array
};

/// @brief Union representing a line of precomputed coefficients for horizontal resampling
union precomputed_line_coefs_t {
    HVX_VectorPair vec[INPUT_WIDTH / VLEN]; ///< Vectorized coefficient pairs (max memory allocated)
    uint8_t ub[INPUT_WIDTH * 2];           ///< Byte array for coefficients
};

/// @brief Structure containing indices and coefficients for horizontal resampling between pyramid levels
struct precomputed_indices_and_coefs_t {
    precomputed_line_indices_t indices_left;  ///< Left indices for resampling
    precomputed_line_indices_t indices_right; ///< Right indices for resampling
    precomputed_line_coefs_t coefs;           ///< Coefficients for resampling
};

/// @brief Global array for precomputed indices and coefficients for all pyramid levels
precomputed_indices_and_coefs_t precomputed_indices_and_coefs[LEVELS - 1];

// --- Legacy Pyramid Calculation Functions ---

/// @brief Calculate a Gaussian pyramid using a naive 2x downsampling approach
/// @param inputData Input image data
/// @param pyramid Output pyramid structure
/// @return 0 on success
int calculate_pyramid_2n_dumb(uint8_t* inputData, pyramid_t* pyramid) {
    // Copy input to level 0
    for (int i = 0; i < INPUT_WIDTH * INPUT_HEIGHT; i++) {
        pyramid->outputLevel0[i] = inputData[i];
    }

    // Level 1: 2x downsampling by averaging 2x2 blocks
    for (int i = 0; i < INPUT_HEIGHT / 2; i++) {
        for (int j = 0; j < INPUT_WIDTH / 2; j++) {
            int sum = 0;
            sum += (int)inputData[2 * (i * INPUT_WIDTH + j)];
            sum += (int)inputData[2 * (i * INPUT_WIDTH + j) + 1];
            sum += (int)inputData[2 * (i * INPUT_WIDTH + j) + INPUT_WIDTH];
            sum += (int)inputData[2 * (i * INPUT_WIDTH + j) + 1 + INPUT_WIDTH];
            pyramid->outputLevel1[i * LEVEL1_STRIDE + j] = (uint8_t)(sum / 4);
        }
    }

    // Level 2: 4x downsampling by averaging 2x2 blocks from level 1
    for (int i = 0; i < INPUT_HEIGHT / 4; i++) {
        for (int j = 0; j < INPUT_WIDTH / 4; j++) {
            int sum = 0;
            sum += (int)pyramid->outputLevel1[2 * (i * LEVEL1_STRIDE + j)];
            sum += (int)pyramid->outputLevel1[2 * (i * LEVEL1_STRIDE + j) + 1];
            sum += (int)pyramid->outputLevel1[2 * (i * LEVEL1_STRIDE + j) + LEVEL1_STRIDE];
            sum += (int)pyramid->outputLevel1[2 * (i * LEVEL1_STRIDE + j) + 1 + LEVEL1_STRIDE];
            pyramid->outputLevel2[i * LEVEL2_STRIDE + j] = (uint8_t)(sum / 4);
        }
    }

    // Level 3: 8x downsampling by averaging 2x2 blocks from level 2
    for (int i = 0; i < INPUT_HEIGHT / 8; i++) {
        for (int j = 0; j < INPUT_WIDTH / 8; j++) {
            int sum = 0;
            sum += (int)pyramid->outputLevel2[2 * (i * LEVEL2_STRIDE + j)];
            sum += (int)pyramid->outputLevel2[2 * (i * LEVEL2_STRIDE + j) + 1];
            sum += (int)pyramid->outputLevel2[2 * (i * LEVEL2_STRIDE + j) + LEVEL2_STRIDE];
            sum += (int)pyramid->outputLevel2[2 * (i * LEVEL2_STRIDE + j) + 1 + LEVEL2_STRIDE];
            pyramid->outputLevel3[i * LEVEL3_STRIDE + j] = (uint8_t)(sum / 4);
        }
    }

    return 0;
}

/// @brief Calculate a single pyramid level using 2x downsampling with HVX
/// @param inputData Input image data
/// @param output_data Output image data
/// @param input_width Input image width
/// @param input_stride Input image stride
/// @param input_height Input image height
/// @param output_stride Output image stride
/// @return 0 on success
int calculate_pyramid_image_2n_hvx(uint8_t* inputData, uint8_t* output_data, int input_width, int input_stride, int input_height, int output_stride) {
    // Initialize cursors
    HVX_Vector* inputCursorTopLeft = (HVX_Vector*)(inputData);
    HVX_Vector* inputCursorTopRight = (HVX_Vector*)(inputData + VLEN);
    HVX_Vector* inputCursorBottomLeft = (HVX_Vector*)(inputData + input_stride);
    HVX_Vector* inputCursorBottomRight = (HVX_Vector*)(inputData + input_stride + VLEN);
    HVX_Vector* outputCursor = (HVX_Vector*)(output_data);

    // Prefetch initial input data
    const uint8_t* l2fetchAddr = inputData;
    l2fetch(l2fetchAddr, input_stride, input_width, 2, 1);

    uint32_t deltaLineVec = 2 * ((input_stride / VLEN) / 2);

    // Process each output line
    for (int i = 0; i < input_height / 2; i++) {
        // Update L2 cache for next iteration
        if (i < input_height / 2 - 1) {
            l2fetchAddr += input_stride * 2;
            l2fetch(l2fetchAddr, input_stride, input_stride, 2, 0);
        }

        outputCursor = (HVX_Vector*)((uint8_t*)(output_data + i * output_stride));

        // Process each vector in the line
        for (int j = 0; j < input_width / 2; j += VLEN) {
            HVX_Vector topLeft = *inputCursorTopLeft;
            inputCursorTopLeft += 2;

            HVX_Vector bottomLeft = *inputCursorBottomLeft;
            inputCursorBottomLeft += 2;

            // Compute vertical mean
            HVX_Vector leftVector = Q6_Vub_vavg_VubVub(topLeft, bottomLeft);

            HVX_Vector topRight = *inputCursorTopRight;
            inputCursorTopRight += 2;

            HVX_Vector bottomRight = *inputCursorBottomRight;
            inputCursorBottomRight += 2;

            // Compute vertical mean
            HVX_Vector rightVector = Q6_Vub_vavg_VubVub(topRight, bottomRight);

            // Realign and compute horizontal mean
            HVX_VectorPair realignedHorizontalPair = Q6_Wb_vshuffoe_VbVb(rightVector, leftVector);
            *outputCursor++ = Q6_Vb_vdeal_Vb(Q6_Vub_vavg_VubVub(Q6_V_hi_W(realignedHorizontalPair), Q6_V_lo_W(realignedHorizontalPair)));
        }

        // Move cursors to next line
        inputCursorTopLeft += deltaLineVec;
        inputCursorTopRight += deltaLineVec;
        inputCursorBottomLeft += deltaLineVec;
        inputCursorBottomRight += deltaLineVec;
    }

    return 0;
}

/// @brief Copy an image to the output using HVX
/// @param inputData Input image data
/// @param output_data Output image data
/// @param input_width Input image width
/// @param input_stride Input image stride
/// @param input_height Input image height
/// @param output_stride Output image stride
/// @return 0 on success
int calculate_pyramid_image_copy_hvx(const uint8_t* inputData, uint8_t* output_data, int input_width, int input_stride, int input_height, int output_stride) {
    // Initialize cursors
    HVX_Vector* inputCursor = (HVX_Vector*)(inputData);
    HVX_Vector* outputCursor = (HVX_Vector*)(output_data);

    // Prefetch initial input data
    const uint8_t* l2fetchAddr = inputData;
    l2fetch(l2fetchAddr, input_stride, input_width, 2, 1);

    uint32_t deltaLineVec = (input_stride - input_width) / VLEN;

    // Process each line
    for (int i = 0; i < input_height; i++) {
        // Update L2 cache for next iteration
        if (i < input_height - 1) {
            l2fetchAddr += input_stride;
            l2fetch(l2fetchAddr, input_stride, input_stride, 1, 0);
        }

        outputCursor = (HVX_Vector*)((uint8_t*)(output_data + i * output_stride));

        // Copy vectors
        for (int j = 0; j < input_width; j += VLEN) {
            *outputCursor++ = *inputCursor++;
        }

        inputCursor += deltaLineVec;
    }

    return 0;
}

/// @brief Calculate a Gaussian pyramid using 2x downsampling with HVX
/// @param inputData Input image data
/// @param pyramid Output pyramid structure
/// @return 0 on success
int calculate_pyramid_2n_hvx(uint8_t* inputData, pyramid_t* pyramid) {
    // Level 0: Copy input
    calculate_pyramid_image_copy_hvx(inputData, pyramid->outputLevel0, INPUT_WIDTH, INPUT_WIDTH, INPUT_HEIGHT, INPUT_WIDTH);

    // Level 1: 2x downsampling
    calculate_pyramid_image_2n_hvx(inputData, pyramid->outputLevel1, INPUT_WIDTH, INPUT_WIDTH, INPUT_HEIGHT, LEVEL1_STRIDE);

    // Level 2: 4x downsampling
    calculate_pyramid_image_2n_hvx(pyramid->outputLevel1, pyramid->outputLevel2, INPUT_WIDTH / 2, LEVEL1_STRIDE, INPUT_HEIGHT / 2, LEVEL2_STRIDE);

    // Level 3: 8x downsampling
    calculate_pyramid_image_2n_hvx(pyramid->outputLevel2, pyramid->outputLevel3, INPUT_WIDTH / 4, LEVEL2_STRIDE, INPUT_HEIGHT / 4, LEVEL3_STRIDE);

    return 0;
}

/// @brief Calculate stereo Gaussian pyramids using 2x downsampling with HVX
/// @param inputData Input stereo image data
/// @param pyramid1 Output pyramid for left image
/// @param pyramid2 Output pyramid for right image
/// @return 0 on success
int calculate_pyramid_2n_hvx_stereo(const uint8_t* inputData, pyramid_t* pyramid1, pyramid_t* pyramid2) {
    // Level 0: Copy left and right images
    calculate_pyramid_image_copy_hvx(inputData, pyramid1->outputLevel0, INPUT_WIDTH, INPUT_WIDTH * 2, INPUT_HEIGHT, INPUT_WIDTH);
    calculate_pyramid_image_copy_hvx(inputData + INPUT_WIDTH * sizeof(uint8_t), pyramid2->outputLevel0, INPUT_WIDTH, INPUT_WIDTH * 2, INPUT_HEIGHT, INPUT_WIDTH);

    // Level 1: 2x downsampling
    calculate_pyramid_image_2n_hvx(pyramid1->outputLevel0, pyramid1->outputLevel1, INPUT_WIDTH, INPUT_WIDTH, INPUT_HEIGHT, LEVEL1_STRIDE);
    calculate_pyramid_image_2n_hvx(pyramid2->outputLevel0, pyramid2->outputLevel1, INPUT_WIDTH, INPUT_WIDTH, INPUT_HEIGHT, LEVEL1_STRIDE);

    // Level 2: 4x downsampling
    calculate_pyramid_image_2n_hvx(pyramid1->outputLevel1, pyramid1->outputLevel2, INPUT_WIDTH / 2, LEVEL1_STRIDE, INPUT_HEIGHT / 2, LEVEL2_STRIDE);
    calculate_pyramid_image_2n_hvx(pyramid2->outputLevel1, pyramid2->outputLevel2, INPUT_WIDTH / 2, LEVEL1_STRIDE, INPUT_HEIGHT / 2, LEVEL2_STRIDE);

    // Level 3: 8x downsampling
    calculate_pyramid_image_2n_hvx(pyramid1->outputLevel2, pyramid1->outputLevel3, INPUT_WIDTH / 4, LEVEL2_STRIDE, INPUT_HEIGHT / 4, LEVEL3_STRIDE);
    calculate_pyramid_image_2n_hvx(pyramid2->outputLevel2, pyramid2->outputLevel3, INPUT_WIDTH / 4, LEVEL2_STRIDE, INPUT_HEIGHT / 4, LEVEL3_STRIDE);

    return 0;
}

/// @brief Calculate a Gaussian pyramid for the left stereo image using 2x downsampling with HVX
/// @param inputData Input stereo image data
/// @param pyramid Output pyramid structure
/// @return 0 on success
int calculate_pyramid_2n_hvx_stereo_left(const uint8_t* inputData, pyramid_t* pyramid) {
    // Level 0: Copy left image
    calculate_pyramid_image_copy_hvx(inputData, pyramid->outputLevel0, INPUT_WIDTH, INPUT_WIDTH * 2, INPUT_HEIGHT, INPUT_WIDTH);

    // Level 1: 2x downsampling
    calculate_pyramid_image_2n_hvx(pyramid->outputLevel0, pyramid->outputLevel1, INPUT_WIDTH, INPUT_WIDTH, INPUT_HEIGHT, LEVEL1_STRIDE);

    // Level 2: 4x downsampling
    calculate_pyramid_image_2n_hvx(pyramid->outputLevel1, pyramid->outputLevel2, INPUT_WIDTH / 2, LEVEL1_STRIDE, INPUT_HEIGHT / 2, LEVEL2_STRIDE);

    // Level 3: 8x downsampling
    calculate_pyramid_image_2n_hvx(pyramid->outputLevel2, pyramid->outputLevel3, INPUT_WIDTH / 4, LEVEL2_STRIDE, INPUT_HEIGHT / 4, LEVEL3_STRIDE);

    return 0;
}

/// @brief Calculate a Gaussian pyramid for the right stereo image using 2x downsampling with HVX
/// @param inputData Input stereo image data
/// @param pyramid Output pyramid structure
/// @return 0 on success
int calculate_pyramid_2n_hvx_stereo_right(const uint8_t* inputData, pyramid_t* pyramid) {
    // Level 0: Copy right image
    calculate_pyramid_image_copy_hvx(inputData + INPUT_WIDTH * sizeof(uint8_t), pyramid->outputLevel0, INPUT_WIDTH, INPUT_WIDTH * 2, INPUT_HEIGHT, INPUT_WIDTH);

    // Level 1: 2x downsampling
    calculate_pyramid_image_2n_hvx(pyramid->outputLevel0, pyramid->outputLevel1, INPUT_WIDTH, INPUT_WIDTH, INPUT_HEIGHT, LEVEL1_STRIDE);

    // Level 2: 4x downsampling
    calculate_pyramid_image_2n_hvx(pyramid->outputLevel1, pyramid->outputLevel2, INPUT_WIDTH / 2, LEVEL1_STRIDE, INPUT_HEIGHT / 2, LEVEL2_STRIDE);

    // Level 3: 8x downsampling
    calculate_pyramid_image_2n_hvx(pyramid->outputLevel2, pyramid->outputLevel3, INPUT_WIDTH / 4, LEVEL2_STRIDE, INPUT_HEIGHT / 4, LEVEL3_STRIDE);

    return 0;
}

// --- VTCM Cache Functions ---

/// @brief Cache a stripe of image data to VTCM without borders
/// @param baseAddress Input image data
/// @param pyramid_cache VTCM cache structure
/// @param height Image height
/// @param input_stride Input image stride
/// @return 0 on success
int cache_stripe_vtcm_2_no_border(uint8_t* baseAddress, pyramid_VTCM_cache_t* pyramid_cache, int height, int input_stride) {
    // Initialize cursors
    HVX_Vector* inputCursor = (HVX_Vector*)(baseAddress);
    HVX_Vector* outputCursor = (HVX_Vector*)(pyramid_cache->cacheData);
    HVX_Vector* leftInputCursor = inputCursor - 1;
    HVX_Vector* rightInputCursor = inputCursor + 1;

    uint32_t deltaLineVec = input_stride / VLEN;
    HVX_VectorPred selectMask = Q6_Q_vsetq_R(VLEN / 2);
    HVX_Vector leftInput = Q6_V_vzero();
    HVX_Vector rightInput = Q6_V_vzero();

    // Process each line
    for (int i = 0; i < height; i++) {
        // Prefetch next line
        l2fetch(inputCursor + deltaLineVec * 10, input_stride, input_stride, 1, 0);

        // Cache left border
        leftInput = *leftInputCursor;
        leftInputCursor += deltaLineVec;
        *outputCursor++ = Q6_V_vmux_QVV(selectMask, rightInput, leftInput);

        // Cache right border
        rightInput = *rightInputCursor;
        rightInputCursor += deltaLineVec;
        *outputCursor++ = *inputCursor;
        inputCursor += deltaLineVec;
    }

    // Finalize with zeroed border
    leftInput = Q6_V_vzero();
    *outputCursor++ = Q6_V_vmux_QVV(selectMask, rightInput, leftInput);

    return 0;
}

/// @brief Cache a stripe of image data to VTCM with left border
/// @param baseAddress Input image data
/// @param pyramid_cache VTCM cache structure
/// @param height Image height
/// @param input_stride Input image stride
/// @return 0 on success
int cache_stripe_vtcm_2_border_left(uint8_t* baseAddress, pyramid_VTCM_cache_t* pyramid_cache, int height, int input_stride) {
    // Initialize cursors
    HVX_Vector* inputCursor = (HVX_Vector*)(baseAddress);
    HVX_Vector* outputCursor = (HVX_Vector*)(pyramid_cache->cacheData);
    HVX_Vector* rightInputCursor = inputCursor + 1;

    uint32_t deltaLineVec = input_stride / VLEN;
    HVX_VectorPred selectMask = Q6_Q_vsetq_R(VLEN / 2);
    HVX_Vector rightInput = Q6_V_vzero();

    // Process each line
    for (int i = 0; i < height; i++) {
        // Prefetch next line
        l2fetch(inputCursor + deltaLineVec * 10, input_stride, input_stride, 1, 0);

        // Cache left border
        *outputCursor++ = Q6_V_vmux_QVV(selectMask, rightInput, Q6_V_vzero());

        // Cache right border
        rightInput = *rightInputCursor;
        rightInputCursor += deltaLineVec;
        *outputCursor++ = *inputCursor;
        inputCursor += deltaLineVec;
    }

    // Finalize with zeroed border
    *outputCursor++ = Q6_V_vmux_QVV(selectMask, rightInput, Q6_V_vzero());

    return 0;
}

/// @brief Cache a stripe of image data to VTCM with right border
/// @param baseAddress Input image data
/// @param pyramid_cache VTCM cache structure
/// @param height Image height
/// @param input_stride Input image stride
/// @return 0 on success
int cache_stripe_vtcm_2_border_right(uint8_t* baseAddress, pyramid_VTCM_cache_t* pyramid_cache, int height, int input_stride) {
    // Initialize cursors
    HVX_Vector* inputCursor = (HVX_Vector*)(baseAddress);
    HVX_Vector* outputCursor = (HVX_Vector*)(pyramid_cache->cacheData);
    HVX_Vector* leftInputCursor = inputCursor - 1;

    uint32_t deltaLineVec = input_stride / VLEN;
    HVX_VectorPred selectMask = Q6_Q_vsetq_R(VLEN / 2);
    HVX_Vector leftInput = Q6_V_vzero();

    // Process each line
    for (int i = 0; i < height; i++) {
        // Prefetch next line
        l2fetch(inputCursor + deltaLineVec * 10, input_stride, input_stride, 1, 0);

        // Cache left border
        leftInput = *leftInputCursor;
        leftInputCursor += deltaLineVec;
        *outputCursor++ = Q6_V_vmux_QVV(selectMask, Q6_V_vzero(), leftInput);

        // Cache center
        *outputCursor++ = *inputCursor;
        inputCursor += deltaLineVec;
    }

    // Finalize with zeroed border
    *outputCursor = Q6_V_vzero();

    return 0;
}

/// @brief Cache a stripe of image data to VTCM with both borders
/// @param baseAddress Input image data
/// @param pyramid_cache VTCM cache structure
/// @param height Image height
/// @param input_stride Input image stride
/// @return 0 on success
int cache_stripe_vtcm_2_both_borders(uint8_t* baseAddress, pyramid_VTCM_cache_t* pyramid_cache, int height, int input_stride) {
    // Initialize cursors
    HVX_Vector* inputCursor = (HVX_Vector*)(baseAddress);
    HVX_Vector* outputCursor = (HVX_Vector*)(pyramid_cache->cacheData);

    uint32_t deltaLineVec = input_stride / VLEN;

    // Process each line
    for (int i = 0; i < height; i++) {
        // Prefetch next line
        l2fetch(inputCursor + deltaLineVec * 10, input_stride, input_stride, 1, 0);

        // Cache zeroed border
        *outputCursor++ = Q6_V_vzero();

        // Cache center
        *outputCursor++ = *inputCursor;
        inputCursor += deltaLineVec;
    }

    // Finalize with zeroed border
    *outputCursor = Q6_V_vzero();

    return 0;
}

/// @brief Calculate a single pyramid level using bilinear interpolation with HVX
/// @param inputData Input image data
/// @param output_data Output image data
/// @param input_width Input image width
/// @param input_height Input image height
/// @param input_stride Input image stride
/// @param output_width Output image width
/// @param output_height Output image height
/// @param output_stride Output image stride
/// @param vtcmCache VTCM cache structure
/// @param levelIndex Pyramid level index
/// @return 0 on success
int calculate_pyramid_image_bilinear_hvx(uint8_t* inputData, uint8_t* output_data, int input_width, int input_height, int input_stride, 
                                         int output_width, int output_height, int output_stride, pyramid_VTCM_cache_t* vtcmCache, int levelIndex) {
    // Initialize cursors
    HVX_Vector* inputCursorTop;
    HVX_Vector* inputCursorBottom;
    HVX_Vector* outputCursor;

    uint32_t deltaLineVec = input_stride / VLEN;

    // Compute vertical reduction factors
    float verticalReductionFactor = (float)input_height / (float)output_height;
    float horizontalReductionFactor = (float)input_width / (float)output_width;
    float verticalPosition;
    int verticalPositionDown;
    float topFactor;
    float bottomFactor;

    HVX_VectorPair inputPair;
    HVX_VectorPair result;
    HVX_VectorPair coefPair;
    HVX_Vector result_hi;
    HVX_Vector result_lo;
    HVX_Vector result_hi_shifted;
    HVX_Vector result_lo_shifted;
    HVX_Vector values_left;
    HVX_Vector values_right;

    // Process each output line
    for (int i = 0; i < output_height; i++) {
        outputCursor = (HVX_Vector*)((uint8_t*)(output_data + i * output_stride));
        verticalPosition = i * verticalReductionFactor;
        verticalPositionDown = (int)verticalPosition;

        inputCursorTop = (HVX_Vector*)(inputData + verticalPositionDown * input_stride);
        inputCursorBottom = (HVX_Vector*)(inputData + (verticalPositionDown + 1) * input_stride);
        topFactor = verticalPosition - verticalPositionDown;
        bottomFactor = 1.0f - topFactor;

        // Prefetch input data
        l2fetch(inputCursorTop, input_stride, input_stride, 10, 0);

        // Vertical reduction
        for (int j = 0; j < (input_width - 1) / VLEN + 1; j++) {
            inputPair = Q6_W_vcombine_VV(*inputCursorTop++, *inputCursorBottom++);
            result = Q6_Wh_vmpa_WubRub(inputPair, (0x10001 * ((uint32_t)(255.0f * topFactor)) + 0x1000100 * ((uint32_t)(255.0f * bottomFactor))));

            result_hi = Q6_V_hi_W(result);
            result_lo = Q6_V_lo_W(result);

            result_hi_shifted = Q6_Vh_vasr_VhR(result_hi, 8);
            result_lo_shifted = Q6_Vh_vasr_VhR(result_lo, 8);

            vtcmCache->bilinearReductionVectors[j] = Q6_Vb_vshuffe_VbVb(result_hi_shifted, result_lo_shifted);
        }

        // Horizontal reduction
        for (int j = 0; j < (output_width - 1) / VLEN + 1; j++) {
            Q6_vgather_ARMVh(&vtcmCache->gatheredVector1, (uint32_t)vtcmCache->bilinearReductionVectors, input_width - 1, precomputed_indices_and_coefs[levelIndex].indices_left.vec[2 * j + 1]);
            Q6_vgather_ARMVh(&vtcmCache->gatheredVector2, (uint32_t)vtcmCache->bilinearReductionVectors, input_width - 1, precomputed_indices_and_coefs[levelIndex].indices_left.vec[2 * j]);
            values_left = Q6_Vb_vshuffe_VbVb(vtcmCache->gatheredVector1, vtcmCache->gatheredVector2);

            Q6_vgather_ARMVh(&vtcmCache->gatheredVector1, (uint32_t)vtcmCache->bilinearReductionVectors, input_width - 1, precomputed_indices_and_coefs[levelIndex].indices_right.vec[2 * j + 1]);
            Q6_vgather_ARMVh(&vtcmCache->gatheredVector2, (uint32_t)vtcmCache->bilinearReductionVectors, input_width - 1, precomputed_indices_and_coefs[levelIndex].indices_right.vec[2 * j]);
            values_right = Q6_Vb_vshuffe_VbVb(vtcmCache->gatheredVector1, vtcmCache->gatheredVector2);

            inputPair = Q6_W_vcombine_VV(values_left, values_right);
            coefPair = precomputed_indices_and_coefs[levelIndex].coefs.vec[j];
            result = Q6_Wh_vmpa_WubWub(inputPair, coefPair);

            result_hi_shifted = Q6_Vh_vasr_VhR(Q6_V_hi_W(result), 8);
            result_lo_shifted = Q6_Vh_vasr_VhR(Q6_V_lo_W(result), 8);

            *outputCursor++ = Q6_Vb_vshuffe_VbVb(result_hi_shifted, result_lo_shifted);
        }
    }

    return 0;
}

/// @brief Calculate stereo Gaussian pyramids using bilinear interpolation with HVX
/// @param inputData Input stereo image data
/// @param pyramid1 Output pyramid for left image
/// @param pyramid2 Output pyramid for right image
/// @param vtcmCache VTCM cache for left image
/// @param vtcmCache2 VTCM cache for right image
/// @return 0 on success
int calculate_pyramid_bilinear_hvx_stereo(const uint8_t* inputData, pyramid_t* pyramid1, pyramid_t* pyramid2, pyramid_VTCM_cache_t* vtcmCache, pyramid_VTCM_cache_t* vtcmCache2) {
    calculate_pyramid_bilinear_hvx_stereo_left(inputData, pyramid1, vtcmCache);
    calculate_pyramid_bilinear_hvx_stereo_right(inputData, pyramid2, vtcmCache2);
    return 0;
}

/// @brief Calculate a Gaussian pyramid for the left stereo image using bilinear interpolation with HVX
/// @param inputData Input stereo image data
/// @param pyramid Output pyramid structure
/// @param vtcmCache VTCM cache structure
/// @return 0 on success
int calculate_pyramid_bilinear_hvx_stereo_left(const uint8_t* inputData, pyramid_t* pyramid, pyramid_VTCM_cache_t* vtcmCache) {
    // Level 0: Copy left image
    calculate_pyramid_image_copy_hvx(inputData, pyramid->outputLevel0, INPUT_WIDTH, INPUT_WIDTH * 2, INPUT_HEIGHT, INPUT_WIDTH);

    // Levels 1-7: Bilinear interpolation
    calculate_pyramid_image_bilinear_hvx(pyramid->outputLevel0, pyramid->outputLevel1, INPUT_WIDTH, INPUT_HEIGHT, INPUT_WIDTH, LEVEL1_WIDTH, LEVEL1_HEIGHT, LEVEL1_STRIDE, vtcmCache, 0);
    calculate_pyramid_image_bilinear_hvx(pyramid->outputLevel1, pyramid->outputLevel2, LEVEL1_WIDTH, LEVEL1_HEIGHT, LEVEL1_STRIDE, LEVEL2_WIDTH, LEVEL2_HEIGHT, LEVEL2_STRIDE, vtcmCache, 1);
    calculate_pyramid_image_bilinear_hvx(pyramid->outputLevel2, pyramid->outputLevel3, LEVEL2_WIDTH, LEVEL2_HEIGHT, LEVEL2_STRIDE, LEVEL3_WIDTH, LEVEL3_HEIGHT, LEVEL3_STRIDE, vtcmCache, 2);
    calculate_pyramid_image_bilinear_hvx(pyramid->outputLevel3, pyramid->outputLevel4, LEVEL3_WIDTH, LEVEL3_HEIGHT, LEVEL3_STRIDE, LEVEL4_WIDTH, LEVEL4_HEIGHT, LEVEL4_STRIDE, vtcmCache, 3);
    calculate_pyramid_image_bilinear_hvx(pyramid->outputLevel4, pyramid->outputLevel5, LEVEL4_WIDTH, LEVEL4_HEIGHT, LEVEL4_STRIDE, LEVEL5_WIDTH, LEVEL5_HEIGHT, LEVEL5_STRIDE, vtcmCache, 4);
    calculate_pyramid_image_bilinear_hvx(pyramid->outputLevel5, pyramid->outputLevel6, LEVEL5_WIDTH, LEVEL5_HEIGHT, LEVEL5_STRIDE, LEVEL6_WIDTH, LEVEL6_HEIGHT, LEVEL6_STRIDE, vtcmCache, 5);
    calculate_pyramid_image_bilinear_hvx(pyramid->outputLevel6, pyramid->outputLevel7, LEVEL6_WIDTH, LEVEL6_HEIGHT, LEVEL6_STRIDE, LEVEL7_WIDTH, LEVEL7_HEIGHT, LEVEL7_STRIDE, vtcmCache, 6);

    return 0;
}

/// @brief Calculate a Gaussian pyramid for the right stereo image using bilinear interpolation with HVX
/// @param inputData Input stereo image data
/// @param pyramid Output pyramid structure
/// @param vtcmCache VTCM cache structure
/// @return 0 on success
int calculate_pyramid_bilinear_hvx_stereo_right(const uint8_t* inputData, pyramid_t* pyramid, pyramid_VTCM_cache_t* vtcmCache) {
    // Level 0: Copy right image
    calculate_pyramid_image_copy_hvx(inputData + INPUT_WIDTH * sizeof(uint8_t), pyramid->outputLevel0, INPUT_WIDTH, INPUT_WIDTH * 2, INPUT_HEIGHT, INPUT_WIDTH);

    // Levels 1-7: Bilinear interpolation
    calculate_pyramid_image_bilinear_hvx(pyramid->outputLevel0, pyramid->outputLevel1, INPUT_WIDTH, INPUT_HEIGHT, INPUT_WIDTH, LEVEL1_WIDTH, LEVEL1_HEIGHT, LEVEL1_STRIDE, vtcmCache, 0);
    calculate_pyramid_image_bilinear_hvx(pyramid->outputLevel1, pyramid->outputLevel2, LEVEL1_WIDTH, LEVEL1_HEIGHT, LEVEL1_STRIDE, LEVEL2_WIDTH, LEVEL2_HEIGHT, LEVEL2_STRIDE, vtcmCache, 1);
    calculate_pyramid_image_bilinear_hvx(pyramid->outputLevel2, pyramid->outputLevel3, LEVEL2_WIDTH, LEVEL2_HEIGHT, LEVEL2_STRIDE, LEVEL3_WIDTH, LEVEL3_HEIGHT, LEVEL3_STRIDE, vtcmCache, 2);
    calculate_pyramid_image_bilinear_hvx(pyramid->outputLevel3, pyramid->outputLevel4, LEVEL3_WIDTH, LEVEL3_HEIGHT, LEVEL3_STRIDE, LEVEL4_WIDTH, LEVEL4_HEIGHT, LEVEL4_STRIDE, vtcmCache, 3);
    calculate_pyramid_image_bilinear_hvx(pyramid->outputLevel4, pyramid->outputLevel5, LEVEL4_WIDTH, LEVEL4_HEIGHT, LEVEL4_STRIDE, LEVEL5_WIDTH, LEVEL5_HEIGHT, LEVEL5_STRIDE, vtcmCache, 4);
    calculate_pyramid_image_bilinear_hvx(pyramid->outputLevel5, pyramid->outputLevel6, LEVEL5_WIDTH, LEVEL5_HEIGHT, LEVEL5_STRIDE, LEVEL6_WIDTH, LEVEL6_HEIGHT, LEVEL6_STRIDE, vtcmCache, 5);
    calculate_pyramid_image_bilinear_hvx(pyramid->outputLevel6, pyramid->outputLevel7, LEVEL6_WIDTH, LEVEL6_HEIGHT, LEVEL6_STRIDE, LEVEL7_WIDTH, LEVEL7_HEIGHT, LEVEL7_STRIDE, vtcmCache, 6);

    return 0;
}

/// @brief Precompute indices and coefficients for horizontal bilinear interpolation
/// @return 0 on success
int precompute_horizontal_bilinear_indices_and_coefs() {
    uint32_t widths[8] = {INPUT_WIDTH, LEVEL1_WIDTH, LEVEL2_WIDTH, LEVEL3_WIDTH, LEVEL4_WIDTH, LEVEL5_WIDTH, LEVEL6_WIDTH, LEVEL7_WIDTH};

    // Process each pyramid level
    for (int n = 0; n < LEVELS - 1; n++) {
        precomputed_indices_and_coefs_t* precomputed = &precomputed_indices_and_coefs[n];
        uint32_t previousWidth = widths[n];
        uint32_t nextWidth = widths[n + 1];

        // Compute indices and coefficients for each output pixel
        for (int i = 0; i < nextWidth; i++) {
            float position = (float)i * (float)previousWidth / (float)nextWidth;
            int positionDown = (int)position;
            int positionUp = (int)position + 1;

            int finalIndex = (i / 2) % (VLEN / 2) + (i / (VLEN)) * (VLEN) + (i % 2) * (VLEN / 2);

            precomputed->indices_left.hw[finalIndex] = positionDown;
            precomputed->indices_right.hw[finalIndex] = positionUp;

            float factor = position - positionDown;
            precomputed->coefs.ub[i % VLEN + 2 * (i / VLEN) * VLEN] = (uint8_t)(255.0f * factor);
            precomputed->coefs.ub[i % VLEN + (2 * (i / VLEN) + 1) * VLEN] = (uint8_t)(255.0f * (1.0f - factor));
        }
    }

    return 0;
}