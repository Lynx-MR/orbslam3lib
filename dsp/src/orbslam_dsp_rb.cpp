/**
 * @file orbslam_dsp_rb.cpp
 *
 * @author Gaston Rouquette (Lynx Mixed Reality)
 *
 * @brief ORB Descriptors and Angle calculation for found features using HVX instructions
**/



#include "orbslam_dsp_rb.h"
#include "HAP_farf.h"
#include "HAP_vtcm_mgr.h"
#include "HAP_compute_res.h"
#include "AEEStdErr.h"
#include "orbslam3.h"
#include "hvx_internal.h"
#include <cmath>

// --- ORB Feature Processing ---

/// @brief Compute intensity centroid angles for feature points using SIMD
/// @param inputImage Input image data
/// @param points Array of feature point coordinates
/// @param u_max Precomputed maximum offsets for patch
/// @param encodedAngles Output array for encoded angles
/// @param imageHeight Image height in pixels
/// @param imageWidth Image width in pixels
/// @param imageStride Image stride in bytes
/// @param vtcmImageCache VTCM cache structure
/// @param featuresCount Number of features per sub-image
void ic_angle_simd(uint8_t* inputImage, uint16_t** points, const uint32_t* u_max, uint16_t** encodedAngles, int imageHeight, 
                   uint32_t imageWidth, int imageStride, pyramid_VTCM_cache_t* vtcmImageCache, uint32_t* featuresCount) {
    uint32_t subImageCount = imageStride / VLEN;

    // Handle single sub-image case
    if (subImageCount == 1) {
        cache_stripe_vtcm_2_both_borders(inputImage, vtcmImageCache, imageHeight, imageStride);
        ic_angle_show_stripe(vtcmImageCache, 80, VLEN, 2 * VLEN, points[0], (int16_t*)encodedAngles[0], featuresCount[0], 0, u_max);
        return;
    }

    // Compute bottom offset for last stripe
    int bottomOffset = 1 + ((imageHeight - 1) % 80);

    // Process left border stripes
    cache_stripe_vtcm_2_border_left(inputImage, vtcmImageCache, 80 + HALF_PATCH_SIZE, imageStride);
    ic_angle_show_stripe(vtcmImageCache, 80 + HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[0], (int16_t*)encodedAngles[0], featuresCount[0], 0, u_max);
    for (int j = 1; j < subImageCount - 1; j++) {
        cache_stripe_vtcm_2_border_left(inputImage + (80 * j - HALF_PATCH_SIZE) * imageStride, vtcmImageCache, 80 + 2 * HALF_PATCH_SIZE, imageStride);
        ic_angle_show_stripe(vtcmImageCache, 80 + 2 * HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[j], (int16_t*)encodedAngles[j], featuresCount[j], 0, u_max);
    }
    cache_stripe_vtcm_2_border_left(inputImage + (80 * (subImageCount - 1) - HALF_PATCH_SIZE) * imageStride, vtcmImageCache, bottomOffset + HALF_PATCH_SIZE, imageStride);
    ic_angle_show_stripe(vtcmImageCache, bottomOffset + HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[subImageCount - 1], (int16_t*)encodedAngles[subImageCount - 1], featuresCount[subImageCount - 1], 0, u_max);

    // Process middle stripes
    for (int i = 1; i < subImageCount - 1; i++) {
        cache_stripe_vtcm_2_no_border(inputImage + i * VLEN, vtcmImageCache, 80 + HALF_PATCH_SIZE, imageStride);
        ic_angle_show_stripe(vtcmImageCache, 80 + HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[subImageCount * i], (int16_t*)encodedAngles[subImageCount * i], featuresCount[subImageCount * i], 0, u_max);
        for (int j = 1; j < subImageCount - 1; j++) {
            cache_stripe_vtcm_2_no_border(inputImage + i * VLEN + (80 * j - HALF_PATCH_SIZE) * imageStride, vtcmImageCache, 80 + 2 * HALF_PATCH_SIZE, imageStride);
            ic_angle_show_stripe(vtcmImageCache, 80 + 2 * HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[subImageCount * i + j], (int16_t*)encodedAngles[subImageCount * i + j], featuresCount[subImageCount * i + j], 0, u_max);
        }
        cache_stripe_vtcm_2_no_border(inputImage + i * VLEN + (80 * (subImageCount - 1) - HALF_PATCH_SIZE) * imageStride, vtcmImageCache, bottomOffset + HALF_PATCH_SIZE, imageStride);
        ic_angle_show_stripe(vtcmImageCache, bottomOffset + HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[subImageCount * i + subImageCount - 1], (int16_t*)encodedAngles[subImageCount * i + subImageCount - 1], featuresCount[subImageCount * i + subImageCount - 1], 0, u_max);
    }

    // Process right border stripes
    cache_stripe_vtcm_2_border_right(inputImage + (subImageCount - 1) * VLEN, vtcmImageCache, 80 + HALF_PATCH_SIZE, imageStride);
    ic_angle_show_stripe(vtcmImageCache, 80 + HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[subImageCount * (subImageCount - 1)], (int16_t*)encodedAngles[subImageCount * (subImageCount - 1)], featuresCount[subImageCount * (subImageCount - 1)], 0, u_max);
    for (int j = 1; j < subImageCount - 1; j++) {
        cache_stripe_vtcm_2_border_right(inputImage + (subImageCount - 1) * VLEN + (80 * j - HALF_PATCH_SIZE) * imageStride, vtcmImageCache, 80 + 2 * HALF_PATCH_SIZE, imageStride);
        ic_angle_show_stripe(vtcmImageCache, 80 + 2 * HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[subImageCount * (subImageCount - 1) + j], (int16_t*)encodedAngles[subImageCount * (subImageCount - 1) + j], featuresCount[subImageCount * (subImageCount - 1) + j], 0, u_max);
    }
    cache_stripe_vtcm_2_border_right(inputImage + (subImageCount - 1) * VLEN + (80 * (subImageCount - 1) - HALF_PATCH_SIZE) * imageStride, vtcmImageCache, bottomOffset + HALF_PATCH_SIZE, imageStride);
    ic_angle_show_stripe(vtcmImageCache, bottomOffset + HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[subImageCount * (subImageCount - 1) + subImageCount - 1], (int16_t*)encodedAngles[subImageCount * (subImageCount - 1) + subImageCount - 1], featuresCount[subImageCount * (subImageCount - 1) + subImageCount - 1], 0, u_max);
}

/// @brief Compute intensity centroid angles for a stripe of feature points
/// @param vtcmImageCache VTCM cache structure
/// @param height Stripe height in pixels
/// @param width Stripe width in pixels
/// @param stride Image stride in bytes
/// @param points Feature point coordinates
/// @param encodedAngles Output array for encoded angles
/// @param featureCount Number of features in the stripe
/// @param xoffset Horizontal offset
/// @param u_max Precomputed maximum offsets for patch
void ic_angle_show_stripe(pyramid_VTCM_cache_t* vtcmImageCache, uint32_t height, uint32_t width, int stride, uint16_t* points, 
                          int16_t* encodedAngles, uint32_t featureCount, uint32_t xoffset, const uint32_t* u_max) {
    if (featureCount == 0) {
        return;
    }

    // Compute number of iterations
    uint32_t iterationsCount = (featureCount - 1) / (VLEN / 2) + 1;
    HVX_Vector* inputCursor = (HVX_Vector*)(points);
    HVX_Vector* outputCursor = (HVX_Vector*)(encodedAngles);

    // Process each vector of features
    for (int i = 0; i < iterationsCount; i++) {
        HVX_Vector centerIndices = *inputCursor++;
        union {
            HVX_VectorPair pair;
            int32_t i32[VLEN / 2];
        } m_01, m_10;

        m_01.pair = Q6_Ww_vunpack_Vh(Q6_V_vzero());
        m_10.pair = Q6_Ww_vunpack_Vh(Q6_V_vzero());

        union {
            HVX_Vector vm;
            uint16_t u16[VLEN / 2];
        } are_indices_inside;
        are_indices_inside.vm = Q6_Vb_vsplat_R(0xFF);

        HVX_Vector j_vector = Q6_V_vand_VV(centerIndices, Q6_Vh_vsplat_R(0x7F));

        // Process center line (v=0)
        for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u) {
            HVX_Vector u_vec = Q6_Vh_vsplat_R(u);
            HVX_Vector center_u_addr = Q6_Vh_vadd_VhVh(centerIndices, u_vec);
            Q6_vgather_ARMVh(&vtcmImageCache->gatheredVector1, (uint32_t)vtcmImageCache->cacheData, (VLEN * 2) * (LEVEL7_HEIGHT + HALF_PATCH_SIZE * 2 + 1) + 1, center_u_addr);
            HVX_Vector center_u = Q6_Vb_vshuffe_VbVb(Q6_V_vzero(), vtcmImageCache->gatheredVector1);
            HVX_VectorPair product_u = Q6_Ww_vmpy_VhVh(u_vec, center_u);
            m_10.pair += product_u;
        }

        // Process other lines
        for (int v = 1; v <= HALF_PATCH_SIZE; ++v) {
            int d = u_max[v];
            HVX_Vector v_vec = Q6_Vh_vsplat_R(v);

            for (int u = -d; u <= d; ++u) {
                HVX_Vector u_vec = Q6_Vh_vsplat_R(u);
                HVX_Vector center_u_addr = Q6_Vh_vadd_VhVh(centerIndices, u_vec);
                HVX_Vector center_u_plus_v_step_addr = Q6_Vh_vadd_VhVh(center_u_addr, Q6_Vh_vsplat_R(v * stride));
                Q6_vgather_ARMVh(&vtcmImageCache->gatheredVector1, (uint32_t)vtcmImageCache->cacheData, (VLEN * 2) * (LEVEL7_HEIGHT + HALF_PATCH_SIZE * 2 + 1) + 1, center_u_plus_v_step_addr);
                HVX_Vector center_u_plus_v_step = Q6_Vb_vshuffe_VbVb(Q6_V_vzero(), vtcmImageCache->gatheredVector1);

                HVX_Vector center_u_minus_v_step_addr = Q6_Vh_vsub_VhVh(center_u_addr, Q6_Vh_vsplat_R(v * stride));
                Q6_vgather_ARMVh(&vtcmImageCache->gatheredVector1, (uint32_t)vtcmImageCache->cacheData, (VLEN * 2) * (LEVEL7_HEIGHT + HALF_PATCH_SIZE * 2 + 1) + 1, center_u_minus_v_step_addr);
                HVX_Vector center_u_minus_v_step = Q6_Vb_vshuffe_VbVb(Q6_V_vzero(), vtcmImageCache->gatheredVector1);

                HVX_VectorPair product_u = Q6_Ww_vmpy_VhVh(u_vec, center_u_plus_v_step + center_u_minus_v_step);
                m_10.pair = Q6_Ww_vadd_WwWw(m_10.pair, product_u);

                HVX_VectorPair product_v = Q6_Ww_vmpy_VhVh(v_vec, Q6_Vh_vsub_VhVh(center_u_plus_v_step, center_u_minus_v_step));
                m_01.pair = Q6_Ww_vadd_WwWw(m_01.pair, product_v);
            }
        }

        // Compute angle encoding
        union {
            HVX_Vector vm;
            uint16_t u16[VLEN / 2];
        } results;
        results.vm = Q6_V_vzero();

        // Determine sign of X
        HVX_Vector m_10_lo = Q6_V_lo_W(m_10.pair);
        HVX_Vector m_10_hi = Q6_V_hi_W(m_10.pair);
        HVX_Vector m_10_positive_lo = Q6_Q_vcmp_gt_VwVw(m_10_hi, Q6_V_vzero());
        HVX_Vector m_10_positive_hi = Q6_Q_vcmp_gt_VwVw(m_10_lo, Q6_V_vzero());
        HVX_Vector m_10_pos_16_bits_mask = Q6_Vh_vshuffe_VhVh(m_10_positive_lo, m_10_positive_hi);
        results.vm = Q6_V_vmux_QVV(m_10_pos_16_bits_mask, Q6_Vh_vsplat_R(0x200), Q6_V_vzero());

        // Determine sign of Y
        HVX_Vector m_01_lo = Q6_V_lo_W(m_01.pair);
        HVX_Vector m_01_hi = Q6_V_hi_W(m_01.pair);
        HVX_Vector m_01_positive_lo = Q6_Q_vcmp_gt_VwVw(m_01_hi, Q6_V_vzero());
        HVX_Vector m_01_positive_hi = Q6_Q_vcmp_gt_VwVw(m_01_lo, Q6_V_vzero());
        HVX_Vector m_01_pos_16_bits_mask = Q6_Vh_vshuffe_VhVh(m_01_positive_lo, m_01_positive_hi);
        results.vm |= Q6_V_vmux_QVV(m_01_pos_16_bits_mask, Q6_Vh_vsplat_R(0x100), Q6_V_vzero());

        // Compute magnitude
        m_10_lo = Q6_Vw_vabs_Vw(m_10_lo);
        m_10_hi = Q6_Vw_vabs_Vw(m_10_hi);
        m_01_lo = Q6_Vw_vabs_Vw(m_01_lo);
        m_01_hi = Q6_Vw_vabs_Vw(m_01_hi);

        HVX_Vector m_lo_max = Q6_Vw_vmax_VwVw(m_10_lo, m_01_lo);
        HVX_Vector m_hi_max = Q6_Vw_vmax_VwVw(m_10_hi, m_01_hi);

        HVX_Vector m_lo_log_max_shift = Q6_Vuw_vsub_VuwVuw_sat(Q6_V_vsplat_R(32 - 4), Q6_Vuw_vcl0_Vuw(m_lo_max));
        HVX_Vector m_hi_log_max_shift = Q6_Vuw_vsub_VuwVuw_sat(Q6_V_vsplat_R(32 - 4), Q6_Vuw_vcl0_Vuw(m_hi_max));

        HVX_Vector m_10_lo_shifted = Q6_Vw_vlsr_VwVw(m_10_lo, m_lo_log_max_shift);
        HVX_Vector m_10_hi_shifted = Q6_Vw_vlsr_VwVw(m_10_hi, m_hi_log_max_shift);
        m_10_lo_shifted = Q6_Vw_vasl_VwR(m_10_lo_shifted, 4);
        m_10_hi_shifted = Q6_Vw_vasl_VwR(m_10_hi_shifted, 4);

        HVX_Vector m_01_lo_shifted = Q6_Vw_vlsr_VwVw(m_01_lo, m_lo_log_max_shift);
        HVX_Vector m_01_hi_shifted = Q6_Vw_vlsr_VwVw(m_01_hi, m_hi_log_max_shift);

        HVX_Vector m_10_mag_16_bits = Q6_Vh_vshuffe_VhVh(m_10_hi_shifted, m_10_lo_shifted);
        HVX_Vector m_01_mag_16_bits = Q6_Vh_vshuffe_VhVh(m_01_hi_shifted, m_01_lo_shifted);

        results.vm = Q6_V_vor_VV(results.vm, m_10_mag_16_bits);
        results.vm = Q6_V_vor_VV(results.vm, m_01_mag_16_bits);

        // Lookup cosine and sine values
        HVX_Vector filtered_angles = results.vm;
        Q6_vgather_ARMVh(&vtcmImageCache->gatheredVector2, (uint32_t)vtcmImageCache->cos_lookup_table, 1024, filtered_angles);
        HVX_Vector cos_lookup = vtcmImageCache->gatheredVector2;

        Q6_vgather_ARMVh(&vtcmImageCache->gatheredVector2, (uint32_t)vtcmImageCache->sin_lookup_table, 1024, filtered_angles);
        HVX_Vector sin_lookup = vtcmImageCache->gatheredVector2;

        HVX_Vector cos_sin_vec = Q6_Vb_vshuffe_VbVb(sin_lookup, cos_lookup);
        *outputCursor++ = cos_sin_vec;
    }
}

/// @brief Compute ORB descriptors for feature points
/// @param inputImage Input image data
/// @param points Array of feature point coordinates
/// @param orbDescriptors Output ORB descriptors
/// @param encodedAngles Encoded angle data
/// @param imageHeight Image height in pixels
/// @param imageWidth Image width in pixels
/// @param imageStride Image stride in bytes
/// @param vtcmImageCache VTCM cache structure
/// @param featuresCount Number of features per sub-image
void calculate_orb_descriptors(uint8_t* inputImage, uint16_t** points, orb_descriptors_vecs_t** orbDescriptors, uint16_t** encodedAngles, 
                              int imageHeight, uint32_t imageWidth, int imageStride, pyramid_VTCM_cache_t* vtcmImageCache, uint32_t* featuresCount) {
    uint32_t subImageCount = imageStride / VLEN;

    // Handle single sub-image case
    if (subImageCount == 1) {
        cache_stripe_vtcm_2_both_borders(inputImage, vtcmImageCache, imageHeight, imageStride);
        calculate_orb_descriptors_stripe(vtcmImageCache, 80, VLEN, 2 * VLEN, points[0], (int16_t*)encodedAngles[0], featuresCount[0], 0, orbDescriptors[0]);
        return;
    }

    // Compute bottom offset for last stripe
    int bottomOffset = 1 + ((imageHeight - 1) % 80);

    // Process left border stripes
    cache_stripe_vtcm_2_border_left(inputImage, vtcmImageCache, 80 + HALF_PATCH_SIZE, imageStride);
    calculate_orb_descriptors_stripe(vtcmImageCache, 80 + HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[0], (int16_t*)encodedAngles[0], featuresCount[0], 0, orbDescriptors[0]);
    for (int j = 1; j < subImageCount - 1; j++) {
        cache_stripe_vtcm_2_border_left(inputImage + (80 * j - HALF_PATCH_SIZE) * imageStride, vtcmImageCache, 80 + 2 * HALF_PATCH_SIZE, imageStride);
        calculate_orb_descriptors_stripe(vtcmImageCache, 80 + 2 * HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[j], (int16_t*)encodedAngles[j], featuresCount[j], 0, orbDescriptors[j]);
    }
    cache_stripe_vtcm_2_border_left(inputImage + (80 * (subImageCount - 1) - HALF_PATCH_SIZE) * imageStride, vtcmImageCache, bottomOffset + HALF_PATCH_SIZE, imageStride);
    calculate_orb_descriptors_stripe(vtcmImageCache, bottomOffset + HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[subImageCount - 1], (int16_t*)encodedAngles[subImageCount - 1], featuresCount[subImageCount - 1], 0, orbDescriptors[subImageCount - 1]);

    // Process middle stripes
    for (int i = 1; i < subImageCount - 1; i++) {
        cache_stripe_vtcm_2_no_border(inputImage + i * VLEN, vtcmImageCache, 80 + HALF_PATCH_SIZE, imageStride);
        calculate_orb_descriptors_stripe(vtcmImageCache, 80 + HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[subImageCount * i], (int16_t*)encodedAngles[subImageCount * i], featuresCount[subImageCount * i], 0, orbDescriptors[subImageCount * i]);
        for (int j = 1; j < subImageCount - 1; j++) {
            cache_stripe_vtcm_2_no_border(inputImage + i * VLEN + (80 * j - HALF_PATCH_SIZE) * imageStride, vtcmImageCache, 80 + 2 * HALF_PATCH_SIZE, imageStride);
            calculate_orb_descriptors_stripe(vtcmImageCache, 80 + 2 * HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[subImageCount * i + j], (int16_t*)encodedAngles[subImageCount * i + j], featuresCount[subImageCount * i + j], 0, orbDescriptors[subImageCount * i + j]);
        }
        cache_stripe_vtcm_2_no_border(inputImage + i * VLEN + (80 * (subImageCount - 1) - HALF_PATCH_SIZE) * imageStride, vtcmImageCache, bottomOffset + HALF_PATCH_SIZE, imageStride);
        calculate_orb_descriptors_stripe(vtcmImageCache, bottomOffset + HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[subImageCount * i + subImageCount - 1], (int16_t*)encodedAngles[subImageCount * i + subImageCount - 1], featuresCount[subImageCount * i + subImageCount - 1], 0, orbDescriptors[subImageCount * i + subImageCount - 1]);
    }

    // Process right border stripes
    cache_stripe_vtcm_2_border_right(inputImage + (subImageCount - 1) * VLEN, vtcmImageCache, 80 + HALF_PATCH_SIZE, imageStride);
    calculate_orb_descriptors_stripe(vtcmImageCache, 80 + HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[subImageCount * (subImageCount - 1)], (int16_t*)encodedAngles[subImageCount * (subImageCount - 1)], featuresCount[subImageCount * (subImageCount - 1)], 0, orbDescriptors[subImageCount * (subImageCount - 1)]);
    for (int j = 1; j < subImageCount - 1; j++) {
        cache_stripe_vtcm_2_border_right(inputImage + (subImageCount - 1) * VLEN + (80 * j - HALF_PATCH_SIZE) * imageStride, vtcmImageCache, 80 + 2 * HALF_PATCH_SIZE, imageStride);
        calculate_orb_descriptors_stripe(vtcmImageCache, 80 + 2 * HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[subImageCount * (subImageCount - 1) + j], (int16_t*)encodedAngles[subImageCount * (subImageCount - 1) + j], featuresCount[subImageCount * (subImageCount - 1) + j], 0, orbDescriptors[subImageCount * (subImageCount - 1) + j]);
    }
    cache_stripe_vtcm_2_border_right(inputImage + (subImageCount - 1) * VLEN + (80 * (subImageCount - 1) - HALF_PATCH_SIZE) * imageStride, vtcmImageCache, bottomOffset + HALF_PATCH_SIZE, imageStride);
    calculate_orb_descriptors_stripe(vtcmImageCache, bottomOffset + HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[subImageCount * (subImageCount - 1) + subImageCount - 1], (int16_t*)encodedAngles[subImageCount * (subImageCount - 1) + subImageCount - 1], featuresCount[subImageCount * (subImageCount - 1) + subImageCount - 1], 0, orbDescriptors[subImageCount * (subImageCount - 1) + subImageCount - 1]);
}

/// @brief Compute ORB descriptors for feature points with optimized 16-bit processing
/// @param pattern_vecs Precomputed pattern vectors
/// @param inputImage Input image data
/// @param points Array of feature point coordinates
/// @param orbDescriptors Output ORB descriptors
/// @param encodedAngles Encoded angle data
/// @param imageHeight Image height in pixels
/// @param imageWidth Image width in pixels
/// @param imageStride Image stride in bytes
/// @param vtcmImageCache VTCM cache structure
/// @param featuresCount Number of features per sub-image
void calculate_orb_descriptors_optimized16(orb_pattern_vecs_cache_t& pattern_vecs, uint8_t* inputImage, uint16_t** points, orb_descriptors_vecs_t** orbDescriptors, 
                                         uint16_t** encodedAngles, int imageHeight, uint32_t imageWidth, int imageStride, pyramid_VTCM_cache_t* vtcmImageCache, 
                                         uint32_t* featuresCount) {
    uint32_t subImageCount = imageStride / VLEN;

    // Handle single sub-image case
    if (subImageCount == 1) {
        cache_stripe_vtcm_2_both_borders(inputImage, vtcmImageCache, imageHeight, imageStride);
        calculate_orb_descriptors_stripe_optimized16(pattern_vecs, vtcmImageCache, 80, VLEN, 2 * VLEN, points[0], (int16_t*)encodedAngles[0], featuresCount[0], 0, orbDescriptors[0]);
        return;
    }

    // Compute bottom offset for last stripe
    int bottomOffset = 1 + ((imageHeight - 1) % 80);

    // Process left border stripes
    cache_stripe_vtcm_2_border_left(inputImage, vtcmImageCache, 80 + HALF_PATCH_SIZE, imageStride);
    calculate_orb_descriptors_stripe_optimized16(pattern_vecs, vtcmImageCache, 80 + HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[0], (int16_t*)encodedAngles[0], featuresCount[0], 0, orbDescriptors[0]);
    for (int j = 1; j < subImageCount - 1; j++) {
        cache_stripe_vtcm_2_border_left(inputImage + (80 * j - HALF_PATCH_SIZE) * imageStride, vtcmImageCache, 80 + 2 * HALF_PATCH_SIZE, imageStride);
        calculate_orb_descriptors_stripe_optimized16(pattern_vecs, vtcmImageCache, 80 + 2 * HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[j], (int16_t*)encodedAngles[j], featuresCount[j], 0, orbDescriptors[j]);
    }
    cache_stripe_vtcm_2_border_left(inputImage + (80 * (subImageCount - 1) - HALF_PATCH_SIZE) * imageStride, vtcmImageCache, bottomOffset + HALF_PATCH_SIZE, imageStride);
    calculate_orb_descriptors_stripe_optimized16(pattern_vecs, vtcmImageCache, bottomOffset + HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[subImageCount - 1], (int16_t*)encodedAngles[subImageCount - 1], featuresCount[subImageCount - 1], 0, orbDescriptors[subImageCount - 1]);

    // Process middle stripes
    for (int i = 1; i < subImageCount - 1; i++) {
        cache_stripe_vtcm_2_no_border(inputImage + i * VLEN, vtcmImageCache, 80 + HALF_PATCH_SIZE, imageStride);
        calculate_orb_descriptors_stripe_optimized16(pattern_vecs, vtcmImageCache, 80 + HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[subImageCount * i], (int16_t*)encodedAngles[subImageCount * i], featuresCount[subImageCount * i], 0, orbDescriptors[subImageCount * i]);
        for (int j = 1; j < subImageCount - 1; j++) {
            cache_stripe_vtcm_2_no_border(inputImage + i * VLEN + (80 * j - HALF_PATCH_SIZE) * imageStride, vtcmImageCache, 80 + 2 * HALF_PATCH_SIZE, imageStride);
            calculate_orb_descriptors_stripe_optimized16(pattern_vecs, vtcmImageCache, 80 + 2 * HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[subImageCount * i + j], (int16_t*)encodedAngles[subImageCount * i + j], featuresCount[subImageCount * i + j], 0, orbDescriptors[subImageCount * i + j]);
        }
        cache_stripe_vtcm_2_no_border(inputImage + i * VLEN + (80 * (subImageCount - 1) - HALF_PATCH_SIZE) * imageStride, vtcmImageCache, bottomOffset + HALF_PATCH_SIZE, imageStride);
        calculate_orb_descriptors_stripe_optimized16(pattern_vecs, vtcmImageCache, bottomOffset + HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[subImageCount * i + subImageCount - 1], (int16_t*)encodedAngles[subImageCount * i + subImageCount - 1], featuresCount[subImageCount * i + subImageCount - 1], 0, orbDescriptors[subImageCount * i + subImageCount - 1]);
    }

    // Process right border stripes
    cache_stripe_vtcm_2_border_right(inputImage + (subImageCount - 1) * VLEN, vtcmImageCache, 80 + HALF_PATCH_SIZE, imageStride);
    calculate_orb_descriptors_stripe_optimized16(pattern_vecs, vtcmImageCache, 80 + HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[subImageCount * (subImageCount - 1)], (int16_t*)encodedAngles[subImageCount * (subImageCount - 1)], featuresCount[subImageCount * (subImageCount - 1)], 0, orbDescriptors[subImageCount * (subImageCount - 1)]);
    for (int j = 1; j < subImageCount - 1; j++) {
        cache_stripe_vtcm_2_border_right(inputImage + (subImageCount - 1) * VLEN + (80 * j - HALF_PATCH_SIZE) * imageStride, vtcmImageCache, 80 + 2 * HALF_PATCH_SIZE, imageStride);
        calculate_orb_descriptors_stripe_optimized16(pattern_vecs, vtcmImageCache, 80 + 2 * HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[subImageCount * (subImageCount - 1) + j], (int16_t*)encodedAngles[subImageCount * (subImageCount - 1) + j], featuresCount[subImageCount * (subImageCount - 1) + j], 0, orbDescriptors[subImageCount * (subImageCount - 1) + j]);
    }
    cache_stripe_vtcm_2_border_right(inputImage + (subImageCount - 1) * VLEN + (80 * (subImageCount - 1) - HALF_PATCH_SIZE) * imageStride, vtcmImageCache, bottomOffset + HALF_PATCH_SIZE, imageStride);
    calculate_orb_descriptors_stripe_optimized16(pattern_vecs, vtcmImageCache, bottomOffset + HALF_PATCH_SIZE, VLEN, 2 * VLEN, points[subImageCount * (subImageCount - 1) + subImageCount - 1], (int16_t*)encodedAngles[subImageCount * (subImageCount - 1) + subImageCount - 1], featuresCount[subImageCount * (subImageCount - 1) + subImageCount - 1], 0, orbDescriptors[subImageCount * (subImageCount - 1) + subImageCount - 1]);
}

/// @brief Round a floating-point number to the nearest even integer
/// @param num Input number
/// @return Rounded integer
int16_t roundClosest(float num) {
    return (int16_t)lrint(num);
}

/// @brief Divide and round to the nearest integer
/// @param num Numerator
/// @param den Denominator
/// @return Rounded quotient
int16_t divRoundClosest(int16_t num, int16_t den) {
    return num / den;
}

/// @brief Compute ORB descriptors for a stripe of feature points (scalar implementation)
/// @param vtcmImageCache VTCM cache structure
/// @param height Stripe height in pixels
/// @param width Stripe width in pixels
/// @param stride Image stride in bytes
/// @param points Feature point coordinates
/// @param encodedAngles Encoded angle data
/// @param featureCount Number of features in the stripe
/// @param xoffset Horizontal offset
/// @param orbDescriptors Output ORB descriptors
void calculate_orb_descriptors_stripe_scalar(pyramid_VTCM_cache_t* vtcmImageCache, uint32_t height, uint32_t width, int stride, uint16_t* points, 
                                            int16_t* encodedAngles, uint32_t featureCount, uint32_t xoffset, orb_descriptors_vecs_t* orbDescriptors) {
    if (featureCount == 0) {
        return;
    }

    uint32_t iterationsCount = (featureCount - 1) / (VLEN / 2) + 1;

    // Scalar implementation for comparison
    for (int i = 0; i < iterationsCount; i++) {
        for (int k = 0; k < VLEN / 2; k++) {
            uint16_t pointPos = points[i * VLEN / 2 + k];
            uint16_t encodedAngle = encodedAngles[i * VLEN / 2 + k];
            int16_t sin_raw = ((int8_t)((encodedAngle & 0b1111111100000000) >> 8));
            int16_t cos_raw = ((int8_t)(encodedAngle & 0b0000000011111111));

            int8_t* pattern = vtcmImageCache->orb_descriptors_patterns;
            uint8_t* center = vtcmImageCache->cacheData + pointPos;

#define GET_VALUE(idx) \
            center[divRoundClosest(pattern[2 * idx] * sin_raw + pattern[2 * idx + 1] * cos_raw, VLEN / 2) * stride + \
                   divRoundClosest(pattern[2 * idx] * cos_raw - pattern[2 * idx + 1] * sin_raw, VLEN / 2)]

            for (int j = 0; j < 16; ++j, pattern += 64) {
                int t0, t1, val;
                t0 = GET_VALUE(0); t1 = GET_VALUE(1);
                val = t0 < t1;
                t0 = GET_VALUE(2); t1 = GET_VALUE(3);
                val |= (t0 < t1) << 1;
                t0 = GET_VALUE(4); t1 = GET_VALUE(5);
                val |= (t0 < t1) << 2;
                t0 = GET_VALUE(6); t1 = GET_VALUE(7);
                val |= (t0 < t1) << 3;
                t0 = GET_VALUE(8); t1 = GET_VALUE(9);
                val |= (t0 < t1) << 4;
                t0 = GET_VALUE(10); t1 = GET_VALUE(11);
                val |= (t0 < t1) << 5;
                t0 = GET_VALUE(12); t1 = GET_VALUE(13);
                val |= (t0 < t1) << 6;
                t0 = GET_VALUE(14); t1 = GET_VALUE(15);
                val |= (t0 < t1) << 7;
                t0 = GET_VALUE(16); t1 = GET_VALUE(17);
                val |= (t0 < t1) << 8;
                t0 = GET_VALUE(18); t1 = GET_VALUE(19);
                val |= (t0 < t1) << 9;
                t0 = GET_VALUE(20); t1 = GET_VALUE(21);
                val |= (t0 < t1) << 10;
                t0 = GET_VALUE(22); t1 = GET_VALUE(23);
                val |= (t0 < t1) << 11;
                t0 = GET_VALUE(24); t1 = GET_VALUE(25);
                val |= (t0 < t1) << 12;
                t0 = GET_VALUE(26); t1 = GET_VALUE(27);
                val |= (t0 < t1) << 13;
                t0 = GET_VALUE(28); t1 = GET_VALUE(29);
                val |= (t0 < t1) << 14;
                t0 = GET_VALUE(30); t1 = GET_VALUE(31);
                val |= (t0 < t1) << 15;

                orbDescriptors[i].descriptors_lines[j].descriptor[k] = val;
            }
#undef GET_VALUE
        }
    }
}

/// @brief Compute ORB descriptors for a stripe of feature points (vectorized implementation)
/// @param vtcmImageCache VTCM cache structure
/// @param height Stripe height in pixels
/// @param width Stripe width in pixels
/// @param stride Image stride in bytes
/// @param points Feature point coordinates
/// @param encodedAngles Encoded angle data
/// @param featureCount Number of features in the stripe
/// @param xoffset Horizontal offset
/// @param orbDescriptors Output ORB descriptors
void calculate_orb_descriptors_stripe(pyramid_VTCM_cache_t* vtcmImageCache, uint32_t height, uint32_t width, int stride, uint16_t* points, 
                                     int16_t* encodedAngles, uint32_t featureCount, uint32_t xoffset, orb_descriptors_vecs_t* orbDescriptors) {
    if (featureCount == 0) {
        return;
    }

    uint32_t iterationsCount = (featureCount - 1) / (VLEN / 2) + 1;
    HVX_Vector* inputCursor = (HVX_Vector*)(points);
    HVX_Vector* cosSinCursor = (HVX_Vector*)(encodedAngles);

    for (int i = 0; i < iterationsCount; i++) {
        HVX_Vector centerIndices = *inputCursor++;
        HVX_Vector cosSinVector = *cosSinCursor++;
        HVX_Vector sinCosVector = Q6_Vh_vlsr_VhVh(cosSinVector, Q6_Vh_vsplat_R(8)) + Q6_Vh_vasl_VhVh(cosSinVector, Q6_Vh_vsplat_R(8));
        int8_t* pattern = vtcmImageCache->orb_descriptors_patterns;

#define GET_VALUE(t_vec, idx) \
            x_pattern = (uint8_t)pattern[2 * idx]; \
            y_pat_signed = pattern[2 * idx + 1]; \
            y_pattern = (uint8_t)y_pat_signed; \
            y_pattern_neg = (uint8_t)(-y_pat_signed); \
            pattern_vec_1 = Q6_Vb_vsplat_R(x_pattern); \
            pattern_vec_2 = Q6_Vh_vsplat_R(y_pattern_neg + 256 * y_pattern); \
            rotated_vec = Q6_Wh_vmpy_VbVb(cosSinVector, pattern_vec_1); \
            rotated_vec = Q6_Wh_vadd_WhWh(rotated_vec, Q6_Wh_vmpy_VbVb(sinCosVector, pattern_vec_2)); \
            x_pattern_rotated_vec = Q6_V_lo_W(rotated_vec); \
            y_pattern_rotated_vec = Q6_V_hi_W(rotated_vec); \
            x_pattern_rotated_vec = Q6_Vh_vasr_VhR(Q6_Vh_vadd_VhVh(x_pattern_rotated_vec, Q6_Vh_vlsr_VhVh(Q6_Q_vcmp_gt_VhVh(Q6_Vh_vsplat_R(0), x_pattern_rotated_vec), Q6_Vh_vsplat_R(10))), 6); \
            y_pattern_rotated_vec = Q6_Vh_vadd_VhVh(Q6_Vh_vasl_VhR(y_pattern_rotated_vec, 2), Q6_Vh_vlsr_VhVh(Q6_Q_vcmp_gt_VhVh(Q6_Vh_vsplat_R(0), y_pattern_rotated_vec), Q6_Vh_vsplat_R(8))); \
            y_pattern_rotated_vec &= Q6_Vh_vsplat_R(0xFF00); \
            values_addr = Q6_Vh_vadd_VhVh(centerIndices, Q6_Vh_vadd_VhVh(x_pattern_rotated_vec, y_pattern_rotated_vec)); \
            Q6_vgather_ARMVh(&vtcmImageCache->gatheredVector1, (uint32_t)vtcmImageCache->cacheData, height * stride - 1, values_addr); \
            t_vec = Q6_Vb_vshuffe_VbVb(Q6_V_vzero(), vtcmImageCache->gatheredVector1)

        for (int j = 0; j < 16; ++j, pattern += 64) {
            HVX_VectorPair rotated_vec;
            HVX_Vector t0_vec, t1_vec, val_vec, values_addr, x_pattern_rotated_vec, y_pattern_rotated_vec, pattern_vec_1, pattern_vec_2;
            uint8_t x_pattern, y_pattern, y_pattern_neg;
            int8_t y_pat_signed;

            GET_VALUE(t0_vec, 0); GET_VALUE(t1_vec, 1);
            val_vec = Q6_V_vmux_QVV(Q6_Q_vcmp_gt_VhVh(t1_vec, t0_vec), Q6_Vh_vsplat_R(0x0001), Q6_V_vzero());
            GET_VALUE(t0_vec, 2); GET_VALUE(t1_vec, 3);
            val_vec |= Q6_V_vmux_QVV(Q6_Q_vcmp_gt_VhVh(t1_vec, t0_vec), Q6_Vh_vsplat_R(0x0002), Q6_V_vzero());
            GET_VALUE(t0_vec, 4); GET_VALUE(t1_vec, 5);
            val_vec |= Q6_V_vmux_QVV(Q6_Q_vcmp_gt_VhVh(t1_vec, t0_vec), Q6_Vh_vsplat_R(0x0004), Q6_V_vzero());
            GET_VALUE(t0_vec, 6); GET_VALUE(t1_vec, 7);
            val_vec |= Q6_V_vmux_QVV(Q6_Q_vcmp_gt_VhVh(t1_vec, t0_vec), Q6_Vh_vsplat_R(0x0008), Q6_V_vzero());
            GET_VALUE(t0_vec, 8); GET_VALUE(t1_vec, 9);
            val_vec |= Q6_V_vmux_QVV(Q6_Q_vcmp_gt_VhVh(t1_vec, t0_vec), Q6_Vh_vsplat_R(0x0010), Q6_V_vzero());
            GET_VALUE(t0_vec, 10); GET_VALUE(t1_vec, 11);
            val_vec |= Q6_V_vmux_QVV(Q6_Q_vcmp_gt_VhVh(t1_vec, t0_vec), Q6_Vh_vsplat_R(0x0020), Q6_V_vzero());
            GET_VALUE(t0_vec, 12); GET_VALUE(t1_vec, 13);
            val_vec |= Q6_V_vmux_QVV(Q6_Q_vcmp_gt_VhVh(t1_vec, t0_vec), Q6_Vh_vsplat_R(0x0040), Q6_V_vzero());
            GET_VALUE(t0_vec, 14); GET_VALUE(t1_vec, 15);
            val_vec |= Q6_V_vmux_QVV(Q6_Q_vcmp_gt_VhVh(t1_vec, t0_vec), Q6_Vh_vsplat_R(0x0080), Q6_V_vzero());
            GET_VALUE(t0_vec, 16); GET_VALUE(t1_vec, 17);
            val_vec |= Q6_V_vmux_QVV(Q6_Q_vcmp_gt_VhVh(t1_vec, t0_vec), Q6_Vh_vsplat_R(0x0100), Q6_V_vzero());
            GET_VALUE(t0_vec, 18); GET_VALUE(t1_vec, 19);
            val_vec |= Q6_V_vmux_QVV(Q6_Q_vcmp_gt_VhVh(t1_vec, t0_vec), Q6_Vh_vsplat_R(0x0200), Q6_V_vzero());
            GET_VALUE(t0_vec, 20); GET_VALUE(t1_vec, 21);
            val_vec |= Q6_V_vmux_QVV(Q6_Q_vcmp_gt_VhVh(t1_vec, t0_vec), Q6_Vh_vsplat_R(0x0400), Q6_V_vzero());
            GET_VALUE(t0_vec, 22); GET_VALUE(t1_vec, 23);
            val_vec |= Q6_V_vmux_QVV(Q6_Q_vcmp_gt_VhVh(t1_vec, t0_vec), Q6_Vh_vsplat_R(0x0800), Q6_V_vzero());
            GET_VALUE(t0_vec, 24); GET_VALUE(t1_vec, 25);
            val_vec |= Q6_V_vmux_QVV(Q6_Q_vcmp_gt_VhVh(t1_vec, t0_vec), Q6_Vh_vsplat_R(0x1000), Q6_V_vzero());
            GET_VALUE(t0_vec, 26); GET_VALUE(t1_vec, 27);
            val_vec |= Q6_V_vmux_QVV(Q6_Q_vcmp_gt_VhVh(t1_vec, t0_vec), Q6_Vh_vsplat_R(0x2000), Q6_V_vzero());
            GET_VALUE(t0_vec, 28); GET_VALUE(t1_vec, 29);
            val_vec |= Q6_V_vmux_QVV(Q6_Q_vcmp_gt_VhVh(t1_vec, t0_vec), Q6_Vh_vsplat_R(0x4000), Q6_V_vzero());
            GET_VALUE(t0_vec, 30); GET_VALUE(t1_vec, 31);
            val_vec |= Q6_V_vmux_QVV(Q6_Q_vcmp_gt_VhVh(t1_vec, t0_vec), Q6_Vh_vsplat_R(0x8000), Q6_V_vzero());

            orbDescriptors[i].descriptors_lines[j].vm = val_vec;
        }
    }
}

/// @brief Precompute ORB descriptor patterns for optimized processing
/// @param pattern Input pattern data
/// @param pattern_vecs Output vectorized pattern cache
void orb_descriptors_patterns_optimized_precompute(int8_t* pattern, orb_pattern_vecs_cache_t& pattern_vecs) {
    for (int i = 0; i < 64; i++) {
        for (int j = 0; j < 16; j++) {
            pattern_vecs.pattern_x_vecs[2 * i].pattern[j] = (0x101 * (uint16_t)pattern[16 * i]) + (pattern[16 * i] < 0 ? 0x100 : 0);
            pattern_vecs.pattern_x_vecs[2 * i + 1].pattern[j] = (0x101 * (uint16_t)pattern[16 * i + 2]) + (pattern[16 * i + 2] < 0 ? 0x100 : 0);
            pattern_vecs.pattern_x_vecs[2 * i].pattern[j + 16] = (0x101 * (uint16_t)pattern[16 * i + 4]) + (pattern[16 * i + 4] < 0 ? 0x100 : 0);
            pattern_vecs.pattern_x_vecs[2 * i + 1].pattern[j + 16] = (0x101 * (uint16_t)pattern[16 * i + 6]) + (pattern[16 * i + 6] < 0 ? 0x100 : 0);
            pattern_vecs.pattern_x_vecs[2 * i].pattern[j + 32] = (0x101 * (uint16_t)pattern[16 * i + 8]) + (pattern[16 * i + 8] < 0 ? 0x100 : 0);
            pattern_vecs.pattern_x_vecs[2 * i + 1].pattern[j + 32] = (0x101 * (uint16_t)pattern[16 * i + 10]) + (pattern[16 * i + 10] < 0 ? 0x100 : 0);
            pattern_vecs.pattern_x_vecs[2 * i].pattern[j + 48] = (0x101 * (uint16_t)pattern[16 * i + 12]) + (pattern[16 * i + 12] < 0 ? 0x100 : 0);
            pattern_vecs.pattern_x_vecs[2 * i + 1].pattern[j + 48] = (0x101 * (uint16_t)pattern[16 * i + 14]) + (pattern[16 * i + 14] < 0 ? 0x100 : 0);

            pattern_vecs.pattern_y_vecs[2 * i].pattern[j] = 0x100 * (uint16_t)pattern[16 * i + 1] + ((uint8_t)((-(int8_t)pattern[16 * i + 1])));
            pattern_vecs.pattern_y_vecs[2 * i + 1].pattern[j] = 0x100 * (uint16_t)pattern[16 * i + 3] + ((uint8_t)((-(int8_t)pattern[16 * i + 3])));
            pattern_vecs.pattern_y_vecs[2 * i].pattern[j + 16] = 0x100 * (uint16_t)pattern[16 * i + 5] + ((uint8_t)((-(int8_t)pattern[16 * i + 5])));
            pattern_vecs.pattern_y_vecs[2 * i + 1].pattern[j + 16] = 0x100 * (uint16_t)pattern[16 * i + 7] + ((uint8_t)((-(int8_t)pattern[16 * i + 7])));
            pattern_vecs.pattern_y_vecs[2 * i].pattern[j + 32] = 0x100 * (uint16_t)pattern[16 * i + 9] + ((uint8_t)((-(int8_t)pattern[16 * i + 9])));
            pattern_vecs.pattern_y_vecs[2 * i + 1].pattern[j + 32] = 0x100 * (uint16_t)pattern[16 * i + 11] + ((uint8_t)((-(int8_t)pattern[16 * i + 11])));
            pattern_vecs.pattern_y_vecs[2 * i].pattern[j + 48] = 0x100 * (uint16_t)pattern[16 * i + 13] + ((uint8_t)((-(int8_t)pattern[16 * i + 13])));
            pattern_vecs.pattern_y_vecs[2 * i + 1].pattern[j + 48] = 0x100 * (uint16_t)pattern[16 * i + 15] + ((uint8_t)((-(int8_t)pattern[16 * i + 15])));
        }
    }
}

/// @brief Compute ORB descriptors for a stripe of feature points with optimized 16-bit processing
/// @param pattern_vecs Precomputed pattern vectors
/// @param vtcmImageCache VTCM cache structure
/// @param height Stripe height in pixels
/// @param width Stripe width in pixels
/// @param stride Image stride in bytes
/// @param points Feature point coordinates
/// @param encodedAngles Encoded angle data
/// @param featureCount Number of features in the stripe
/// @param xoffset Horizontal offset
/// @param orbDescriptors Output ORB descriptors
void calculate_orb_descriptors_stripe_optimized16(orb_pattern_vecs_cache_t& pattern_vecs, pyramid_VTCM_cache_t* vtcmImageCache, uint32_t height, uint32_t width, 
                                                  int stride, uint16_t* points, int16_t* encodedAngles, uint32_t featureCount, uint32_t xoffset, 
                                                  orb_descriptors_vecs_t* orbDescriptors) {
    // Early exit if no features
    if (featureCount == 0) {
        return;
    }

    // Error handling for excessive features
    if (featureCount > 16) {
        return;
    }

    // Compute number of iterations
    uint32_t iterationsCount = (featureCount - 1) / (VLEN / 2) + 1;

    // Initialize cursors
    HVX_Vector* inputCursor = (HVX_Vector*)(points);
    HVX_Vector* cosSinCursor = (HVX_Vector*)(encodedAngles);

    // Initialize shift vector for bit alignment
    union {
        HVX_Vector vm;
        uint16_t u16[VLEN / 2];
    } shiftVector;
    for (int i = 0; i < VLEN / 2; i++) {
        shiftVector.u16[i] = i / 16;
    }

    // Process each iteration
    for (int i = 0; i < iterationsCount; i++) {
        HVX_Vector centerIndices = (*inputCursor++);
        HVX_Vector cosSinVector = (*cosSinCursor++);
        HVX_Vector sinCosVector = Q6_Vh_vlsr_VhVh(cosSinVector, Q6_Vh_vsplat_R(8)) + Q6_Vh_vasl_VhVh(cosSinVector, Q6_Vh_vsplat_R(8));

        // Make centerIndices periodic for pattern processing
        HVX_Vector centerIndices1 = Q6_V_vlalign_VVR(centerIndices, Q6_V_vzero(), 48 * 2);
        HVX_Vector centerIndices2 = Q6_V_valign_VVR(Q6_V_vzero(), centerIndices1, 16 * 2);
        HVX_Vector centerIndices3 = Q6_V_valign_VVR(Q6_V_vzero(), centerIndices1, 32 * 2);
        HVX_Vector centerIndices4 = Q6_V_valign_VVR(Q6_V_vzero(), centerIndices1, 48 * 2);
        centerIndices = centerIndices1 | centerIndices2 | centerIndices3 | centerIndices4;

        // Make cosSinVector periodic
        HVX_Vector cosSinVector1 = Q6_V_vlalign_VVR(cosSinVector, Q6_V_vzero(), 48 * 2);
        HVX_Vector cosSinVector2 = Q6_V_valign_VVR(Q6_V_vzero(), cosSinVector1, 16 * 2);
        HVX_Vector cosSinVector3 = Q6_V_valign_VVR(Q6_V_vzero(), cosSinVector1, 32 * 2);
        HVX_Vector cosSinVector4 = Q6_V_valign_VVR(Q6_V_vzero(), cosSinVector1, 48 * 2);
        cosSinVector = cosSinVector1 | cosSinVector2 | cosSinVector3 | cosSinVector4;

        // Make sinCosVector periodic
        HVX_Vector sinCosVector1 = Q6_V_vlalign_VVR(sinCosVector, Q6_V_vzero(), 48 * 2);
        HVX_Vector sinCosVector2 = Q6_V_valign_VVR(Q6_V_vzero(), sinCosVector1, 16 * 2);
        HVX_Vector sinCosVector3 = Q6_V_valign_VVR(Q6_V_vzero(), sinCosVector1, 32 * 2);
        HVX_Vector sinCosVector4 = Q6_V_valign_VVR(Q6_V_vzero(), sinCosVector1, 48 * 2);
        sinCosVector = sinCosVector1 | sinCosVector2 | sinCosVector3 | sinCosVector4;

        int8_t* pattern = vtcmImageCache->orb_descriptors_patterns;  // DEBUG reference

#define GET_VALUES(t_vec, pattern_x_vec, pattern_y_vec) \
        rotated_vec = Q6_Wh_vmpy_VbVb(cosSinVector, pattern_x_vec); \
        rotated_vec = Q6_Wh_vadd_WhWh(rotated_vec, Q6_Wh_vmpy_VbVb(sinCosVector, pattern_y_vec)); \
        x_pattern_rotated_vec = Q6_V_lo_W(rotated_vec); \
        y_pattern_rotated_vec = Q6_V_hi_W(rotated_vec); \
        x_pattern_rotated_vec = Q6_Vh_vasr_VhR(Q6_Vh_vadd_VhVh(x_pattern_rotated_vec, Q6_Vh_vlsr_VhVh(Q6_Q_vcmp_gt_VhVh(Q6_Vh_vsplat_R(0), x_pattern_rotated_vec), Q6_Vh_vsplat_R(10))), 6); \
        y_pattern_rotated_vec = Q6_Vh_vadd_VhVh(Q6_Vh_vasl_VhR(y_pattern_rotated_vec, 2), Q6_Vh_vlsr_VhVh(Q6_Q_vcmp_gt_VhVh(Q6_Vh_vsplat_R(0), y_pattern_rotated_vec), Q6_Vh_vsplat_R(8))); \
        y_pattern_rotated_vec &= Q6_Vh_vsplat_R(0xFF00); \
        values_addr = Q6_Vh_vadd_VhVh(centerIndices, Q6_Vh_vadd_VhVh(x_pattern_rotated_vec, y_pattern_rotated_vec)); \
        Q6_vgather_ARMVh(&vtcmImageCache->gatheredVector1, (uint32_t)vtcmImageCache->cacheData, (VLEN * 2) * (LEVEL7_HEIGHT + HALF_PATCH_SIZE * 2 + 1) - 1, values_addr); \
        t_vec = Q6_Vb_vshuffe_VbVb(Q6_V_vzero(), vtcmImageCache->gatheredVector1)

        // Process 16 descriptor lines
        for (int j = 0; j < 16; ++j) {
            HVX_VectorPair rotated_vec;
            HVX_Vector t0_vec, t1_vec, val_vec, values_addr, x_pattern_rotated_vec, y_pattern_rotated_vec, pattern_vec_1, pattern_vec_2;
            uint8_t x_pattern, y_pattern, y_pattern_neg;
            int8_t y_pat_signed;

            GET_VALUES(t0_vec, pattern_vecs.pattern_x_vecs[8 * j].vm, pattern_vecs.pattern_y_vecs[8 * j].vm);
            GET_VALUES(t1_vec, pattern_vecs.pattern_x_vecs[8 * j + 1].vm, pattern_vecs.pattern_y_vecs[8 * j + 1].vm);
            val_vec = Q6_V_vmux_QVV(Q6_Q_vcmp_gt_VhVh(t1_vec, t0_vec), Q6_Vh_vsplat_R(0x0001), Q6_V_vzero());

            GET_VALUES(t0_vec, pattern_vecs.pattern_x_vecs[8 * j + 2].vm, pattern_vecs.pattern_y_vecs[8 * j + 2].vm);
            GET_VALUES(t1_vec, pattern_vecs.pattern_x_vecs[8 * j + 3].vm, pattern_vecs.pattern_y_vecs[8 * j + 3].vm);
            val_vec |= Q6_V_vmux_QVV(Q6_Q_vcmp_gt_VhVh(t1_vec, t0_vec), Q6_Vh_vsplat_R(0x0010), Q6_V_vzero());

            GET_VALUES(t0_vec, pattern_vecs.pattern_x_vecs[8 * j + 4].vm, pattern_vecs.pattern_y_vecs[8 * j + 4].vm);
            GET_VALUES(t1_vec, pattern_vecs.pattern_x_vecs[8 * j + 5].vm, pattern_vecs.pattern_y_vecs[8 * j + 5].vm);
            val_vec |= Q6_V_vmux_QVV(Q6_Q_vcmp_gt_VhVh(t1_vec, t0_vec), Q6_Vh_vsplat_R(0x0100), Q6_V_vzero());

            GET_VALUES(t0_vec, pattern_vecs.pattern_x_vecs[8 * j + 6].vm, pattern_vecs.pattern_y_vecs[8 * j + 6].vm);
            GET_VALUES(t1_vec, pattern_vecs.pattern_x_vecs[8 * j + 7].vm, pattern_vecs.pattern_y_vecs[8 * j + 7].vm);
            val_vec |= Q6_V_vmux_QVV(Q6_Q_vcmp_gt_VhVh(t1_vec, t0_vec), Q6_Vh_vsplat_R(0x1000), Q6_V_vzero());

            // Shift and align bits for descriptor
            val_vec = Q6_Vh_vasl_VhVh(val_vec, shiftVector.vm);
            val_vec |= Q6_V_valign_VVR(Q6_V_vzero(), val_vec, 32 * 2);
            val_vec |= Q6_V_valign_VVR(Q6_V_vzero(), val_vec, 16 * 2);

            orbDescriptors[i].descriptors_lines[j].vm = val_vec;
        }
#undef GET_VALUES
    }
}