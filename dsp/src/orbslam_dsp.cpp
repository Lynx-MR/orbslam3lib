/**
 * @file orbslam_dsp.cpp
 *
 * @author Gaston Rouquette (Lynx Mixed Reality)
 *
 * @brief The main program running on the DSP to manage the features extraction and matching, also manages HWA calls.
**/



// Uncomment to switch to simulator mode, also uncomment # liborbslam3_skel.C_SRCS += orbslam_dsp_tests.cpp in hexagon.min (not in open-source release)
// #define TEST_MODE

#ifndef TEST_MODE

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

// Avoid unused warning error
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wunused-variable"

#include "HAP_farf.h"
#include "HAP_vtcm_mgr.h"
#include "HAP_compute_res.h"

#include "AEEStdErr.h"

#include "orbslam3.h"

#include "hvx_internal.h"

#include "orbslam_rb_cos_sin_lut.h"
#include "orbslam_dsp_pyramid.h"
#include "orbslam_dsp_fast.h"
#include "orbslam_dsp_sort.h"
#include "orbslam_dsp_rb.h"
#include "orbslam_dsp_matcher.h"

#include "qurt.h"

#include <cmath>

#include "hexagon_sim_timer.h"

#include "orbslam_dsp_hwa_pipeline.h"

#define USED_HWA_MODULES (HWA_EXTRACTOR_MODULE)

// LEGACY - UNUSED
#define THRESHOLD 17 // Threshold used to get features during the first step
#define MIN_THRESHOLD 17 // Threshold used to get features during the second step (Optional)
#define MIN_FEATURES 0 // Minimum features to get during the first step inside a subimage (128x80 block) to continue the process without trying to get more features with a lower threshold (Second step MIN_THRESHOLD) (0 = Disable second detection)
#define FINAL_SCORE_THRESHOLD 17 // Threshold used to keep the features after the non-maximal suppression

#define MAXIMUM_BEGINNING_FEATURES_PER_SUBIMAGE 256 // Count of features to get during the first step fast-predection
#define MAXIMUM_INTERMEDIATE_FEATURES_PER_SUBIMAGE 256 // Maximum features to keep before the sorting step
#define MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE 16 // Final maximum features to keep at the end of FAST, the best features are kept (should not be more than 16 for optimized ORB descriptor computation)
#define MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE_ROUNDED 64 // MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE rounded up to the next multiple of 64

// We define the total count of sub-images by doing the sum of all the sub-images of each level
#define TOTAL_SUBIMAGES \
    (((INPUT_WIDTH-1)/VLEN + 1)*((INPUT_WIDTH-1)/VLEN + 1) +\
    ((LEVEL1_WIDTH-1)/VLEN + 1)*((LEVEL1_WIDTH-1)/VLEN + 1) +\
    ((LEVEL2_WIDTH-1)/VLEN + 1)*((LEVEL2_WIDTH-1)/VLEN + 1) +\
    ((LEVEL3_WIDTH-1)/VLEN + 1)*((LEVEL3_WIDTH-1)/VLEN + 1) +\
    ((LEVEL4_WIDTH-1)/VLEN + 1)*((LEVEL4_WIDTH-1)/VLEN + 1) +\
    ((LEVEL5_WIDTH-1)/VLEN + 1)*((LEVEL5_WIDTH-1)/VLEN + 1) +\
    ((LEVEL6_WIDTH-1)/VLEN + 1)*((LEVEL6_WIDTH-1)/VLEN + 1) +\
    ((LEVEL7_WIDTH-1)/VLEN + 1)*((LEVEL7_WIDTH-1)/VLEN + 1))

// The stack memory size for each thread, 16KB, should be a bit too much, but it's better to have a bit more than not enough
#define STACK_SIZE 1024*16

// THREADS -- BOTH THREADS USED FOR EXTRACTIONS, EACH THREAD CAN USE ONE OF THE TWO HVX CORES -- Some matching is done in parallel too using these threads
qurt_thread_t extractor_left_thread;
qurt_thread_t extractor_right_thread;

// MUTEXES -- Used to synchronize the threads for signaling beginning and end of the extraction threads works
qurt_mutex_t extractor_mutex;

// SIGNALS -- Used to signal the beginning and end of the extraction threads works
qurt_cond_t extractor_left_cond; // BEGIN
qurt_cond_t extractor_right_cond; // BEGIN

qurt_cond_t extractor_left_join_cond; // END
qurt_cond_t extractor_right_join_cond; // END

// BARRIERS -- Used to synchronize both threads at the end of the extraction step to start the matching step
qurt_barrier_t extractor_to_matcher_barrier;

// BOOLS -- Used to know if the threads are ready to start their work
bool extractor_left_ready = false;
bool extractor_right_ready = false;

bool stop_threads = false; // Used to stop the threads when the extraction program is closed

void* thread_stacks[2]; // Contains the stack memory pointers for each thread

// HWA Pipeline context
orbslam_dsp_hwa_pipeline_context_t* hwa_pipeline_context = nullptr; // HWA pipeline context used for the HWA feature extraction

// Global pointers to the input images, and structures used for the extraction
uint8_t* inputImages = nullptr;
pyramid_t* imgPyramidHVX; // Pyramid used for the left image
pyramid_t* imgPyramidHVX2; // Pyramid used for the right image
pyramid_t* imgPyramidBlurHVX; // Gaussian version of the left pyramid
pyramid_t* imgPyramidBlurHVX2; // Gaussian version of the right pyramid
pyramid_VTCM_cache_t* imgPyramidCache; // VTCM Cache structure used for the left image
pyramid_VTCM_cache_t* imgPyramidCache2; // VTCM Caches structure used for the right image

/* Structures containing points and scores ptr */
uint16_t* points_hvx_temp[((INPUT_WIDTH-1)/VLEN+1)*((INPUT_WIDTH-1)/VLEN+1)]; // Left image points positions, VTCM Memory, used for every image, used to store the points before the sorting step
uint16_t* points_hvx[TOTAL_SUBIMAGES]; // Left image points positions, Non-VTCM Memory, final for each image, used to store the points after the sorting step, contains the final points of the pyramid
uint16_t* scores_hvx_temp[((INPUT_WIDTH-1)/VLEN+1)*((INPUT_WIDTH-1)/VLEN+1)]; // Left image scores, VTCM Memory, used for every image, used to store the scores before the sorting step
uint16_t* scores_hvx[TOTAL_SUBIMAGES]; // Left image scores, Non-VTCM Memory, final for each image, used to store the scores after the sorting step, contains the final scores of the pyramid

uint16_t* points_hvx_2_temp[((INPUT_WIDTH-1)/VLEN+1)*((INPUT_WIDTH-1)/VLEN+1)]; // Right image points positions, VTCM Memory, used for every image, used to store the points before the sorting step
uint16_t* points_hvx_2[TOTAL_SUBIMAGES]; // Right image points positions, Non-VTCM Memory, final for each image, used to store the points after the sorting step, contains the final points of the pyramid
uint16_t* scores_hvx_2_temp[((INPUT_WIDTH-1)/VLEN+1)*((INPUT_WIDTH-1)/VLEN+1)]; // Right image scores, VTCM Memory, used for every image, used to store the scores before the sorting step
uint16_t* scores_hvx_2[TOTAL_SUBIMAGES]; // Right image scores, Non-VTCM Memory, final for each image, used to store the scores after the sorting step, contains the final scores of the pyramid

uint16_t* encodedAngles[TOTAL_SUBIMAGES]; // Encoded angles for each point of the left pyramid, basically 8-bits cos and 8-bits sin per value
uint16_t* encodedAngles_2[TOTAL_SUBIMAGES]; // Encoded angles for each point of the right pyramid, basically 8-bits cos and 8-bits sin per value

uint32_t points_count[TOTAL_SUBIMAGES]; // Count of points for each sub-image of the left pyramid
uint32_t points_count_2[TOTAL_SUBIMAGES]; // Count of points for each sub-image of the right pyramid

orb_descriptors_vecs_t* orb_descriptors[TOTAL_SUBIMAGES]; // ORB descriptors for each sub-image of the left pyramid
orb_descriptors_vecs_t* orb_descriptors_2[TOTAL_SUBIMAGES]; // ORB descriptors for each sub-image of the right pyramid

uint32_t umax[HALF_PATCH_SIZE + 1]; // Precomputed values for the encoded angles computations

orb_pattern_vecs_cache_t* pattern_vecs; // Precomputed pattern values for faster ORB descriptor computation

// Matching structures
orb_descriptors_vecs_t* _leftDescriptorsVecsMatching1; // ORB descriptors for the left image used for the first matching step (All the sub-images are merged)
orb_descriptors_vecs_t* _rightDescriptorsVecsMatching1; // ORB descriptors for the right image used for the first matching step (All the sub-images are merged)

// Temporary memory for matching calculations
uint16_t* _matchingTemp1;
uint16_t* _matchingTemp2;
uint16_t* _matchingTemp3;
uint16_t* _matchingTemp4;

// Thread 2
orb_descriptors_vecs_t* _leftDescriptorsVecsMatching2;
orb_descriptors_vecs_t* _rightDescriptorsVecsMatching2;
// TEMP MEMORY
uint16_t* _matchingTemp5;
uint16_t* _matchingTemp6;
uint16_t* _matchingTemp7;
uint16_t* _matchingTemp8;

// ARGS COPY
// Mono-Stereo Points separation for more efficient matching
int _vLappingAreaLeft0; // Left lapping area 0
int _vLappingAreaLeft1; // Left lapping area 1
int _vLappingAreaRight0; // Right lapping area 0
int _vLappingAreaRight1; // Right lapping area 1

int* _totalPointsLeft; // Total points for the left image
int* _totalPointsRight; // Total points for the right image
int* _monoLeft; // Mono points for the left image
int* _monoRight; // Mono points for the right image
int* _leftKeypointsX; // Left keypoints X Positions
int* _leftKeypointsY; // Left keypoints Y Positions
int* _leftKeypointsAngle; // Left keypoints angle
int* _leftKeypointsLevel; // Left keypoints level (Pyramid level)
int* _rightKeypointsX; // Right keypoints X Positions
int* _rightKeypointsY; // Right keypoints Y Positions
int* _rightKeypointsAngle; // Right keypoints angle
int* _rightKeypointsLevel; // Right keypoints level (Pyramid level)
uint8_t* _leftKeypointsORB; // Left keypoints ORB descriptors
uint8_t* _rightKeypointsORB; // Right keypoints ORB descriptors

short* _indices; // Matching indices (The most corresponding points)
int _indicesLen; // Matching indices length
short* _distances1; // Matching distances 1 (Distance to the most corresponding points)
int _distances1Len; // Matching distances 1 length
short* _distances2; // Matching distances 2 (Distance to the second most corresponding points)
int _distances2Len; // Matching distances 2 length

// Helper method to allocate VTCM Memory (MAX is 256KiB (XR2 Gen 1))
unsigned int vtcm_context_id = 0;

/// @brief Helper method to allocate VTCM Memory (MAX is 256KiB (XR2 Gen 1))
/// @param addr Pointer to the allocated VTCM Memory
/// @param vtcm_size Size of the VTCM Memory to allocate
/// @return 0 if success, -1 if error
int allocateVTCM(void** addr, unsigned int vtcm_size) {
    // Initialize compute resource attributes
    compute_res_attr_t res_info;
    int retVal = 0;

    retVal = HAP_compute_res_attr_init(&res_info);

    // Enable serialization (remove to reduce lag if possible)
    unsigned char b_serialize = 1;
    retVal = HAP_compute_res_attr_set_serialize(&res_info, b_serialize);

    // Set VTCM parameters (single page for efficiency)
    unsigned char b_vtcm_single_page = 1;
    retVal = HAP_compute_res_attr_set_vtcm_param(&res_info, vtcm_size, b_vtcm_single_page);

    // Acquire VTCM context
    vtcm_context_id = 0;
    if (0 == (vtcm_context_id = HAP_compute_res_acquire(&res_info, 10000))) {
        return -1;
    }

    // Get VTCM start address
    void* vtcm_start_address = HAP_compute_res_attr_get_vtcm_ptr(&res_info);
    if (0 == vtcm_start_address) {
        HAP_compute_res_release(vtcm_context_id);
        return -1;
    }

    *addr = vtcm_start_address;
    return 0;
}

/// @brief Release VTCM Memory, Very important or the VTCM will be locked for other processes
/// @return 0 if success
int clearVTCM() {
    return HAP_compute_res_release(vtcm_context_id);
}

/// @brief Method to do memory aligned NON-VTCM allocations and prepare calculation caches and structures at the beginning of the program
/// @return 0 if success
int prepare() {
    // Allocate aligned memory for pyramids (NON-VTCM)
    imgPyramidHVX = (pyramid_t*)memalign(VLEN, sizeof(pyramid_t));
    imgPyramidHVX2 = (pyramid_t*)memalign(VLEN, sizeof(pyramid_t));
    imgPyramidBlurHVX = (pyramid_t*)memalign(VLEN, sizeof(pyramid_t));
    imgPyramidBlurHVX2 = (pyramid_t*)memalign(VLEN, sizeof(pyramid_t));

    // Allocate aligned memory for points and scores (NON-VTCM)
    uint16_t* points_hvx_mem = (uint16_t*)memalign(VLEN, MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE_ROUNDED * sizeof(uint16_t) * TOTAL_SUBIMAGES);
    uint16_t* points_hvx_mem_2 = (uint16_t*)memalign(VLEN, MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE_ROUNDED * sizeof(uint16_t) * TOTAL_SUBIMAGES);
    uint16_t* score_hvx_mem = (uint16_t*)memalign(VLEN, MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE_ROUNDED * sizeof(uint16_t) * TOTAL_SUBIMAGES);
    uint16_t* score_hvx_mem_2 = (uint16_t*)memalign(VLEN, MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE_ROUNDED * sizeof(uint16_t) * TOTAL_SUBIMAGES);

    // Allocate angles and descriptors for each sub-image
    for (int i = 0; i < TOTAL_SUBIMAGES; i++) {
        encodedAngles[i] = (uint16_t*)memalign(VLEN, MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE_ROUNDED * sizeof(uint16_t));
        encodedAngles_2[i] = (uint16_t*)memalign(VLEN, MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE_ROUNDED * sizeof(uint16_t));

        points_hvx[i] = points_hvx_mem + i * MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE_ROUNDED;
        points_hvx_2[i] = points_hvx_mem_2 + i * MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE_ROUNDED;

        scores_hvx[i] = score_hvx_mem + i * MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE_ROUNDED;
        scores_hvx_2[i] = score_hvx_mem_2 + i * MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE_ROUNDED;

        orb_descriptors[i] = (orb_descriptors_vecs_t*)memalign(VLEN, MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE_ROUNDED * sizeof(orb_descriptors_vecs_t));
        orb_descriptors_2[i] = (orb_descriptors_vecs_t*)memalign(VLEN, MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE_ROUNDED * sizeof(orb_descriptors_vecs_t));
    }

    // Generate u_max cache for encoded angles computation
    int v, v0, vmax = std::floor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);
    int vmin = std::ceil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
    const double hp2 = HALF_PATCH_SIZE * HALF_PATCH_SIZE;
    for (v = 0; v <= vmax; ++v) {
        umax[v] = std::round(sqrt(hp2 - v * v));
    }

    // Ensure symmetry in u_max
    for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v) {
        while (umax[v0] == umax[v0 + 1]) {
            ++v0;
        }
        umax[v] = v0;
        ++v0;
    }

    // Precompute bilinear interpolation cache
    precompute_horizontal_bilinear_indices_and_coefs();

    // Precompute ORB-Descriptor cache for optimized computation (16 values per sub-image)
    pattern_vecs = (orb_pattern_vecs_cache_t*)memalign(VLEN, sizeof(orb_pattern_vecs_cache_t));
    orb_descriptors_patterns_optimized_precompute(bit_pattern_31_, *pattern_vecs);

    // Allocate memory for matching structures
    _leftDescriptorsVecsMatching1 = (orb_descriptors_vecs_t*)memalign(VLEN, (TOTAL_SUBIMAGES * MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE_ROUNDED) * sizeof(orb_descriptors_vecs_t) + VLEN * 32);
    _rightDescriptorsVecsMatching1 = (orb_descriptors_vecs_t*)memalign(VLEN, (TOTAL_SUBIMAGES * MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE_ROUNDED) * sizeof(orb_descriptors_vecs_t) + VLEN * 32);
    _leftDescriptorsVecsMatching2 = (orb_descriptors_vecs_t*)memalign(VLEN, (TOTAL_SUBIMAGES * MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE_ROUNDED) * sizeof(orb_descriptors_vecs_t) + VLEN * 32);
    _rightDescriptorsVecsMatching2 = (orb_descriptors_vecs_t*)memalign(VLEN, (TOTAL_SUBIMAGES * MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE_ROUNDED) * sizeof(orb_descriptors_vecs_t) + VLEN * 32);
    _matchingTemp1 = (uint16_t*)memalign(VLEN, (TOTAL_SUBIMAGES * MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE_ROUNDED) * sizeof(uint16_t) + 16 * VLEN);
    _matchingTemp2 = (uint16_t*)memalign(VLEN, (TOTAL_SUBIMAGES * MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE_ROUNDED) * sizeof(uint16_t) + 16 * VLEN);
    _matchingTemp3 = (uint16_t*)memalign(VLEN, (TOTAL_SUBIMAGES * MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE_ROUNDED) * sizeof(uint16_t) + 16 * VLEN);
    _matchingTemp4 = (uint16_t*)memalign(VLEN, (TOTAL_SUBIMAGES * MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE_ROUNDED) * sizeof(uint16_t) + 16 * VLEN);
    _matchingTemp5 = (uint16_t*)memalign(VLEN, (TOTAL_SUBIMAGES * MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE_ROUNDED) * sizeof(uint16_t) + 16 * VLEN);
    _matchingTemp6 = (uint16_t*)memalign(VLEN, (TOTAL_SUBIMAGES * MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE_ROUNDED) * sizeof(uint16_t) + 16 * VLEN);
    _matchingTemp7 = (uint16_t*)memalign(VLEN, (TOTAL_SUBIMAGES * MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE_ROUNDED) * sizeof(uint16_t) + 16 * VLEN);
    _matchingTemp8 = (uint16_t*)memalign(VLEN, (TOTAL_SUBIMAGES * MAXIMUM_FINAL_FEATURES_PER_SUBIMAGE_ROUNDED) * sizeof(uint16_t) + 16 * VLEN);

    // Setup Hardware Accelerator Settings
    hwa_setup_parameters(HWA_FPS, THRESHOLD_ADJUSTER_FEATURES_AVG, THRESHOLD_ADJUSTER_FEATURES_SIGMA);

    return 0;
}

/// @brief This method allocate the VTCM Memory and set the pointers to their correct values of the VTCM Memory. This method needs to be called every frame before the extraction process.
/// @return 0 if success
int prepareVTCM() {
    // Allocate VTCM memory block
    void* vtcm_addr = nullptr;
    if (allocateVTCM(&vtcm_addr, 2 * (MAXIMUM_BEGINNING_FEATURES_PER_SUBIMAGE * (((INPUT_WIDTH - 1) / VLEN + 1) * ((INPUT_WIDTH - 1) / VLEN + 1))) * (sizeof(uint16_t) + sizeof(uint16_t)) + 2 * sizeof(pyramid_VTCM_cache_t)) != 0) {
        FARF(HIGH, "Error allocating VTCM\n");
        return 3;
    }

    // Assign temporary points and scores in VTCM (first half for left, second for right)
    int j = 0;
    for (; j < (((INPUT_WIDTH - 1) / VLEN + 1) * ((INPUT_WIDTH - 1) / VLEN + 1)); j++) {
        points_hvx_temp[j] = (uint16_t*)((uint8_t*)vtcm_addr + j * MAXIMUM_BEGINNING_FEATURES_PER_SUBIMAGE * sizeof(uint16_t));
    }
    for (; j < 2 * (((INPUT_WIDTH - 1) / VLEN + 1) * ((INPUT_WIDTH - 1) / VLEN + 1)); j++) {
        scores_hvx_temp[j - (((INPUT_WIDTH - 1) / VLEN + 1) * ((INPUT_WIDTH - 1) / VLEN + 1))] = (uint16_t*)((uint8_t*)vtcm_addr + j * MAXIMUM_BEGINNING_FEATURES_PER_SUBIMAGE * sizeof(uint16_t));
    }
    for (; j < 3 * (((INPUT_WIDTH - 1) / VLEN + 1) * ((INPUT_WIDTH - 1) / VLEN + 1)); j++) {
        points_hvx_2_temp[j - 2 * (((INPUT_WIDTH - 1) / VLEN + 1) * ((INPUT_WIDTH - 1) / VLEN + 1))] = (uint16_t*)((uint8_t*)vtcm_addr + j * MAXIMUM_BEGINNING_FEATURES_PER_SUBIMAGE * sizeof(uint16_t));
    }
    for (; j < 4 * (((INPUT_WIDTH - 1) / VLEN + 1) * ((INPUT_WIDTH - 1) / VLEN + 1)); j++) {
        scores_hvx_2_temp[j - 3 * (((INPUT_WIDTH - 1) / VLEN + 1) * ((INPUT_WIDTH - 1) / VLEN + 1))] = (uint16_t*)((uint8_t*)vtcm_addr + j * MAXIMUM_BEGINNING_FEATURES_PER_SUBIMAGE * sizeof(uint16_t));
    }

    // Allocate VTCM Image Cache for left and right
    imgPyramidCache = (pyramid_VTCM_cache_t*)((uint8_t*)vtcm_addr + 2 * (MAXIMUM_BEGINNING_FEATURES_PER_SUBIMAGE * (((INPUT_WIDTH - 1) / VLEN + 1) * ((INPUT_WIDTH - 1) / VLEN + 1))) * (sizeof(uint16_t) + sizeof(uint16_t)));
    imgPyramidCache2 = (pyramid_VTCM_cache_t*)((uint8_t*)vtcm_addr + 2 * (MAXIMUM_BEGINNING_FEATURES_PER_SUBIMAGE * (((INPUT_WIDTH - 1) / VLEN + 1) * ((INPUT_WIDTH - 1) / VLEN + 1))) * (sizeof(uint16_t) + sizeof(uint16_t)) + sizeof(pyramid_VTCM_cache_t));

    // Copy cos and sin lookup tables to VTCM
    memcpy(imgPyramidCache->cos_lookup_table, lut_cos, 256 * 4);
    memcpy(imgPyramidCache->sin_lookup_table, lut_sin, 256 * 4);
    memcpy(imgPyramidCache2->cos_lookup_table, lut_cos, 256 * 4);
    memcpy(imgPyramidCache2->sin_lookup_table, lut_sin, 256 * 4);

    // Copy ORB descriptors patterns to VTCM
    memcpy(imgPyramidCache->orb_descriptors_patterns, bit_pattern_31_, 256 * 4);
    memcpy(imgPyramidCache2->orb_descriptors_patterns, bit_pattern_31_, 256 * 4);

    return 0;
}

/// @brief Clean the memory allocated at the beginning of the program, this method needs to be called at the end of the program (Not FINISHED)
/// @return 0 if success
int clearMemory() {
    free(imgPyramidHVX);
    free(imgPyramidHVX2);
    free(imgPyramidBlurHVX);
    free(imgPyramidBlurHVX2);

    // TODO FREE NON-VTCM MEMORY

    clearVTCM();

    return 0;
}

// This struct contains the data used by the extraction threads
struct extractor_job_data {
    bool is_right;
    uint8_t* inputImg;
    pyramid_t* pyramid;
    pyramid_t* pyramid_blur;
    pyramid_VTCM_cache_t* imgPyramidCache;
    uint16_t* points[TOTAL_SUBIMAGES];
    uint32_t points_count[TOTAL_SUBIMAGES];
    uint16_t* scores[TOTAL_SUBIMAGES];
    uint16_t* points_temp[(((INPUT_WIDTH - 1) / VLEN + 1) * ((INPUT_WIDTH - 1) / VLEN + 1))];
    uint16_t* scores_temp[(((INPUT_WIDTH - 1) / VLEN + 1) * ((INPUT_WIDTH - 1) / VLEN + 1))];
    uint16_t* encodedAngles[TOTAL_SUBIMAGES];
    orb_descriptors_vecs_t* orb_descriptors[TOTAL_SUBIMAGES];
};

extractor_job_data extractor_data_left;  // Data for the left image
extractor_job_data extractor_data_right; // Data for the right image

/// @brief Copy VTCM Data into the left extractor data struct
/// @return 0 if success
void copyVTCMPtrLeft() {
    extractor_data_left.inputImg = inputImages;
    extractor_data_left.pyramid = imgPyramidHVX;
    extractor_data_left.is_right = false;
    extractor_data_left.pyramid_blur = imgPyramidBlurHVX;
    extractor_data_left.imgPyramidCache = imgPyramidCache;

    for (int i = 0; i < TOTAL_SUBIMAGES; i++) {
        extractor_data_left.points[i] = points_hvx[i];
        extractor_data_left.scores[i] = scores_hvx[i];
        extractor_data_left.encodedAngles[i] = encodedAngles[i];
        extractor_data_left.orb_descriptors[i] = orb_descriptors[i];
    }
    for (int i = 0; i < ((INPUT_WIDTH - 1) / VLEN + 1) * ((INPUT_WIDTH - 1) / VLEN + 1); i++) {
        extractor_data_left.points_temp[i] = points_hvx_temp[i];
        extractor_data_left.scores_temp[i] = scores_hvx_temp[i];
    }
}

/// @brief Copy VTCM Data into the right extractor data struct
/// @return 0 if success
void copyVTCMPtrRight() {
    extractor_data_right.inputImg = inputImages;
    extractor_data_right.pyramid = imgPyramidHVX2;
    extractor_data_right.is_right = true;
    extractor_data_right.pyramid_blur = imgPyramidBlurHVX2;
    extractor_data_right.imgPyramidCache = imgPyramidCache2;

    for (int i = 0; i < TOTAL_SUBIMAGES; i++) {
        extractor_data_right.points[i] = points_hvx_2[i];
        extractor_data_right.scores[i] = scores_hvx_2[i];
        extractor_data_right.encodedAngles[i] = encodedAngles_2[i];
        extractor_data_right.orb_descriptors[i] = orb_descriptors_2[i];
    }
    for (int i = 0; i < ((INPUT_WIDTH - 1) / VLEN + 1) * ((INPUT_WIDTH - 1) / VLEN + 1); i++) {
        extractor_data_right.points_temp[i] = points_hvx_2_temp[i];
        extractor_data_right.scores_temp[i] = scores_hvx_2_temp[i];
    }
}

static int debugOffsetCounter = 0; // DEBUG COUNTER

/// @brief This method will gather all the data found by the extraction threads in subimages of the left pyramid and put them into the correct structures for the matching step (and the final output). It will also separate the mono and stereo points for more efficient matching.
/// @param level Pyramid level
/// @param i_offset Offset of the subimages for the current level
/// @param subImagesCount Count of subimages for the current level
/// @param scale Scale of the pyramid level
/// @param leftKeypointsX X Positions of the keypoints for the left image
/// @param leftKeypointsY Y Positions of the keypoints for the left image
/// @param leftKeypointsAngle Angles of the keypoints for the left image
/// @param leftKeypointsLevel Pyramid level of the keypoints for the left image
/// @param stereoLeftIndex Index of the stereo keypoints for the left image
/// @param left_descriptors ORB descriptors of the left image
/// @param vLapping0 Lapping area 0 for the left image
/// @param vLapping1 Lapping area 1 for the left image
/// @param vMono Mono keypoints index for the left image
void saveLevelLeft(int level, int i_offset, int subImagesCount, float scale, int* leftKeypointsX, int* leftKeypointsY, int* leftKeypointsAngle, int* leftKeypointsLevel, int& stereoLeftIndex, orb_descriptor* left_descriptors, int vLapping0, int vLapping1, int& vMono) {
    for (int i = i_offset; i < subImagesCount * subImagesCount + i_offset; i++) {
        for (int j = 0; j < extractor_data_left.points_count[i]; j++) {
            if (extractor_data_left.encodedAngles[i][j] == 0) {
                continue;
            }

            uint32_t offsetX = ((i - i_offset) / subImagesCount) * VLEN;
            uint32_t offsetY = ((i - i_offset) % subImagesCount) * 80 - (((i - i_offset) % subImagesCount > 0) ? HALF_PATCH_SIZE : 0);
            uint16_t rectifiedKeypointPos = (extractor_data_left.points[i][j]) - VLEN;

            int leftKeypointX = ((rectifiedKeypointPos % (VLEN * 2)) + offsetX) * scale;

            if (leftKeypointX < vLapping0 || leftKeypointX > vLapping1) {
                // MONO
                leftKeypointsX[vMono] = leftKeypointX;
                leftKeypointsY[vMono] = ((rectifiedKeypointPos / (VLEN * 2)) + offsetY) * scale;
                leftKeypointsAngle[vMono] = extractor_data_left.encodedAngles[i][j];
                leftKeypointsLevel[vMono] = level;

                uint16_t xj = j % (VLEN / 2);
                uint16_t yj = j / (VLEN / 2);

                for (int k = 0; k < 16; k++) {
                    left_descriptors[vMono].ui16[k] = extractor_data_left.orb_descriptors[i][yj].descriptors_lines[k].descriptor[xj];
                }
                vMono++;
            } else {
                // STEREO
                leftKeypointsX[stereoLeftIndex] = leftKeypointX;
                leftKeypointsY[stereoLeftIndex] = ((rectifiedKeypointPos / (VLEN * 2)) + offsetY) * scale;
                leftKeypointsAngle[stereoLeftIndex] = extractor_data_left.encodedAngles[i][j];
                leftKeypointsLevel[stereoLeftIndex] = level;

                uint16_t xj = j % (VLEN / 2);
                uint16_t yj = j / (VLEN / 2);

                for (int k = 0; k < 16; k++) {
                    left_descriptors[stereoLeftIndex].ui16[k] = extractor_data_left.orb_descriptors[i][yj].descriptors_lines[k].descriptor[xj];
                }

                stereoLeftIndex--;
            }
        }
    }
}

/// @brief This method will gather all the data found by the extraction threads in subimages of the right pyramid and put them into the correct structures for the matching step (and the final output). It will also separate the mono and stereo points for more efficient matching.
/// @param level Pyramid level
/// @param i_offset Offset of the subimages for the current level
/// @param subImagesCount Count of subimages for the current level
/// @param scale Scale of the pyramid level
/// @param rightKeypointsX X Positions of the keypoints for the right image
/// @param rightKeypointsY Y Positions of the keypoints for the right image
/// @param rightKeypointsAngle Angles of the keypoints for the right image
/// @param rightKeypointsLevel Pyramid level of the keypoints for the right image
/// @param stereoRightIndex Index of the stereo keypoints for the right image
/// @param right_descriptors ORB descriptors of the right image
/// @param vLapping0 Lapping area 0 for the right image
/// @param vLapping1 Lapping area 1 for the right image
/// @param vMono Mono keypoints index for the right image
void saveLevelRight(int level, int i_offset, int subImagesCount, float scale, int* rightKeypointsX, int* rightKeypointsY, int* rightKeypointsAngle, int* rightKeypointsLevel, int& stereoRightIndex, orb_descriptor* right_descriptors, int vLapping0, int vLapping1, int& vMono) {
    for (int i = i_offset; i < subImagesCount * subImagesCount + i_offset; i++) {
        for (int j = 0; j < extractor_data_right.points_count[i]; j++) {
            if (extractor_data_right.encodedAngles[i][j] == 0) {
                continue;
            }

            uint32_t offsetX = ((i - i_offset) / subImagesCount) * VLEN;
            uint32_t offsetY = ((i - i_offset) % subImagesCount) * 80 - (((i - i_offset) % subImagesCount > 0) ? HALF_PATCH_SIZE : 0);
            uint16_t rectifiedKeypointPos = (extractor_data_right.points[i][j]) - VLEN;

            int rightKeypointX = ((rectifiedKeypointPos % (VLEN * 2)) + offsetX) * scale;

            if (rightKeypointX < vLapping0 || rightKeypointX > vLapping1) {
                // MONO
                rightKeypointsX[vMono] = rightKeypointX;
                rightKeypointsY[vMono] = ((rectifiedKeypointPos / (VLEN * 2)) + offsetY) * scale;
                rightKeypointsAngle[vMono] = extractor_data_right.encodedAngles[i][j];
                rightKeypointsLevel[vMono] = level;

                uint16_t xj = j % (VLEN / 2);
                uint16_t yj = j / (VLEN / 2);

                for (int k = 0; k < 16; k++) {
                    right_descriptors[vMono].ui16[k] = extractor_data_right.orb_descriptors[i][yj].descriptors_lines[k].descriptor[xj];
                }
                vMono++;
            } else {
                // STEREO
                rightKeypointsX[stereoRightIndex] = rightKeypointX;
                rightKeypointsY[stereoRightIndex] = ((rectifiedKeypointPos / (VLEN * 2)) + offsetY) * scale;
                rightKeypointsAngle[stereoRightIndex] = extractor_data_right.encodedAngles[i][j];
                rightKeypointsLevel[stereoRightIndex] = level;

                uint16_t xj = j % (VLEN / 2);
                uint16_t yj = j / (VLEN / 2);

                bool isNull = true;
                for (int k = 0; k < 16; k++) {
                    right_descriptors[stereoRightIndex].ui16[k] = extractor_data_right.orb_descriptors[i][yj].descriptors_lines[k].descriptor[xj];
                    if (right_descriptors[stereoRightIndex].ui16[k] != 0) {
                        isNull = false;
                    }
                }

                if (isNull) {
                    FARF(HIGH, "Bugged Point Null Descriptor (right) (ID: %d) : %d %d %d %d %d %d/%d\n", debugOffsetCounter, rightKeypointX, rightKeypointsY[stereoRightIndex], extractor_data_right.encodedAngles[i][j], level, i, j, extractor_data_right.points_count[i]);
                }

                stereoRightIndex--;
            }
        }
    }
}

/// @brief Deprecated method to match the ORB descriptors of the left and right images using data from the CPU (Not used anymore)
/// @param h Remote handle (unused)
/// @param leftDescriptors ORB descriptors of the left image
/// @param leftDescriptorsLen Length of the left descriptors
/// @param rightDescriptors ORB descriptors of the right image
/// @param rightDescriptorsLen Length of the right descriptors
/// @param indices Matching indices (The most corresponding points)
/// @param indicesLen Matching indices max length
/// @param distances1 Matching distances 1 (Distance to the most corresponding points)
/// @param distances1Len Matching distances 1 max length
/// @param distances2 Matching distances 2 (Distance to the second most corresponding points)
/// @param distances2Len Matching distances 2 max length
/// @return 0 if success
int orbslam3_bfMatchStereo(remote_handle64 h, const unsigned char* leftDescriptors, int leftDescriptorsLen, const unsigned char* rightDescriptors, int rightDescriptorsLen, short* indices, int indicesLen, short* distances1, int distances1Len, short* distances2, int distances2Len) {
    int leftDescriptorCount = leftDescriptorsLen / 32;
    int rightDescriptorCount = rightDescriptorsLen / 32;

    prepareORBDescriptorsVector(_leftDescriptorsVecsMatching1, leftDescriptorCount, (orb_descriptor*)leftDescriptors);
    prepareORBDescriptorsVector(_rightDescriptorsVecsMatching1, rightDescriptorCount, (orb_descriptor*)rightDescriptors);

    knnMatchORB(_leftDescriptorsVecsMatching1, _rightDescriptorsVecsMatching1, leftDescriptorCount, rightDescriptorCount, (uint16_t*)indices, (uint16_t*)distances1, (uint16_t*)distances2, _matchingTemp1, _matchingTemp2, _matchingTemp3, _matchingTemp4);

    return 0;
}

/// @brief Method to match the ORB descriptors of the left and right images using data from the DSP (Used for the final matching step) (First of the two threads)
/// @param leftDescriptors ORB descriptors of the left image
/// @param leftDescriptorsLen Length of the left descriptors
/// @param rightDescriptors ORB descriptors of the right image
/// @param rightDescriptorsLen Length of the right descriptors
/// @param indices Matching indices (The most corresponding points)
/// @param indicesLen Matching indices max length
/// @param distances1 Matching distances 1 (Distance to the most corresponding points)
/// @param distances1Len Matching distances 1 max length
/// @param distances2 Matching distances 2 (Distance to the second most corresponding points)
/// @param distances2Len Matching distances 2 max length
/// @return 0 if success
int bfMatchStereoLocal1(const unsigned char* leftDescriptors, int leftDescriptorsLen, const unsigned char* rightDescriptors, int rightDescriptorsLen, short* indices, int indicesLen, short* distances1, int distances1Len, short* distances2, int distances2Len) {
    int leftDescriptorCount = leftDescriptorsLen / 32;
    int rightDescriptorCount = rightDescriptorsLen / 32;

    prepareORBDescriptorsVector(_leftDescriptorsVecsMatching1, leftDescriptorCount, (orb_descriptor*)leftDescriptors);
    prepareORBDescriptorsVector(_rightDescriptorsVecsMatching1, rightDescriptorCount, (orb_descriptor*)rightDescriptors);

    knnMatchORB(_leftDescriptorsVecsMatching1, _rightDescriptorsVecsMatching1, leftDescriptorCount, rightDescriptorCount, (uint16_t*)indices, (uint16_t*)distances1, (uint16_t*)distances2, _matchingTemp1, _matchingTemp2, _matchingTemp3, _matchingTemp4);

    return 0;
}

/// @brief Method to match the ORB descriptors of the left and right images using data from the DSP (Used for the final matching step) (Second of the two threads)
/// @param leftDescriptors ORB descriptors of the left image
/// @param leftDescriptorsLen Length of the left descriptors
/// @param rightDescriptors ORB descriptors of the right image
/// @param rightDescriptorsLen Length of the right descriptors
/// @param indices Matching indices (The most corresponding points)
/// @param indicesLen Matching indices max length
/// @param distances1 Matching distances 1 (Distance to the most corresponding points)
/// @param distances1Len Matching distances 1 max length
/// @param distances2 Matching distances 2 (Distance to the second most corresponding points)
/// @param distances2Len Matching distances 2 max length
/// @return 0 if success
int bfMatchStereoLocal2(const unsigned char* leftDescriptors, int leftDescriptorsLen, const unsigned char* rightDescriptors, int rightDescriptorsLen, short* indices, int indicesLen, short* distances1, int distances1Len, short* distances2, int distances2Len) {
    int leftDescriptorCount = leftDescriptorsLen / 32;
    int rightDescriptorCount = rightDescriptorsLen / 32;

    prepareORBDescriptorsVector(_leftDescriptorsVecsMatching2, leftDescriptorCount, (orb_descriptor*)leftDescriptors);
    prepareORBDescriptorsVector(_rightDescriptorsVecsMatching2, rightDescriptorCount, (orb_descriptor*)rightDescriptors);

    knnMatchORB(_leftDescriptorsVecsMatching2, _rightDescriptorsVecsMatching2, leftDescriptorCount, rightDescriptorCount, (uint16_t*)indices, (uint16_t*)distances1, (uint16_t*)distances2, _matchingTemp5, _matchingTemp6, _matchingTemp7, _matchingTemp8);

    return 0;
}

/// @brief Extract ORB Features Method (left or right image) (One of the two threads)
/// @param data Extractor Data Struct for the extraction
/// @return 0 if success
void extractORB(extractor_job_data* data) {
    debugOffsetCounter++;

    // Copy VTCM pointers based on left/right image
    if (data->is_right) {
        copyVTCMPtrRight();
    } else {
        copyVTCMPtrLeft();
    }

    pyramid_t* pyramid = data->pyramid;

    // Calculate the pyramid using bilinear interpolation (right or left)
    if (data->is_right) {
        // Copy the input image to the pyramid (right half)
        calculate_pyramid_image_copy_hvx(inputImages + INPUT_WIDTH * sizeof(uint8_t), pyramid->outputLevel0, INPUT_WIDTH * 1, INPUT_WIDTH * 2, INPUT_HEIGHT, INPUT_WIDTH);
    } else {
        calculate_pyramid_image_copy_hvx(inputImages, pyramid->outputLevel0, INPUT_WIDTH * 1, INPUT_WIDTH * 2, INPUT_HEIGHT, INPUT_WIDTH);
    }

    // Start feature extraction: HWA extracts features asynchronously while DSP computes next pyramid levels
    hwa_pipeline_process_feature_extraction_async(hwa_pipeline_context, pyramid->outputLevel0, data->points, data->scores, 0, 0, data->is_right, INPUT_WIDTH, INPUT_HEIGHT, INPUT_WIDTH, data->points_count, MAXIMUM_BEGINNING_FEATURES_PER_SUBIMAGE, debugOffsetCounter);

    // LEVEL 1
    calculate_pyramid_image_bilinear_hvx(pyramid->outputLevel0, pyramid->outputLevel1, INPUT_WIDTH, INPUT_HEIGHT, INPUT_WIDTH, LEVEL1_WIDTH, LEVEL1_HEIGHT, LEVEL1_STRIDE, data->imgPyramidCache, 0);
    hwa_pipeline_process_feature_extraction_async(hwa_pipeline_context, pyramid->outputLevel1, &data->points[LEVEL1_SUBIMAGE_OFFSET], &data->scores[LEVEL1_SUBIMAGE_OFFSET], 1, 1, data->is_right, LEVEL1_WIDTH, LEVEL1_HEIGHT, LEVEL1_STRIDE, &data->points_count[LEVEL1_SUBIMAGE_OFFSET], MAXIMUM_BEGINNING_FEATURES_PER_SUBIMAGE, debugOffsetCounter);

    // LEVEL 2
    calculate_pyramid_image_bilinear_hvx(pyramid->outputLevel1, pyramid->outputLevel2, LEVEL1_WIDTH, LEVEL1_HEIGHT, LEVEL1_STRIDE, LEVEL2_WIDTH, LEVEL2_HEIGHT, LEVEL2_STRIDE, data->imgPyramidCache, 1);
    hwa_pipeline_process_feature_extraction_async(hwa_pipeline_context, pyramid->outputLevel2, &data->points[LEVEL2_SUBIMAGE_OFFSET], &data->scores[LEVEL2_SUBIMAGE_OFFSET], 2, 2, data->is_right, LEVEL2_WIDTH, LEVEL2_HEIGHT, LEVEL2_STRIDE, &data->points_count[LEVEL2_SUBIMAGE_OFFSET], MAXIMUM_BEGINNING_FEATURES_PER_SUBIMAGE, debugOffsetCounter);

    // LEVEL 3
    calculate_pyramid_image_bilinear_hvx(pyramid->outputLevel2, pyramid->outputLevel3, LEVEL2_WIDTH, LEVEL2_HEIGHT, LEVEL2_STRIDE, LEVEL3_WIDTH, LEVEL3_HEIGHT, LEVEL3_STRIDE, data->imgPyramidCache, 2);
    hwa_pipeline_process_feature_extraction_async(hwa_pipeline_context, pyramid->outputLevel3, &data->points[LEVEL3_SUBIMAGE_OFFSET], &data->scores[LEVEL3_SUBIMAGE_OFFSET], 2, 3, data->is_right, LEVEL3_WIDTH, LEVEL3_HEIGHT, LEVEL3_STRIDE, &data->points_count[LEVEL3_SUBIMAGE_OFFSET], MAXIMUM_BEGINNING_FEATURES_PER_SUBIMAGE, debugOffsetCounter);

    // LEVEL 4
    calculate_pyramid_image_bilinear_hvx(pyramid->outputLevel3, pyramid->outputLevel4, LEVEL3_WIDTH, LEVEL3_HEIGHT, LEVEL3_STRIDE, LEVEL4_WIDTH, LEVEL4_HEIGHT, LEVEL4_STRIDE, data->imgPyramidCache, 3);
    hwa_pipeline_process_feature_extraction_async(hwa_pipeline_context, pyramid->outputLevel4, &data->points[LEVEL4_SUBIMAGE_OFFSET], &data->scores[LEVEL4_SUBIMAGE_OFFSET], 3, 4, data->is_right, LEVEL4_WIDTH, LEVEL4_HEIGHT, LEVEL4_STRIDE, &data->points_count[LEVEL4_SUBIMAGE_OFFSET], MAXIMUM_BEGINNING_FEATURES_PER_SUBIMAGE, debugOffsetCounter);

    // LEVEL 5
    calculate_pyramid_image_bilinear_hvx(pyramid->outputLevel4, pyramid->outputLevel5, LEVEL4_WIDTH, LEVEL4_HEIGHT, LEVEL4_STRIDE, LEVEL5_WIDTH, LEVEL5_HEIGHT, LEVEL5_STRIDE, data->imgPyramidCache, 4);
    hwa_pipeline_process_feature_extraction_async(hwa_pipeline_context, pyramid->outputLevel5, &data->points[LEVEL5_SUBIMAGE_OFFSET], &data->scores[LEVEL5_SUBIMAGE_OFFSET], 3, 5, data->is_right, LEVEL5_WIDTH, LEVEL5_HEIGHT, LEVEL5_STRIDE, &data->points_count[LEVEL5_SUBIMAGE_OFFSET], MAXIMUM_BEGINNING_FEATURES_PER_SUBIMAGE, debugOffsetCounter);

    // LEVEL 6
    calculate_pyramid_image_bilinear_hvx(pyramid->outputLevel5, pyramid->outputLevel6, LEVEL5_WIDTH, LEVEL5_HEIGHT, LEVEL5_STRIDE, LEVEL6_WIDTH, LEVEL6_HEIGHT, LEVEL6_STRIDE, data->imgPyramidCache, 5);
    hwa_pipeline_process_feature_extraction_async(hwa_pipeline_context, pyramid->outputLevel6, &data->points[LEVEL6_SUBIMAGE_OFFSET], &data->scores[LEVEL6_SUBIMAGE_OFFSET], 3, 6, data->is_right, LEVEL6_WIDTH, LEVEL6_HEIGHT, LEVEL6_STRIDE, &data->points_count[LEVEL6_SUBIMAGE_OFFSET], MAXIMUM_BEGINNING_FEATURES_PER_SUBIMAGE, debugOffsetCounter);

    // DSP starts computing descriptors while HWA finishes last levels
    // LEVEL 0
    hwa_pipeline_process_feature_extraction_wait_finish(hwa_pipeline_context, 0, data->is_right);
    ic_angle_simd(data->pyramid->outputLevel0, data->points, umax, data->encodedAngles, INPUT_HEIGHT, INPUT_WIDTH, INPUT_WIDTH, data->imgPyramidCache, data->points_count);
    calculate_orb_descriptors_optimized16(*pattern_vecs, data->pyramid->outputLevel0, data->points, data->orb_descriptors, data->encodedAngles, INPUT_HEIGHT, INPUT_WIDTH, INPUT_WIDTH, data->imgPyramidCache, data->points_count);

    // LEVEL 1
    hwa_pipeline_process_feature_extraction_wait_finish(hwa_pipeline_context, 1, data->is_right);
    ic_angle_simd(data->pyramid->outputLevel1, &data->points[LEVEL1_SUBIMAGE_OFFSET], umax, &data->encodedAngles[LEVEL1_SUBIMAGE_OFFSET], LEVEL1_HEIGHT, LEVEL1_WIDTH, LEVEL1_STRIDE, data->imgPyramidCache, &data->points_count[LEVEL1_SUBIMAGE_OFFSET]);
    calculate_orb_descriptors_optimized16(*pattern_vecs, data->pyramid->outputLevel1, &data->points[LEVEL1_SUBIMAGE_OFFSET], &data->orb_descriptors[LEVEL1_SUBIMAGE_OFFSET], &data->encodedAngles[LEVEL1_SUBIMAGE_OFFSET], LEVEL1_HEIGHT, LEVEL1_WIDTH, LEVEL1_STRIDE, data->imgPyramidCache, &data->points_count[LEVEL1_SUBIMAGE_OFFSET]);

    // LEVEL 2
    hwa_pipeline_process_feature_extraction_wait_finish(hwa_pipeline_context, 2, data->is_right);
    ic_angle_simd(data->pyramid->outputLevel2, &data->points[LEVEL2_SUBIMAGE_OFFSET], umax, &data->encodedAngles[LEVEL2_SUBIMAGE_OFFSET], LEVEL2_HEIGHT, LEVEL2_WIDTH, LEVEL2_STRIDE, data->imgPyramidCache, &data->points_count[LEVEL2_SUBIMAGE_OFFSET]);
    calculate_orb_descriptors_optimized16(*pattern_vecs, data->pyramid->outputLevel2, &data->points[LEVEL2_SUBIMAGE_OFFSET], &data->orb_descriptors[LEVEL2_SUBIMAGE_OFFSET], &data->encodedAngles[LEVEL2_SUBIMAGE_OFFSET], LEVEL2_HEIGHT, LEVEL2_WIDTH, LEVEL2_STRIDE, data->imgPyramidCache, &data->points_count[LEVEL2_SUBIMAGE_OFFSET]);

    // LEVEL 3
    hwa_pipeline_process_feature_extraction_wait_finish(hwa_pipeline_context, 3, data->is_right);
    ic_angle_simd(data->pyramid->outputLevel3, &data->points[LEVEL3_SUBIMAGE_OFFSET], umax, &data->encodedAngles[LEVEL3_SUBIMAGE_OFFSET], LEVEL3_HEIGHT, LEVEL3_WIDTH, LEVEL3_STRIDE, data->imgPyramidCache, &data->points_count[LEVEL3_SUBIMAGE_OFFSET]);
    calculate_orb_descriptors_optimized16(*pattern_vecs, data->pyramid->outputLevel3, &data->points[LEVEL3_SUBIMAGE_OFFSET], &data->orb_descriptors[LEVEL3_SUBIMAGE_OFFSET], &data->encodedAngles[LEVEL3_SUBIMAGE_OFFSET], LEVEL3_HEIGHT, LEVEL3_WIDTH, LEVEL3_STRIDE, data->imgPyramidCache, &data->points_count[LEVEL3_SUBIMAGE_OFFSET]);

    // LEVEL 4
    hwa_pipeline_process_feature_extraction_wait_finish(hwa_pipeline_context, 4, data->is_right);
    ic_angle_simd(data->pyramid->outputLevel4, &data->points[LEVEL4_SUBIMAGE_OFFSET], umax, &data->encodedAngles[LEVEL4_SUBIMAGE_OFFSET], LEVEL4_HEIGHT, LEVEL4_WIDTH, LEVEL4_STRIDE, data->imgPyramidCache, &data->points_count[LEVEL4_SUBIMAGE_OFFSET]);
    calculate_orb_descriptors_optimized16(*pattern_vecs, data->pyramid->outputLevel4, &data->points[LEVEL4_SUBIMAGE_OFFSET], &data->orb_descriptors[LEVEL4_SUBIMAGE_OFFSET], &data->encodedAngles[LEVEL4_SUBIMAGE_OFFSET], LEVEL4_HEIGHT, LEVEL4_WIDTH, LEVEL4_STRIDE, data->imgPyramidCache, &data->points_count[LEVEL4_SUBIMAGE_OFFSET]);

    // LEVEL 5
    hwa_pipeline_process_feature_extraction_wait_finish(hwa_pipeline_context, 5, data->is_right);
    ic_angle_simd(data->pyramid->outputLevel5, &data->points[LEVEL5_SUBIMAGE_OFFSET], umax, &data->encodedAngles[LEVEL5_SUBIMAGE_OFFSET], LEVEL5_HEIGHT, LEVEL5_WIDTH, LEVEL5_STRIDE, data->imgPyramidCache, &data->points_count[LEVEL5_SUBIMAGE_OFFSET]);
    calculate_orb_descriptors_optimized16(*pattern_vecs, data->pyramid->outputLevel5, &data->points[LEVEL5_SUBIMAGE_OFFSET], &data->orb_descriptors[LEVEL5_SUBIMAGE_OFFSET], &data->encodedAngles[LEVEL5_SUBIMAGE_OFFSET], LEVEL5_HEIGHT, LEVEL5_WIDTH, LEVEL5_STRIDE, data->imgPyramidCache, &data->points_count[LEVEL5_SUBIMAGE_OFFSET]);

    // LEVEL 6
    hwa_pipeline_process_feature_extraction_wait_finish(hwa_pipeline_context, 6, data->is_right);
    ic_angle_simd(data->pyramid->outputLevel6, &data->points[LEVEL6_SUBIMAGE_OFFSET], umax, &data->encodedAngles[LEVEL6_SUBIMAGE_OFFSET], LEVEL6_HEIGHT, LEVEL6_WIDTH, LEVEL6_STRIDE, data->imgPyramidCache, &data->points_count[LEVEL6_SUBIMAGE_OFFSET]);
    calculate_orb_descriptors_optimized16(*pattern_vecs, data->pyramid->outputLevel6, &data->points[LEVEL6_SUBIMAGE_OFFSET], &data->orb_descriptors[LEVEL6_SUBIMAGE_OFFSET], &data->encodedAngles[LEVEL6_SUBIMAGE_OFFSET], LEVEL6_HEIGHT, LEVEL6_WIDTH, LEVEL6_STRIDE, data->imgPyramidCache, &data->points_count[LEVEL6_SUBIMAGE_OFFSET]);

    // Separate right and left case for saving data and matching
    if (!data->is_right) {
        // LEFT IMAGE

        // Count total features
        int total_features_left = 0;
        for (int i = 0; i < LEVEL7_SUBIMAGE_OFFSET; i++) { // 72, LEVEL7_SUBIMAGE_OFFSET
            total_features_left += extractor_data_left.points_count[i];
        }

        // INDEX FOR STEREO AND MONO counting
        int stereoLeftIndex = total_features_left - 1;
        int monoLeftIndex = 0;

        orb_descriptor* left_descriptors = (orb_descriptor*)_leftKeypointsORB;

        // LEFT LEVELS - SAVE DATA FOR MATCHING AND OUTPUT + MONO/STEREO SEPARATION
        saveLevelLeft(0, LEVEL0_SUBIMAGE_OFFSET, 5, 1.0f, _leftKeypointsX, _leftKeypointsY, _leftKeypointsAngle, _leftKeypointsLevel, stereoLeftIndex, left_descriptors, _vLappingAreaLeft0, _vLappingAreaLeft1, monoLeftIndex);
        saveLevelLeft(1, LEVEL1_SUBIMAGE_OFFSET, 4, 5.0f / 4.0f, _leftKeypointsX, _leftKeypointsY, _leftKeypointsAngle, _leftKeypointsLevel, stereoLeftIndex, left_descriptors, _vLappingAreaLeft0, _vLappingAreaLeft1, monoLeftIndex);
        saveLevelLeft(2, LEVEL2_SUBIMAGE_OFFSET, 3, 5.0f / 3.0f, _leftKeypointsX, _leftKeypointsY, _leftKeypointsAngle, _leftKeypointsLevel, stereoLeftIndex, left_descriptors, _vLappingAreaLeft0, _vLappingAreaLeft1, monoLeftIndex);
        saveLevelLeft(3, LEVEL3_SUBIMAGE_OFFSET, 3, 5.0f / 2.44948974278f, _leftKeypointsX, _leftKeypointsY, _leftKeypointsAngle, _leftKeypointsLevel, stereoLeftIndex, left_descriptors, _vLappingAreaLeft0, _vLappingAreaLeft1, monoLeftIndex);
        saveLevelLeft(4, LEVEL4_SUBIMAGE_OFFSET, 2, 5.0f / 2.0f, _leftKeypointsX, _leftKeypointsY, _leftKeypointsAngle, _leftKeypointsLevel, stereoLeftIndex, left_descriptors, _vLappingAreaLeft0, _vLappingAreaLeft1, monoLeftIndex);
        saveLevelLeft(5, LEVEL5_SUBIMAGE_OFFSET, 2, 5.0f / 1.58740105197f, _leftKeypointsX, _leftKeypointsY, _leftKeypointsAngle, _leftKeypointsLevel, stereoLeftIndex, left_descriptors, _vLappingAreaLeft0, _vLappingAreaLeft1, monoLeftIndex);
        saveLevelLeft(6, LEVEL6_SUBIMAGE_OFFSET, 2, 5.0f / 1.25992104989f, _leftKeypointsX, _leftKeypointsY, _leftKeypointsAngle, _leftKeypointsLevel, stereoLeftIndex, left_descriptors, _vLappingAreaLeft0, _vLappingAreaLeft1, monoLeftIndex);
        // saveLevelLeft(7, LEVEL7_SUBIMAGE_OFFSET, 1, 5.0f / 1.0f, _leftKeypointsX, _leftKeypointsY, _leftKeypointsAngle, _leftKeypointsLevel, stereoLeftIndex, left_descriptors, _vLappingAreaLeft0, _vLappingAreaLeft1, monoLeftIndex);

        // Save the total points and the mono/stereo separation index
        *_totalPointsLeft = total_features_left;
        *_monoLeft = monoLeftIndex;

        // Wait for the other thread to finish before starting matching
        qurt_barrier_wait(&extractor_to_matcher_barrier);

        // Calculate the args for the stereo matching (half of the right descriptors with all the left descriptors, and round to VLEN/2)
        int stereoPoints = *_totalPointsLeft - *_monoLeft;
        int totalStereoVecs = stereoPoints / (VLEN / 2);

        int firstHalfStereoVecs = totalStereoVecs / 2;
        int firstHalfStereoPoints = firstHalfStereoVecs * (VLEN / 2);
        int secondHalfStereoPoints = stereoPoints - firstHalfStereoPoints;

        // Match half of the right descriptors with all the left descriptors (note : right and left may be inverted)
        bfMatchStereoLocal1(&_rightKeypointsORB[(*_monoRight) * sizeof(orb_descriptor)], ((*_totalPointsRight - *_monoRight) / 2) * sizeof(orb_descriptor),
                            &_leftKeypointsORB[(*_monoLeft) * sizeof(orb_descriptor)], (firstHalfStereoPoints) * sizeof(orb_descriptor),
                            _indices, _indicesLen, _distances1, _distances1Len, _distances2, _distances2Len);
    } else {
        // RIGHT IMAGE

        // Count total features
        int total_features_right = 0;
        for (int i = 0; i < LEVEL7_SUBIMAGE_OFFSET; i++) { // 72, LEVEL7_SUBIMAGE_OFFSET
            total_features_right += extractor_data_right.points_count[i];
        }

        // INDEX FOR STEREO AND MONO counting
        int stereoRightIndex = total_features_right - 1;
        int monoRightIndex = 0;

        orb_descriptor* right_descriptors = (orb_descriptor*)_rightKeypointsORB;

        // RIGHT LEVELS - SAVE DATA FOR MATCHING AND OUTPUT + MONO/STEREO SEPARATION
        saveLevelRight(0, LEVEL0_SUBIMAGE_OFFSET, 5, 1.0f, _rightKeypointsX, _rightKeypointsY, _rightKeypointsAngle, _rightKeypointsLevel, stereoRightIndex, right_descriptors, _vLappingAreaRight0, _vLappingAreaRight1, monoRightIndex);
        saveLevelRight(1, LEVEL1_SUBIMAGE_OFFSET, 4, 5.0f / 4.0f, _rightKeypointsX, _rightKeypointsY, _rightKeypointsAngle, _rightKeypointsLevel, stereoRightIndex, right_descriptors, _vLappingAreaRight0, _vLappingAreaRight1, monoRightIndex);
        saveLevelRight(2, LEVEL2_SUBIMAGE_OFFSET, 3, 5.0f / 3.0f, _rightKeypointsX, _rightKeypointsY, _rightKeypointsAngle, _rightKeypointsLevel, stereoRightIndex, right_descriptors, _vLappingAreaRight0, _vLappingAreaRight1, monoRightIndex);
        saveLevelRight(3, LEVEL3_SUBIMAGE_OFFSET, 3, 5.0f / 2.44948974278f, _rightKeypointsX, _rightKeypointsY, _rightKeypointsAngle, _rightKeypointsLevel, stereoRightIndex, right_descriptors, _vLappingAreaRight0, _vLappingAreaRight1, monoRightIndex);
        saveLevelRight(4, LEVEL4_SUBIMAGE_OFFSET, 2, 5.0f / 2.0f, _rightKeypointsX, _rightKeypointsY, _rightKeypointsAngle, _rightKeypointsLevel, stereoRightIndex, right_descriptors, _vLappingAreaRight0, _vLappingAreaRight1, monoRightIndex);
        saveLevelRight(5, LEVEL5_SUBIMAGE_OFFSET, 2, 5.0f / 1.58740105197f, _rightKeypointsX, _rightKeypointsY, _rightKeypointsAngle, _rightKeypointsLevel, stereoRightIndex, right_descriptors, _vLappingAreaRight0, _vLappingAreaRight1, monoRightIndex);
        saveLevelRight(6, LEVEL6_SUBIMAGE_OFFSET, 2, 5.0f / 1.25992104989f, _rightKeypointsX, _rightKeypointsY, _rightKeypointsAngle, _rightKeypointsLevel, stereoRightIndex, right_descriptors, _vLappingAreaRight0, _vLappingAreaRight1, monoRightIndex);
        // saveLevelRight(7, LEVEL7_SUBIMAGE_OFFSET, 1, 5.0f / 1.0f, _rightKeypointsX, _rightKeypointsY, _rightKeypointsAngle, _rightKeypointsLevel, stereoRightIndex, right_descriptors, _vLappingAreaRight0, _vLappingAreaRight1, monoRightIndex);

        // Save the total points and the mono/stereo separation index
        *_totalPointsRight = total_features_right;
        *_monoRight = monoRightIndex;

        // Wait for the other thread to finish before starting matching
        qurt_barrier_wait(&extractor_to_matcher_barrier);

        // Calculate the args for the stereo matching (half of the right descriptors with all the left descriptors, and round to VLEN/2)
        int stereoPoints = *_totalPointsLeft - *_monoLeft;
        int totalStereoVecs = stereoPoints / (VLEN / 2);
        int firstHalfStereoVecs = totalStereoVecs / 2;
        int firstHalfStereoPoints = firstHalfStereoVecs * (VLEN / 2);
        int secondHalfStereoPoints = stereoPoints - firstHalfStereoPoints;

        // Match half of the right descriptors with all the left descriptors (note : right and left may be inverted)
        bfMatchStereoLocal2(&_rightKeypointsORB[(*_monoRight) * sizeof(orb_descriptor)], (*_totalPointsRight - *_monoRight) * sizeof(orb_descriptor),
                            &_leftKeypointsORB[(*_monoLeft + firstHalfStereoPoints) * sizeof(orb_descriptor)], secondHalfStereoPoints * sizeof(orb_descriptor),
                            &_indices[firstHalfStereoPoints], _indicesLen, &_distances1[firstHalfStereoPoints], _distances1Len, &_distances2[firstHalfStereoPoints], _distances2Len);
    }
}

/// @brief Always active left extractor worker thread
/// @param args unused (we use global variables instead)
void workerExtractorThreadLeft(void* args) {
    // Created once by open and destroyed by close
    while (!stop_threads) {
        // Wait for the signal to start extractor job
        qurt_mutex_lock(&extractor_mutex);
        while (!extractor_left_ready && !stop_threads) {
            qurt_cond_wait(&extractor_left_cond, &extractor_mutex);
        }
        qurt_mutex_unlock(&extractor_mutex);
        if (stop_threads) {
            break;
        }

        // Call the extractor method
        extractORB(&extractor_data_left);

        // Signal that the job is done
        qurt_mutex_lock(&extractor_mutex);
        extractor_left_ready = false;
        qurt_mutex_unlock(&extractor_mutex);

        qurt_cond_signal(&extractor_left_join_cond);
    }

    qurt_thread_exit(QURT_EOK);
}

/// @brief Always active right extractor worker thread
/// @param args unused (we use global variables instead)
void workerExtractorThreadRight(void* args) {
    // Created once by open and destroyed by close
    while (!stop_threads) {
        // Wait for the signal to start extractor job
        qurt_mutex_lock(&extractor_mutex);
        while (!extractor_right_ready && !stop_threads) {
            qurt_cond_wait(&extractor_right_cond, &extractor_mutex);
        }
        qurt_mutex_unlock(&extractor_mutex);
        if (stop_threads) {
            break;
        }

        // Call the extractor method
        extractORB(&extractor_data_right);

        // Signal that the job is done
        qurt_mutex_lock(&extractor_mutex);
        extractor_right_ready = false;
        qurt_mutex_unlock(&extractor_mutex);

        qurt_cond_signal(&extractor_right_join_cond);
    }

    qurt_thread_exit(QURT_EOK);
}

/// @brief Open the FastRPC handle and initialize the program
/// @param uri unused
/// @param handle the returned value (unused)
/// @return AEEResult 0 for success
AEEResult orbslam3_open(const char* uri, remote_handle64* handle) {
    int* tptr = NULL;

    prepare(); // Init the program, do memory allocations and prepare calculation caches

    // Init worker threads
    qurt_mutex_init(&extractor_mutex);

    // Init condition variables
    qurt_cond_init(&extractor_left_cond);
    qurt_cond_init(&extractor_right_cond);
    qurt_cond_init(&extractor_left_join_cond);
    qurt_cond_init(&extractor_right_join_cond);

    // Init extractor data
    extractor_data_left.is_right = false;
    extractor_data_right.is_right = true;

    // Create worker threads
    qurt_thread_attr_t extractorLeftAttr;
    qurt_thread_attr_t extractorRightAttr;

    // Allocate left stack memory
    thread_stacks[0] = malloc(STACK_SIZE);

    // Create left extractor thread
    qurt_thread_attr_init(&extractorLeftAttr);
    qurt_thread_attr_set_name(&extractorLeftAttr, "orbextL");
    qurt_thread_attr_set_stack_addr(&extractorLeftAttr, thread_stacks[0]);
    qurt_thread_attr_set_stack_size(&extractorLeftAttr, STACK_SIZE);
    qurt_thread_attr_set_priority(&extractorLeftAttr, QURT_THREAD_ATTR_PRIORITY_DEFAULT / 2); // TODO SET HIGH PRIORITY WHEN USED AS MAIN SLAM
    if (qurt_thread_create(&extractor_left_thread, &extractorLeftAttr, workerExtractorThreadLeft, nullptr) != QURT_EOK) {
        FARF(HIGH, "Error creating thread left\n");
    }

    // Allocate right stack memory
    thread_stacks[1] = malloc(STACK_SIZE);

    // Create right extractor thread
    qurt_thread_attr_init(&extractorRightAttr);
    qurt_thread_attr_set_name(&extractorRightAttr, "orbextR");
    qurt_thread_attr_set_stack_addr(&extractorRightAttr, thread_stacks[1]);
    qurt_thread_attr_set_stack_size(&extractorRightAttr, STACK_SIZE);
    qurt_thread_attr_set_priority(&extractorRightAttr, QURT_THREAD_ATTR_PRIORITY_DEFAULT / 2); // TODO SET HIGH PRIORITY WHEN USED AS MAIN SLAM
    if (qurt_thread_create(&extractor_right_thread, &extractorRightAttr, workerExtractorThreadRight, nullptr) != QURT_EOK) {
        FARF(HIGH, "Error creating thread right\n");
    }

    // Init barrier
    qurt_barrier_init(&extractor_to_matcher_barrier, 2);

    // Init HWA
    init_hwa_pipeline(&hwa_pipeline_context, USED_HWA_MODULES, imgPyramidHVX, imgPyramidHVX2);

    // Return FastRPC handle
    tptr = (int*)malloc(sizeof(int));
    *tptr = 1561968176; // random value, unused
    *handle = (remote_handle64)tptr;
    assert(*handle);
    return 0;
}

/// @brief Close the FastRPC handle and free memory
/// @param handle the value returned by open
/// @return AEEResult 0 for success, should always succeed
AEEResult orbslam3_close(remote_handle64 handle) {
    // Destroy worker threads
    stop_threads = true;
    qurt_mutex_lock(&extractor_mutex);
    qurt_cond_signal(&extractor_left_cond);
    qurt_mutex_unlock(&extractor_mutex);

    qurt_mutex_lock(&extractor_mutex);
    qurt_cond_signal(&extractor_right_cond);
    qurt_mutex_unlock(&extractor_mutex);

    // Join threads
    qurt_thread_join(extractor_left_thread, NULL);
    qurt_thread_join(extractor_right_thread, NULL);

    // Free stack memory
    free(thread_stacks[0]);
    free(thread_stacks[1]);

    // Destroy mutex and condition variables, and barrier
    qurt_mutex_destroy(&extractor_mutex);

    qurt_cond_destroy(&extractor_left_cond);
    qurt_cond_destroy(&extractor_right_cond);
    qurt_cond_destroy(&extractor_left_join_cond);
    qurt_cond_destroy(&extractor_right_join_cond);

    qurt_barrier_destroy(&extractor_to_matcher_barrier);

    // Destroy HWA pipeline
    destroy_hwa_pipeline(hwa_pipeline_context);

    // Free memory
    clearMemory();

    // Release FastRPC Handle
    if (handle) {
        free((void*)handle);
    }

    return 0;
}

/// @brief Method called from the CPU to extract features from the image
/// @param h the FastRPC handle (unused but mandatory)
/// @param image the input image (stereo)
/// @param imageLen the byte length of the input image
/// @param width the width of the image (unused)
/// @param height the height of the image (unused)
/// @param stride the stride of the image (unused)
/// @param threshold the threshold for the FAST feature extraction (unused)
/// @param vLappingAreaLeft0 the vertical lapping area for the left image (beginning)
/// @param vLappingAreaLeft1 the vertical lapping area for the left image (end)
/// @param vLappingAreaRight0 the vertical lapping area for the right image (beginning)
/// @param vLappingAreaRight1 the vertical lapping area for the right image (end)
/// @param totalPointsLeft the total amount of features in the left image
/// @param leftKeypointsX the x coordinates of the features in the left image
/// @param leftKeypointsXLen the maximum length of the leftKeypointsX array
/// @param leftKeypointsY the y coordinates of the features in the left image
/// @param leftKeypointsYLen the maximum length of the leftKeypointsY array
/// @param leftKeypointsAngle the angles (encoded cos/sin) of the features in the left image
/// @param leftKeypointsAngleLen the maximum length of the leftKeypointsAngle array
/// @param leftKeypointsLevel the levels of the features in the left image
/// @param leftKeypointsLevelLen the maximum length of the leftKeypointsLevel array
/// @param leftKeypointsORB the descriptors of the features in the left image
/// @param leftKeypointsORBLen the maximum length of the leftKeypointsORB array
/// @param totalPointsRight the total amount of features in the right image
/// @param rightKeypointsX the x coordinates of the features in the right image
/// @param rightKeypointsXLen the maximum length of the rightKeypointsX array
/// @param rightKeypointsY the y coordinates of the features in the right image
/// @param rightKeypointsYLen the maximum length of the rightKeypointsY array
/// @param rightKeypointsAngle the angles (encoded cos/sin) of the features in the right image
/// @param rightKeypointsAngleLen the maximum length of the rightKeypointsAngle array
/// @param rightKeypointsLevel the levels of the features in the right image
/// @param rightKeypointsLevelLen the maximum length of the rightKeypointsLevel array
/// @param rightKeypointsORB the descriptors of the features in the right image
/// @param rightKeypointsORBLen the maximum length of the rightKeypointsORB array
/// @param monoLeft the mono feature count in the left image
/// @param monoRight the mono feature count in the right image
/// @param indices the indices of the matched features
/// @param indicesLen the maximum length of the indices array
/// @param distances1 the first distances of the matched features
/// @param distances1Len the maximum length of the distances1 array
/// @param distances2 the second distances of the matched features
/// @param distances2Len the maximum length of the distances2 array
/// @return 0 for success, 1 for error
int orbslam3_extractFeatures(remote_handle64 h, const unsigned char* image, int imageLen, int width, int height, int stride, int threshold, int vLappingAreaLeft0, int vLappingAreaLeft1, int vLappingAreaRight0, int vLappingAreaRight1,
                             int* totalPointsLeft, int* leftKeypointsX, int leftKeypointsXLen, int* leftKeypointsY, int leftKeypointsYLen, int* leftKeypointsAngle, int leftKeypointsAngleLen, int* leftKeypointsLevel, int leftKeypointsLevelLen, uint8_t* leftKeypointsORB, int leftKeypointsORBLen,
                             int* totalPointsRight, int* rightKeypointsX, int rightKeypointsXLen, int* rightKeypointsY, int rightKeypointsYLen, int* rightKeypointsAngle, int rightKeypointsAngleLen, int* rightKeypointsLevel, int rightKeypointsLevelLen, uint8_t* rightKeypointsORB, int rightKeypointsORBLen,
                             int* monoLeft, int* monoRight,
                             short* indices, int indicesLen, short* distances1, int distances1Len, short* distances2, int distances2Len) {
    // Copy the input data to global variables for use in the worker threads
    _vLappingAreaLeft0 = vLappingAreaLeft0;
    _vLappingAreaLeft1 = vLappingAreaLeft1;
    _vLappingAreaRight0 = vLappingAreaRight0;
    _vLappingAreaRight1 = vLappingAreaRight1;
    _totalPointsLeft = totalPointsLeft;
    _totalPointsRight = totalPointsRight;
    _monoLeft = monoLeft;
    _monoRight = monoRight;
    _leftKeypointsX = leftKeypointsX;
    _leftKeypointsY = leftKeypointsY;
    _leftKeypointsAngle = leftKeypointsAngle;
    _leftKeypointsLevel = leftKeypointsLevel;
    _rightKeypointsX = rightKeypointsX;
    _rightKeypointsY = rightKeypointsY;
    _rightKeypointsAngle = rightKeypointsAngle;
    _rightKeypointsLevel = rightKeypointsLevel;
    _leftKeypointsORB = leftKeypointsORB;
    _rightKeypointsORB = rightKeypointsORB;

    _indices = indices;
    _indicesLen = indicesLen;
    _distances1 = distances1;
    _distances1Len = distances1Len;
    _distances2 = distances2;
    _distances2Len = distances2Len;

    // Prepare VTCM Memory for the extractor
    if (prepareVTCM() != 0) {
        FARF(HIGH, "Error preparing VTCM\n");
        return 1;
    }

    // Copy input image to global pointer
    inputImages = (uint8_t*)image;

    // Lock the mutex and signal the worker threads to start
    qurt_mutex_lock(&extractor_mutex);

    // Signal the worker threads to start
    extractor_left_ready = true;
    qurt_cond_signal(&extractor_left_cond);

    extractor_right_ready = true;
    qurt_cond_signal(&extractor_right_cond);

    // Wait for the extractor threads to finish (unlock the mutex while waiting)
    while (extractor_left_ready) {
        qurt_cond_wait(&extractor_left_join_cond, &extractor_mutex);
    }

    while (extractor_right_ready) {
        qurt_cond_wait(&extractor_right_join_cond, &extractor_mutex);
    }

    // Unlock the mutex
    qurt_mutex_unlock(&extractor_mutex);

    // Free VTCM Resource
    if (clearVTCM() != 0) {
        FARF(HIGH, "Error clearing VTCM\n");
        return 7;
    }

    return 0;
}

#else

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "HAP_farf.h"
#include "HAP_vtcm_mgr.h"
#include "HAP_compute_res.h"

#include "AEEStdErr.h"

#include "orbslam3.h"

#include "hvx_internal.h"

#include "orbslam_dsp_tests.h"

#ifndef CALC_EXPORT
#define CALC_EXPORT
#endif /*CALC_EXPORT*/

#if defined(_WIN32) && !defined(_ARM_)
#include "ptl_remote_invoke.h"
CALC_EXPORT int init(pfn_getSymbol GetSymbol) {
    return remote_invoke_stub_init(GetSymbol);
}
#endif

CALC_EXPORT int main(void) {
    FARF(HIGH, "############################################################\n");
    FARF(HIGH, "Hello Hexagon, Starting tests!\n");

    // orbslam3_tests();
    orbslam3_tests_stereo();

    FARF(HIGH, "############################################################\n");

    return 0;
}

int orbslam3_extractFeatures(remote_handle64 h, const unsigned char* image, int imageLen, int width, int height, int stride, int threshold,
                             int* totalPointsLeft, int* leftKeypointsX, int leftKeypointsXLen, int* leftKeypointsY, int leftKeypointsYLen, int* leftKeypointsAngle, int leftKeypointsAngleLen, int* leftKeypointsLevel, int leftKeypointsLevelLen, uint8_t* leftKeypointsORB, int leftKeypointsORBLen,
                             int* totalPointsRight, int* rightKeypointsX, int rightKeypointsXLen, int* rightKeypointsY, int rightKeypointsYLen, int* rightKeypointsAngle, int rightKeypointsAngleLen, int* rightKeypointsLevel, int rightKeypointsLevelLen, uint8_t* rightKeypointsORB, int rightKeypointsORBLen) {
    return 0;
}

int orbslam3_open(const char* uri, remote_handle64* handle) {
    return 0;
}

int orbslam3_close(remote_handle64 handle) {
    return 0;
}

#endif