/**
 * @file orbslam_dsp_pyramid.h
 *
 * @author Gaston Rouquette (Lynx Mixed Reality)
 *
 * @brief Image Pyramid construction algorithm and VTCM custom cache management
**/



#pragma once

#include "HAP_farf.h"
#include "HAP_vtcm_mgr.h"
#include "HAP_compute_res.h"


#include "AEEStdErr.h"

#include "orbslam3.h"
#include "hvx_internal.h"

#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wunused-variable"


// NEW PYRAMID 5 -> 4 -> 3 -> sqrt(6) -> 2 -> 2^(2/3) -> 2^(1/3) -> 1
// ROUNDED to 5 -> 4 -> 3 -> 2.44948974278 -> 2 -> 1.58740105197 -> 1.25992104989 -> 1
/*
    //ROUNDED TO THE CLOSEST
    WIDTH : 640, 512, 384, 314, 256, 203, 161, 128 
    HEIGHT : 400, 320, 240, 196, 160, 127, 101, 80
    STRIDE : 640, 512, 384, 384, 256, 256, 256, 128
*/

//Size of the input image
#define INPUT_WIDTH 640
#define INPUT_HEIGHT 400
#define LEVELS 8

//Size of the pyramid (WIDTH)
#define LEVEL1_WIDTH 512
#define LEVEL2_WIDTH 384
#define LEVEL3_WIDTH 314
#define LEVEL4_WIDTH 256
#define LEVEL5_WIDTH 203
#define LEVEL6_WIDTH 161
#define LEVEL7_WIDTH 128

//Size of the pyramid (HEIGHT)
#define LEVEL1_HEIGHT 320
#define LEVEL2_HEIGHT 240
#define LEVEL3_HEIGHT 196
#define LEVEL4_HEIGHT 160
#define LEVEL5_HEIGHT 127
#define LEVEL6_HEIGHT 101
#define LEVEL7_HEIGHT 80

//Stride of the pyramid (STRIDE = WIDTH ROUNDED TO THE CLOSEST MULTIPLE OF VLEN (128))
#define LEVEL1_STRIDE 512
#define LEVEL2_STRIDE 384
#define LEVEL3_STRIDE 384
#define LEVEL4_STRIDE 256
#define LEVEL5_STRIDE 256
#define LEVEL6_STRIDE 256
#define LEVEL7_STRIDE 128

#define LEVEL0_SUBIMAGE_OFFSET 0
#define LEVEL1_SUBIMAGE_OFFSET 25
#define LEVEL2_SUBIMAGE_OFFSET 41
#define LEVEL3_SUBIMAGE_OFFSET 50
#define LEVEL4_SUBIMAGE_OFFSET 59
#define LEVEL5_SUBIMAGE_OFFSET 63
#define LEVEL6_SUBIMAGE_OFFSET 67
#define LEVEL7_SUBIMAGE_OFFSET 71

//Size of the patch for the ORB descriptor (The radius of the circle we use to compute the ORB descriptor and angle)
#define HALF_PATCH_SIZE 15


//Pyramid Data (Non-VTCM) (128-bit aligned) (NON-HWA Memory version)
// struct pyramid_t{
//     uint8_t outputLevel0[INPUT_WIDTH*INPUT_HEIGHT];
//     uint8_t outputLevel1[(LEVEL1_STRIDE)*LEVEL1_HEIGHT];
//     uint8_t outputLevel2[(LEVEL2_STRIDE)*LEVEL2_HEIGHT];
//     uint8_t outputLevel3[(LEVEL3_STRIDE)*LEVEL3_HEIGHT];
//     uint8_t outputLevel4[(LEVEL4_STRIDE)*LEVEL4_HEIGHT];
//     uint8_t outputLevel5[(LEVEL5_STRIDE)*LEVEL5_HEIGHT];
//     uint8_t outputLevel6[(LEVEL6_STRIDE)*LEVEL6_HEIGHT];
//     uint8_t outputLevel7[(LEVEL7_STRIDE)*LEVEL7_HEIGHT];
//     uint8_t reserved[VLEN]; //For extra vector checks
// };

struct pyramid_t{ //HWA Memory version
    uint8_t outputLevel7[(LEVEL7_STRIDE)*LEVEL7_HEIGHT];     //UNUSED
    uint8_t reserved[VLEN]; //For extra vector checks

    uint8_t* outputLevel0;
    uint8_t* outputLevel1;
    uint8_t* outputLevel2;
    uint8_t* outputLevel3;
    uint8_t* outputLevel4;
    uint8_t* outputLevel5;
    uint8_t* outputLevel6;

    uint8_t reserved2[VLEN]; //For extra vector checks
};

//VTCM Cache Data for each extractor
struct pyramid_VTCM_cache_t{ 
    //We do 256x(80+32) for the sub-image cache
    uint8_t cacheData[(VLEN*2)*(LEVEL7_HEIGHT+HALF_PATCH_SIZE*2+1)];
    
    //We use 16 bits per pixels for the cache so easier gathering
    //Used for general gather operations
    HVX_Vector gatheredVector1; //Multiple cache for better performance
    HVX_Vector gatheredVector2;
    HVX_Vector gatheredVector3;
    HVX_Vector gatheredVector4;

    //Used for horizontal bilinear interpolation
    HVX_Vector bilinearReductionVectors[INPUT_WIDTH/VLEN];

    //Used for cos/sin lookup
    int8_t cos_lookup_table[4*256];
    int8_t sin_lookup_table[4*256];

    //Used for the ORB descriptor optimized computation
    int8_t orb_descriptors_patterns[4*256];
};

//Unused functions - Not used in the final implementation (2n scaleFactor pyramid)
int calculate_pyramid_2n_dumb(uint8_t* inputData, pyramid_t* pyramid);
int calculate_pyramid_image_2n_hvx(uint8_t* inputData, uint8_t* output_data, int input_width, int input_stride, int input_height, int output_stride);
int calculate_pyramid_2n_hvx(uint8_t* inputData, pyramid_t* pyramid);
int calculate_pyramid_2n_hvx_stereo(const uint8_t* inputData, pyramid_t* pyramid1, pyramid_t* pyramid2);
int calculate_pyramid_2n_hvx_stereo_left(const uint8_t* inputData, pyramid_t* pyramid);
int calculate_pyramid_2n_hvx_stereo_right(const uint8_t* inputData, pyramid_t* pyramid);

/**
* @brief Precompute coefficients and indices for horizontal bilinear interpolation of the pyramid. Called once at the beginning of the program.
*/
int precompute_horizontal_bilinear_indices_and_coefs();

/**
 * @brief Copy the input image to the output image. Used for the first level of the pyramid and to separate the stereo images.
 * @param inputData Input image data pointer
 * @param output_data Output image data pointer
 * @param input_width Width of the input image
 * @param input_height Height of the input image
 * @param input_stride Stride of the input image
 * @param output_stride Stride of the output image
 * @return 0 if success
 */
int calculate_pyramid_image_copy_hvx(const uint8_t* inputData, uint8_t* output_data, int input_width, int input_stride, int input_height, int output_stride);

/**
 * @brief Calculate the bilinear interpolation of the input image to the output image. Used for the pyramid calculation. The reduction factor must be between 1 and 2
 * @param inputData Input image data pointer
 * @param output_data Output image data pointer
 * @param input_width Width of the input image
 * @param input_height Height of the input image
 * @param input_stride Stride of the input image
 * @param output_width Width of the output image
 * @param output_height Height of the output image
 * @param output_stride Stride of the output image
 * @param vtcmCache VTCM cache data pointer used for the calculation
 * @param levelIndex Index of the level in the pyramid
 * @return 0 if success
 */
int calculate_pyramid_image_bilinear_hvx(uint8_t* inputData, uint8_t* output_data, int input_width, int input_height, int input_stride, int output_width, int output_height, int output_stride, pyramid_VTCM_cache_t* vtcmCache, int levelIndex);

/**
 * @brief Create an image pyramid using bilinear interpolation. Used for the stereo images. Calculate the left and right images. (Maybe not used in the final implementation)
 * @param inputData Input image data pointer
 * @param pyramid1 Pyramid data pointer for the left image
 * @param pyramid2 Pyramid data pointer for the right image
 * @param vtcmCache VTCM cache data pointer used for the calculation of the left image
 * @param vtcmCache2 VTCM cache data pointer used for the calculation of the right image
 * @return 0 if success
 */
int calculate_pyramid_bilinear_hvx_stereo(const uint8_t* inputData, pyramid_t* pyramid1, pyramid_t* pyramid2, pyramid_VTCM_cache_t* vtcmCache, pyramid_VTCM_cache_t* vtcmCache2);

/**
 * @brief Create an image pyramid using bilinear interpolation. Used for the stereo images. Calculate the left image only.
 * @param inputData Input image data pointer
 * @param pyramid Pyramid data pointer for the left image
 * @param vtcmCache VTCM cache data pointer used for the calculation of the left image
 * @return 0 if success
 */
int calculate_pyramid_bilinear_hvx_stereo_left(const uint8_t* inputData, pyramid_t* pyramid, pyramid_VTCM_cache_t* vtcmCache);

/**
 * @brief Create an image pyramid using bilinear interpolation. Used for the stereo images. Calculate the right image only.
 * @param inputData Input image data pointer
 * @param pyramid Pyramid data pointer for the right image
 * @param vtcmCache VTCM cache data pointer used for the calculation of the right image
 * @return 0 if success
 */
int calculate_pyramid_bilinear_hvx_stereo_right(const uint8_t* inputData, pyramid_t* pyramid, pyramid_VTCM_cache_t* vtcmCache);

/**
 * @brief Cache a stripe of the image with no border. Used to prepare the VTCM sub-image cache. This is used for middle stripes of the image.
 * @param baseAddress Base address of the image data
 * @param pyramid_cache VTCM cache data pointer to store the vtcm cache data
 * @param height Height of the stripe
 * @param input_stride Stride of the input image
 * @return 0 if success
 */
int cache_stripe_vtcm_2_no_border(uint8_t* baseAddress, pyramid_VTCM_cache_t* pyramid_cache, int height, int input_stride);

/**
 * @brief Cache a stripe of the image with a border on the left. Used to prepare the VTCM sub-image cache. This is used for the left border of the image.
 * @param baseAddress Base address of the image data
 * @param pyramid_cache VTCM cache data pointer to store the vtcm cache data
 * @param height Height of the stripe
 * @param input_stride Stride of the input image
 * @return 0 if success
 */
int cache_stripe_vtcm_2_border_left(uint8_t* baseAddress, pyramid_VTCM_cache_t* pyramid_cache, int height, int input_stride);

/**
 * @brief Cache a stripe of the image with a border on the right. Used to prepare the VTCM sub-image cache. This is used for the right border of the image.
 * @param baseAddress Base address of the image data
 * @param pyramid_cache VTCM cache data pointer to store the vtcm cache data
 * @param height Height of the stripe
 * @param input_stride Stride of the input image
 * @return 0 if success
 */
int cache_stripe_vtcm_2_border_right(uint8_t* baseAddress, pyramid_VTCM_cache_t* pyramid_cache, int height, int input_stride);

/**
 * @brief Cache a stripe of the image with borders on both sides. Used to prepare the VTCM sub-image cache. This is used for sub-images with borders on both sides. (Used on the last level only)
 * @param baseAddress Base address of the image data
 * @param pyramid_cache VTCM cache data pointer to store the vtcm cache data
 * @param height Height of the stripe
 * @param input_stride Stride of the input image
 * @return 0 if success
 */
int cache_stripe_vtcm_2_both_borders(uint8_t* baseAddress, pyramid_VTCM_cache_t* pyramid_cache, int height, int input_stride);
