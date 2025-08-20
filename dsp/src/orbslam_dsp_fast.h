/**
 * @file orbslam_dsp_fast.h
 *
 * @author Gaston Rouquette (Lynx Mixed Reality)
 *
 * @brief Legacy DSP Code to calculate features based on an image, should now be replaced by Hardware Accelerators
**/

#pragma once

#include "orbslam3.h"
#include "orbslam_dsp_pyramid.h"
#include "orbslam_dsp_sort.h"


#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wunused-variable"

// Some macros to help debugging
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0') 

/**
 * @brief The FAST features of an image using scalar code, UNUSED and UNOPTIMIZED LEGACY CODE
 */
int calculate_fast_features_dumb(uint8_t* image, uint32_t width, uint32_t height, int stride, uint16_t** subImageFeatures, uint32_t subImageCount, uint8_t threshold, uint32_t maxFeaturesPerSubImage, uint32_t* featuresPerSubImage);


/**
 * @brief Get the FAST features candidates of an image using optimized HVX code.
 * @param image the input image pointer
 * @param width the width of the image
 * @param height the height of the image
 * @param stride the stride of the image
 * @param subImageFeatures the output features positions array
 * @param subImageCount the count of subimages to process
 * @param threshold the threshold for the FAST feature extraction (1st step)
 * @param minThreshold the threshold for the FAST feature extraction (2nd optional step)
 * @param minFeatures the minimum features to get during the first step inside a subimage (128x80 block) to continue the process without trying to get more features with a lower threshold (Second step MIN_THRESHOLD) (0 = Disable second detection)
 * @param maxFeaturesPerSubImage the maximum features to get during this step for each subimage
 * @param featuresPerSubImage the output features count array for each subimage
 * @retval The total count of features found (unused)
 */
int calculate_fast_features_hvx(uint8_t* image, uint32_t width, uint32_t height, int stride, uint16_t** subImageFeatures, uint32_t subImageCount, uint8_t threshold, uint8_t minThreshold, int minFeatures, uint32_t maxFeaturesPerSubImage, uint32_t* featuresPerSubImage);

/**
 * @brief Get the FAST features candidates of a sub-image using optimized HVX code.
 * @param imageCol the input image pointer
 * @param height the height of the image
 * @param stride the stride of the image
 * @param features the output features positions array
 * @param threshold the threshold for the FAST feature extraction
 * @param maxFeatures the maximum features to get during this step
 * @param featureCount the current feature count
 * @param yoffset the y offset of the sub-image
 * @param predmaskInit the initial prediction mask
 * @retval The total count of features found
 * @note This function is used by calculate_fast_features_hvx
 */
int calculate_fast_features_hvx_colvec(uint8_t* imageCol, uint32_t height, int stride, uint16_t* features, uint8_t threshold, uint32_t maxFeatures, uint32_t featureCount, uint32_t yoffset, HVX_Vector predmaskInit);

/**
 * @brief Calculate the FAST scores of the features of an image using optimized HVX code.
 * @param image the input image pointer
 * @param imageHeight the height of the image
 * @param imageStride the stride of the image
 * @param vtcmImageCache the VTCM cache of the image
 * @param subImageFeatures the input features positions array
 * @param subImageFeaturesScores the output features scores array
 * @param subImageCount the count of subimages to process
 * @param featuresPerSubImage the input features count array for each subimage
 * @retval 0 for success, 1 for error
 */
int calculate_fast_scores(uint8_t* image, uint32_t imageHeight, uint32_t imageStride, pyramid_VTCM_cache_t* vtcmImageCache, uint16_t** subImageFeatures, uint16_t** subImageFeaturesScores, uint32_t subImageCount, uint32_t* featuresPerSubImage);

/**
 * @brief Calculate the FAST scores of the features of a sub-image using optimized HVX code.
 * @param vtcmImageCache the VTCM cache of the image
 * @param height the height of the image
 * @param stride the stride of the image
 * @param features the input features positions array
 * @param featuresScore the output features scores array
 * @param featureCount the input features count
 * @param yoffset the vertical offset of the sub-image (used to align to the VTCM Image Cache correctly)
 * @retval 0 for success, 1 for error
 * @note This function is used by calculate_fast_scores
 */
int calculate_fast_scores_stride(pyramid_VTCM_cache_t* vtcmImageCache, uint32_t height, int stride, uint16_t* features, uint16_t* featuresScore, uint32_t featureCount, uint32_t yoffset);

/**
 * @brief Calculate the non-maximal suppression of the features of an image using optimized HVX code.
 * @param imageHeight the height of the image
 * @param subImageFeatures the input features positions array
 * @param subImageFeaturesScores the input features scores array
 * @param subImageCount the count of subimages to process
 * @param featuresPerSubImage the input features count array for each subimage
 * @param maxFeaturesPerSubImage the maximum features to keep before the sorting step
 * @param minimumThreshold the threshold to keep the features after the non-maximal suppression
 * @retval 0 for success, 1 for error
 */
int calculate_non_maximal_suppression(uint32_t imageHeight, uint16_t** subImageFeatures, uint16_t** subImageFeaturesScores, uint32_t subImageCount, uint32_t* featuresPerSubImage, int maxFeaturesPerSubImage, int minimumThreshold);

/**
 * @brief Calculate the horizontal non-maximal suppression of the features of a sub-image using optimized HVX code.
 * @param featuresCount the input features count
 * @param imageHeight the height of the image
 * @param subImageFeatures the input features positions array
 * @param subImageFeaturesScores the input features scores array
 * @param maxFeaturesPerSubImage the maximum features to keep before the sorting step
 * @retval 0 for success, 1 for error
 * @note This function is used by calculate_non_maximal_suppression_stride
 */
int calculate_non_maximal_horizontal_suppression_stride(uint32_t featuresCount, uint32_t imageHeight, uint16_t* subImageFeatures, uint16_t* subImageFeaturesScores);

/**
 * @brief Sort and clean the features of an image using optimized HVX code.
 * @param featuresCount the input features count
 * @param imageHeight the height of the image
 * @param subImageFeatures the input features positions array
 * @param subImageFeaturesScores the input features scores array
 * @param maxFeaturesPerSubImage the maximum features to keep before the sorting step
 * @param minimumScore the minimum score to keep the features after the sorting step
 * @retval The total count of features found
 */
int calculate_non_maximal_suppression_stride(uint32_t featuresCount, uint32_t imageHeight, uint16_t* subImageFeatures, uint16_t* subImageFeaturesScores, int maxFeaturesPerSubImage, int minimumThreshold);

/**
 * @brief Sort and clean the features of an image using optimized HVX code to keep only the best ones.
 * @param imageHeight the height of the image
 * @param subImageFeatures the input features positions array
 * @param subImageFeaturesScores the input features scores array
 * @param subImageCount the count of subimages to process
 * @param featuresPerSubImage the input features count array for each subimage
 * @param maxFeaturesPerSubImage the maximum features to keep before the sorting step
 * @param minimumScore the minimum score to keep the features after the sorting step
 * @retval 0 for success, 1 for error
 */
int sort_and_clean_features(uint32_t imageHeight, uint16_t** subImageFeatures, uint16_t** subImageFeaturesScores, uint32_t subImageCount, uint32_t* featuresPerSubImage, uint32_t maxFeaturesPerSubImage, uint32_t minimumScore, bool verticalToHorizontalCoordinateReordering = true);

/**
 * @brief Sort and clean the features of a sub-image using optimized HVX code to keep only the best ones.
 * @param featuresCount the input features count
 * @param imageHeight the height of the image
 * @param subImageFeatures the input features positions array
 * @param subImageFeaturesScores the input features scores array
 * @param maxFeaturesPerSubImage the maximum features to keep before the sorting step (must be a multiple of VLEN/2 (64))
 * @param minimumScore the minimum score to keep the features after the sorting step
 * @retval The total count of features found
 */
int sort_and_clean_features_stride(uint32_t featuresCount, uint32_t imageHeight, uint16_t* subImageFeatures, uint16_t* subImageFeaturesScores, uint32_t maxFeaturesPerSubImage, uint32_t minimumScore, bool verticalToHorizontalCoordinateReordering);

/**
 * @brief Clean the features of an image using optimized HVX code to remove ones which do not have a high enough score.
 * @param imageHeight the height of the image
 * @param subImageFeatures the input/output features positions array
 * @param subImageFeaturesScores the input features scores array
 * @param subImageCount the count of subimages to process
 * @param featuresPerSubImage the input/output features count array for each subimage
 * @param maxFeaturesPerSubImage the maximum features to keep before the sorting step
 * @param minimumScore the minimum score to keep the features after the sorting step
 * @retval 0 for success, 1 for error
 */
int clean_features(uint32_t imageHeight, uint16_t** subImageFeatures, uint16_t** subImageFeaturesScores, uint32_t subImageCount, uint32_t* featuresPerSubImage, uint32_t maxFeaturesPerSubImage, uint32_t minimumScore);

/**
 * @brief Clean the features of a sub-image using optimized HVX code to remove ones which do not have a high enough score.
 * @param featuresCount the input features count
 * @param imageHeight the height of the image
 * @param subImageFeatures the input features positions array
 * @param subImageFeaturesScores the input features scores array
 * @param maxFeaturesPerSubImage the maximum features to keep before the sorting step
 * @param minimumScore the minimum score to keep the features after the sorting step
 * @retval The total count of features found
 */
int clean_features_stride(uint32_t featuresCount, uint32_t imageHeight, uint16_t* subImageFeatures, uint16_t* subImageFeaturesScores, uint32_t maxFeaturesPerSubImage, uint32_t minimumScore);