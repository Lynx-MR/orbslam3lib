/**
 * @file orbslam_dsp_hwa_pipeline.h
 *
 * @author Gaston Rouquette (Lynx Mixed Reality)
 *
 * @brief Header to use the Hardware Acceleration Pipeline for feature extraction
**/



//--Do not touch--
#define HWA_EXTRACTOR_MODULE 1 << 1
#define HWA_PYRAMID_EXTRACTOR_MODULE 1 << 2

#define HWA_FPS 90 //TO SET


#define THRESHOLD_ADJUSTER_FEATURES_AVG 170 //Average number of features that should be detected in average
#define THRESHOLD_ADJUSTER_FEATURES_SIGMA 30 //The number of features tolerance before changing the threshold to have the right amount of features


struct orbslam_dsp_hwa_pipeline_context_t;

/// @brief Setup some parameters for the Hardware accelerator pipeline
/// @param hwa_fps 
/// @param threshold_adjusters_features_avg 
/// @param threshold_adjusters_features_sigma 
void hwa_setup_parameters(int hwa_fps, int threshold_adjusters_features_avg, int threshold_adjusters_features_sigma);

/// @brief Initialize the hardware accelerator pipeline for ORB-SLAM feature processing
/// @param contextPtr Pointer to the pipeline context to be initialized
/// @param modules Bitmask specifying which hardware modules to enable
/// @param left_pyramid_for_extractor Pointer to the left image pyramid for feature extraction
/// @param right_pyramid_for_extractor Pointer to the right image pyramid for feature extraction
/// @return 0 on success, negative error code on failure
int init_hwa_pipeline(orbslam_dsp_hwa_pipeline_context_t** contextPtr, int modules, pyramid_t* left_pyramid_for_extractor, pyramid_t* right_pyramid_for_extractor);

/// @brief Destroy and deallocate the hardware accelerator pipeline
/// @param contextPtr Pointer to the pipeline context to be destroyed
/// @return 0 on success, negative error code on failure
int destroy_hwa_pipeline(orbslam_dsp_hwa_pipeline_context_t* contextPtr);

/// @brief Process feature extraction synchronously using the hardware accelerator pipeline
/// @param contextPtr Pointer to the initialized pipeline context
/// @param image Input image data
/// @param points Output array of extracted feature points (x, y coordinates)
/// @param scores Output array of feature scores
/// @param imgLevel Image pyramid level to process
/// @param threadLevel Thread level for parallel processing
/// @param isRight Flag indicating if the image is from the right camera (true) or left (false)
/// @param width Image width in pixels
/// @param height Image height in pixels
/// @param stride Image stride in bytes
/// @param point_count Output number of detected feature points
/// @param max_features Maximum number of features to extract
/// @param threshold Feature detection threshold
/// @return 0 on success, negative error code on failure
int hwa_pipeline_process_feature_extraction(orbslam_dsp_hwa_pipeline_context_t* contextPtr, const uint8_t* image, uint16_t** points, uint16_t** scores, int imgLevel, int threadLevel, bool isRight, int width, int height, int stride, uint32_t* point_count, int max_features, int threshold);

/// @brief Process feature extraction asynchronously using the hardware accelerator pipeline
/// @param contextPtr Pointer to the initialized pipeline context
/// @param image Input image data
/// @param points Output array of extracted feature points (x, y coordinates)
/// @param scores Output array of feature scores
/// @param imgLevel Image pyramid level to process
/// @param threadLevel Thread level for parallel processing
/// @param isRight Flag indicating if the image is from the right camera (true) or left (false)
/// @param width Image width in pixels
/// @param height Image height in pixels
/// @param stride Image stride in bytes
/// @param point_count Output number of detected feature points
/// @param max_features Maximum number of features to extract
/// @param threshold Feature detection threshold (unused)
/// @return 0 on success, negative error code on failure
int hwa_pipeline_process_feature_extraction_async(orbslam_dsp_hwa_pipeline_context_t* contextPtr, const uint8_t* image, uint16_t** points, uint16_t** scores, int imgLevel, int threadLevel, bool isRight, int width, int height, int stride, uint32_t* point_count, int max_features, int threshold);

/// @brief Wait for asynchronous feature extraction to complete
/// @param contextPtr Pointer to the initialized pipeline context
/// @param threadLevel Thread level for parallel processing
/// @param isRight Flag indicating if the image is from the right camera (true) or left (false)
/// @return 0 on success, negative error code on failure
int hwa_pipeline_process_feature_extraction_wait_finish(orbslam_dsp_hwa_pipeline_context_t* contextPtr, int threadLevel, bool isRight);

/// @brief Initialize multithreading for feature extraction in the hardware accelerator pipeline
/// @param contextPtr Pointer to the initialized pipeline context
/// @return 0 on success, negative error code on failure
int init_features_extraction_multithreading(orbslam_dsp_hwa_pipeline_context_t* contextPtr);

/// @brief Stop multithreading for feature extraction in the hardware accelerator pipeline
/// @param contextPtr Pointer to the initialized pipeline context
/// @return 0 on success, negative error code on failure
int stop_feature_extraction_multithreading(orbslam_dsp_hwa_pipeline_context_t* contextPtr);