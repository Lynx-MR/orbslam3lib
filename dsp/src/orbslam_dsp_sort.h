/**
 * @file orbslam_dsp_sort.h
 *
 * @author Gaston Rouquette (Lynx Mixed Reality)
 *
 * @brief Optimized bitonic sort algorithm using HVX instructions (Legacy)
**/


#pragma once

#include "orbslam3.h"
#include "orbslam_dsp_pyramid.h"

#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wunused-variable"

//This file is for bitonic sort functions
//Optimized Sort algorithm using HVX Instructions


int bitonic_sort_uint16_increasing_hvx(uint16_t* keys, uint16_t* values, uint32_t vectorCount);
int bitonic_sort_uint16_decreasing_hvx(uint16_t* keys, uint16_t* values, uint32_t vectorCount);


int bitonic_sort_test(uint16_t* memory);