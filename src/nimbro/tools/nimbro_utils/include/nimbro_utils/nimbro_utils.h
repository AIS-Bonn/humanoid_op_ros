// Header file that includes all nimbro utilities header files
// File: nimbro_utils.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef NIMBRO_UTILS_H
#define NIMBRO_UTILS_H

/**
* @namespace nimbro_utils
*
* @brief This namespace defines a set of various utilities, both ROS dependent and ROS agnostic,
* that can be used for common fundamental tasks.
**/
namespace nimbro_utils {}

// Includes
#include <nimbro_utils/ew_integrator.h>
#include <nimbro_utils/lin_sin_fillet.h>
#include <nimbro_utils/math_funcs.h>
#include <nimbro_utils/math_spline.h>
#include <nimbro_utils/math_vec_mat.h>
#include <nimbro_utils/mean_filter.h>
#include <nimbro_utils/ros_timing.h>
#include <nimbro_utils/slope_limiter.h>
#include <nimbro_utils/smooth_deadband.h>
#include <nimbro_utils/wlbf_filter.h>

#endif /* NIMBRO_UTILS_H */
// EOF