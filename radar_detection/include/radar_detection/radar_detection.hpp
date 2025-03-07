/**
 * @File: radar_detection.hpp
 * @Date: March 2022
 * @Author: James Swedeen
 *
 * @brief
 * A convenience header that includes everything needed to use the radar detection code.
 **/

#ifndef RADAR_DETECTION_RADAR_DETECTION_HPP
#define RADAR_DETECTION_RADAR_DETECTION_HPP

/* Base Headers */
#include<radar_detection/state_definitions.hpp>
#include<radar_detection/partial_derivatives.hpp>

/* Cross Section Models */
#include<radar_detection/cross_sections/cross_section_model.hpp>
#include<radar_detection/cross_sections/constant_cross_section_model.hpp>
#include<radar_detection/cross_sections/ellipsoid_cross_section_model.hpp>
#include<radar_detection/cross_sections/spikeball_cross_section_model.hpp>

#include<radar_detection/radar_model.hpp>

#endif
/* radar_detection.hpp */
