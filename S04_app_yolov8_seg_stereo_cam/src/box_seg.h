/***********************************************************************************************************************
* Copyright (C) 2023-2026 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name    : box_seg.h
* Version      : R2026-07
* Description  : For RZ/V2H DRP-AI Sample Application with MIPI/USB Camera
***********************************************************************************************************************/

#ifndef BOX_SEG_H
#define BOX_SEG_H

#include "define.h"
#include "box.h"

#include <vector>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <array>

/*****************************************
* detection : Detected result
******************************************/
typedef struct detection_seg
{
    Box bbox;
    int32_t c;
    float prob;

    /* YOLOv8-seg */
    std::array<float, NUM_MASK> mask_coeff;
    cv::Mat mask;
 
    float avg_disparity;      // Average disparity in Prototype (pixel)
    float median_disparity;   // Median  disparity in Prototype (pixel)
    float avg_distance_m;     // Average distance [m]
    float median_distance_m;  // Median  distance [m]
} detection_seg;

void filter_boxes_nms_seg(std::vector<detection_seg> &det, int32_t size, float th_nms);

#endif
