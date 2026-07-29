/***********************************************************************************************************************
* Copyright (C) 2023-2026 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name    : dfl_proc_seg.h
* Version      : R2026-07
* Description  : RZ/V2H DRP-AI Sample Application for Ultralytics Detection YOLOv8 with MIPI/USB Camera
***********************************************************************************************************************/

#ifndef DFL_PROC_SEG_H
#define DFL_PROC_SEG_H

#include "define.h"
#include "box_seg.h"
#include <define_color_yolov8_seg.h>

inline uint32_t get_index_hwc_tensor(uint32_t h, uint32_t w, uint32_t c, size_t h_size, size_t w_size, size_t c_size)
{
    return ( h * w_size + w ) * c_size + c;
}

inline uint32_t get_index_chw_tensor(uint32_t c, uint32_t h, uint32_t w, size_t c_size, size_t h_size, size_t w_size)
{
    return ( c * h_size + h) * w_size + w;
}

inline void transpose_chw_to_hwc( float_t* chw_tensor, float_t* hwc_tensor, size_t c_size, size_t h_size, size_t w_size)
{
    for (uint32_t h = 0; h < h_size; h++) 
    {
        for (uint32_t w = 0; w < w_size; w++) 
        {
            for (uint32_t c = 0; c < c_size; c++)
            {
                uint32_t idx_chw = get_index_chw_tensor( c, h, w, c_size, h_size, w_size );
                uint32_t idx_hwc = get_index_hwc_tensor( h, w, c, h_size, w_size, c_size );
                hwc_tensor[idx_hwc] = chw_tensor[idx_chw];
            }
        }
    }
}

class DFL
{
    public:
        DFL();
        ~DFL();

        void DFL_Proc_Seg(float* dfl80, float* dfl40, float* dfl20,
                      float* class80, float* class40, float* class20,
                      float* mask80, float* mask40, float* mask20,
                      std::vector<detection_seg>* det);
        double sigmoid(double x);

    private:
        void softmax(const float* input, int size, float* output);
        float stage_conv(const float* input, int size);
        float* stage_add_0(float* arr, int32_t h, int32_t w);
        float* stage_add_1(float* arr, int32_t h, int32_t w);
        float* stage_sub_0(float* arr, int32_t h, int32_t w);
        float* stage_sub_1(float* arr, int32_t h, int32_t w);
        void dfl_seg_process(float* dfl_arr, float* mask_arr, size_t grid_size, float* grid_confidences, uint32_t* grid_classes, std::vector<detection_seg>* det);
        void get_class_predictions(float_t* class_scores, uint32_t grid_size, uint32_t num_classes, float_t* grid_confidences, uint32_t* grid_classes );
};

#endif
