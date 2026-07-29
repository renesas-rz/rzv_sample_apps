/***********************************************************************************************************************
* Copyright (C) 2023-2026 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name    : dfl_proc_seg.cpp
* Version      : R2026-07
* Description  : RZ/V2H DRP-AI Sample Application for Ultralytics Detection YOLOv8-seg and Stereo with MIPI/USB Camera
***********************************************************************************************************************/

/*****************************************
* Includes
******************************************/
#include "dfl_proc_seg.h"

using namespace std;

DFL::DFL()
{

}

DFL::~DFL()
{

}

/*****************************************
* Function Name : get_threshold_for_non_sigmoid_value
* Description   : process for thread
* Arguments     : -
* Return value  : -
******************************************/
inline float get_threshold_for_non_sigmoid_value()
{          
    /* Threshold for non-sigmoid value */ 
    float th_prob = 0;   
    if ( TH_PROB <= 0 ) 
    { 
        th_prob = -FLT_MAX; 
    }
    else if (TH_PROB >= 1) 
    { 
        th_prob = FLT_MAX; 
    }
    else 
    { 
        th_prob = logf( TH_PROB / (1.0f-TH_PROB) ); 
    }
    return th_prob;
}

inline float get_threshold_for_sigmoid_value()
{
    return TH_PROB;
}

/*****************************************
* Function Name : sigmoid
* Description   : Helper function for YOLO Post Processing
* Arguments     : x = input argument for the calculation
* Return value  : sigmoid result of input x
******************************************/
double DFL::sigmoid(double x)
{
    return 1.0/(1.0 + exp(-x));
}

/*****************************************
* Function Name : softmax
* Description   : Helper function for YOLO Post Processing
* Arguments     : input = input array
*                 size = size of input array
* Return value  : softmax result of input array
******************************************/
void DFL::softmax(const float* input, int size, float* output) 
{
    float max_val = *max_element(input, input + size);
    float sum = 0.0;
    
    for (int i = 0; i < size; i++) 
    {
        output[i] = exp(input[i] - max_val);
        sum += output[i];
    }

    for (int i = 0; i < size; i++) 
    {
        output[i] /= sum;
    }
}

/*****************************************
* Function Name : stage_conv
* Description   : Helper function for YOLO Post Processing
* Arguments     : input = input array
*                 size = size of input array
* Return value  : result of convolution
******************************************/
float DFL::stage_conv(const float* input, int size) 
{
    float result = 0.0f;
    for (int i = 0; i < size; ++i) { result += input[i] * (float)i; }    
    return result;
}

/*****************************************
* Function Name : dfl_seg_process
* Description   : process for thread
* Arguments     : dfl_arr, mask_arr, grid_size, grid_confidences, grid_classes, det
* Return value  : -
******************************************/
void DFL::dfl_seg_process(
    float* dfl_arr,
    float* mask_arr,
    size_t grid_size,
    float* grid_confidences,
    uint32_t* grid_classes,
    std::vector<detection_seg>* det)
{
    int32_t C_in = 64;
    int32_t C_unit_size = 16;
    int32_t C_out = 4;
    size_t H = grid_size;
    size_t W = grid_size;
    int32_t div_val = 2;
    int32_t mul_val = 0;

    if (80 == grid_size)      mul_val = 8;
    else if (40 == grid_size) mul_val = 16;
    else if (20 == grid_size) mul_val = 32;

    float threshold = get_threshold_for_sigmoid_value();
    int area = grid_size * grid_size;

    for (int h = 0; h < H; h++)
    {
        for (int w = 0; w < W; w++)
        {
            uint32_t grid_idx = h * W + w;

            if (grid_confidences[grid_idx] <= threshold)
                continue;

            float softmax_in_0[C_unit_size];
            float softmax_in_1[C_unit_size];
            float softmax_in_2[C_unit_size];
            float softmax_in_3[C_unit_size];
            float softmax_out_0[C_unit_size];
            float softmax_out_1[C_unit_size];
            float softmax_out_2[C_unit_size];
            float softmax_out_3[C_unit_size];

            for (int cu = 0; cu < C_unit_size; cu++)
            {
                uint32_t c0 = get_index_chw_tensor(cu + C_unit_size * 0, h, w, C_in, H, W);
                uint32_t c1 = get_index_chw_tensor(cu + C_unit_size * 1, h, w, C_in, H, W);
                uint32_t c2 = get_index_chw_tensor(cu + C_unit_size * 2, h, w, C_in, H, W);
                uint32_t c3 = get_index_chw_tensor(cu + C_unit_size * 3, h, w, C_in, H, W);

                softmax_in_0[cu] = dfl_arr[c0];
                softmax_in_1[cu] = dfl_arr[c1];
                softmax_in_2[cu] = dfl_arr[c2];
                softmax_in_3[cu] = dfl_arr[c3];
            }

            softmax(softmax_in_0, C_unit_size, softmax_out_0);
            softmax(softmax_in_1, C_unit_size, softmax_out_1);
            softmax(softmax_in_2, C_unit_size, softmax_out_2);
            softmax(softmax_in_3, C_unit_size, softmax_out_3);

            float intm_2_0 = stage_conv(softmax_out_0, C_unit_size);
            float intm_2_1 = stage_conv(softmax_out_1, C_unit_size);
            float intm_2_2 = stage_conv(softmax_out_2, C_unit_size);
            float intm_2_3 = stage_conv(softmax_out_3, C_unit_size);

            /* Add & Sub by fixed parameter */
            float intm_3_sub_0 = w + 0.5f - intm_2_0;
            float intm_3_sub_1 = h + 0.5f - intm_2_1;
            float intm_3_add_0 = w + 0.5f + intm_2_2;
            float intm_3_add_1 = h + 0.5f + intm_2_3;

            float intm_4_add_out_0 = intm_3_sub_0 + intm_3_add_0;
            float intm_4_add_out_1 = intm_3_sub_1 + intm_3_add_1;
            float intm_4_sub_out_0 = intm_3_add_0 - intm_3_sub_0;
            float intm_4_sub_out_1 = intm_3_add_1 - intm_3_sub_1;

            /* Div & Mul */
            float cx = (intm_4_add_out_0 / div_val) * mul_val;
            float cy = (intm_4_add_out_1 / div_val) * mul_val;
            float bw = (intm_4_sub_out_0) * mul_val;
            float bh = (intm_4_sub_out_1) * mul_val;

            detection_seg d{};
            d.bbox = {cx, cy, bw, bh};
            d.c    = grid_classes[grid_idx];
            d.prob = grid_confidences[grid_idx];

            for (int k = 0; k < NUM_MASK; k++)
            {
                d.mask_coeff[k] = mask_arr[k * area + grid_idx];
            }

            det->push_back(d);
        }
    }
}

/*****************************************
* Function Name : get_class_predictions
* Description   : process for thread
* Arguments     : class_scores, grid_size, num_classes, grid_confidences, grid_classes
* Return value  : -
******************************************/

void DFL::get_class_predictions(float_t* class_scores, uint32_t grid_size, uint32_t num_classes, float_t* grid_confidences, uint32_t* grid_classes)
{
#if (1) == ONLY_PERSON
    for (int c = 0; c < 1; c++)
#else
    for (int c = 0; c < num_classes; c++)
#endif
    {
        for (uint32_t h = 0; h < grid_size; h++) 
        {
            for (uint32_t w = 0; w < grid_size; w++) 
            {
                // calculate index
                uint32_t grid_idx = h * grid_size + w;
                uint32_t idx = get_index_chw_tensor( c, h, w, num_classes, grid_size, grid_size );
                float_t class_score = class_scores[idx];

                // Apply teh argmax function for each grid
                if( c == 0 )
                {
                    grid_confidences[grid_idx] = class_score;
                    grid_classes[grid_idx] = c;
                }
                else
                {   
                    if (class_score > grid_confidences[grid_idx])
                    {
                        grid_confidences[grid_idx] = class_score;
                        grid_classes[grid_idx] = c;
                    }
                }
                
                // Apply the sigmoid function to argmax results for each grid
#if (1) == ONLY_PERSON
                grid_confidences[grid_idx] = DFL::sigmoid(grid_confidences[grid_idx]);
#else
                if ( c == num_classes-1 )
                {
                    grid_confidences[grid_idx] = DFL::sigmoid(grid_confidences[grid_idx]);
                }
#endif
            }
        }
    }
}

/*****************************************
* Function Name : DFL_Proc_Seg
* Description   : DFL process for Yolov8
* Arguments     : dfl80, dfl40, dfl20 = dfl array
*                 class80, class40, class20 = class array
                  mask80, mask40, mask20 = mask array
                  det
* Return value  : -
******************************************/
void DFL::DFL_Proc_Seg(float* dfl80, float* dfl40, float* dfl20,
    float* class80, float* class40, float* class20,
    float* mask80, float* mask40, float* mask20,
    std::vector<detection_seg>* det)
{
    static float_t grid_confidences_80[num_grid80];
    static float_t grid_confidences_40[num_grid40];
    static float_t grid_confidences_20[num_grid20];
    static uint32_t grid_classes_80[num_grid80];
    static uint32_t grid_classes_40[num_grid40];
    static uint32_t grid_classes_20[num_grid20]; 

#if (1) == CPU_DFL_MULTI_THREAD
    thread thread_sigmoid_80(&DFL::get_class_predictions, this, class80, num_grids[0], NUM_CLASS, grid_confidences_80, grid_classes_80);
    thread thread_sigmoid_40(&DFL::get_class_predictions, this, class40, num_grids[1], NUM_CLASS, grid_confidences_40, grid_classes_40);
    thread thread_sigmoid_20(&DFL::get_class_predictions, this, class20, num_grids[2], NUM_CLASS, grid_confidences_20, grid_classes_20);
    thread_sigmoid_80.join();
    thread_sigmoid_40.join();
    thread_sigmoid_20.join();
#else 
    DFL::get_class_predictions(class80, num_grids[0], NUM_CLASS, grid_confidences_80, grid_classes_80);
    DFL::get_class_predictions(class40, num_grids[1], NUM_CLASS, grid_confidences_40, grid_classes_40);
    DFL::get_class_predictions(class20, num_grids[2], NUM_CLASS, grid_confidences_20, grid_classes_20);
#endif

    vector<detection_seg> det80;
    vector<detection_seg> det40;
    vector<detection_seg> det20;
#if (1) == CPU_DFL_MULTI_THREAD
    thread thread_dfl_80(&DFL::dfl_seg_process, this, dfl80, mask80, num_grids[0], grid_confidences_80, grid_classes_80, &det80);
    thread thread_dfl_40(&DFL::dfl_seg_process, this, dfl40, mask40, num_grids[1], grid_confidences_40, grid_classes_40, &det40);
    thread thread_dfl_20(&DFL::dfl_seg_process, this, dfl20, mask20, num_grids[2], grid_confidences_20, grid_classes_20, &det20);
    thread_dfl_80.join();
    thread_dfl_40.join();
    thread_dfl_20.join();
#else 
    DFL::dfl_seg_process(dfl80, mask80, num_grids[0], grid_confidences_80, grid_classes_80, &det80);
    DFL::dfl_seg_process(dfl40, mask40, num_grids[1], grid_confidences_40, grid_classes_40, &det40);
    DFL::dfl_seg_process(dfl20, mask20, num_grids[2], grid_confidences_20, grid_classes_20, &det20);
#endif
    
    std::copy(det80.begin(), det80.end(), std::back_inserter(*det) );
    std::copy(det40.begin(), det40.end(), std::back_inserter(*det) );
    std::copy(det20.begin(), det20.end(), std::back_inserter(*det) );

    return; 
}
