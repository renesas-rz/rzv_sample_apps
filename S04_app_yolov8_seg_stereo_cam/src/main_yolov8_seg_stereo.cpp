/***********************************************************************************************************************
* Copyright (C) 2023-2026 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name    : main_yolov8_seg_stereo.cpp
* Version      : R2026-07
* Description  : RZ/V2H DRP-AI Sample Application for Ultralytics Detection YOLOv8-seg and Stereo with MIPI/USB Camera
***********************************************************************************************************************/

/*****************************************
* Includes
******************************************/
/*DRP-AI TVM[*1] Runtime*/
#include "MeraDrpRuntimeWrapper.h"
/*Pre-processing Runtime Header*/
#include "PreRuntime.h"

/*DRPAI Driver Header*/
#include <linux/drpai.h>
/*Definition of Macros & other variables*/
#include "define.h"
#include "define_color_yolov8_seg.h"
/*DFL process control*/
#include "dfl_proc_seg.h"
/*USB camera control*/
#include "camera2.h"
/*Input Files control*/
#include "input_files.h"
/*Image control*/
#include "image_yolov8_seg_stereo.h"
/*Wayland control*/
#include "wayland.h"
/*box drawing*/
#include "box_seg.h"
/*mode*/
#include "mode_seg_stereo.h"

using namespace std;
/*****************************************
* Global Variables
******************************************/
/*Multithreading*/
static sem_t terminate_req_sem;
static pthread_t ai_inf_thread;
static pthread_t stereo_thread;
static pthread_t kbhit_thread;
static pthread_t capture_thread;
static pthread_t img_thread;
static pthread_t hdmi_thread;
static mutex mtx;

/*Flags*/
static atomic<uint8_t> inference_start (0);
static atomic<uint8_t> stereo_start    (0);
static atomic<uint8_t> img_obj_ready   (0);
static atomic<uint8_t> hdmi_obj_ready  (0);
#ifdef INPUT_IMAGE
static atomic<uint8_t> post_proc_end   (0);
static int inf_1st_ready = 4;
static int output_en = 1;
#endif

/*Global Variables*/
static float output_dfl80[num_dfl80_out];
static float output_dfl40[num_dfl40_out];
static float output_dfl20[num_dfl20_out];
static float output_class80[num_class80_out];
static float output_class40[num_class40_out];
static float output_class20[num_class20_out];

// ===== YOLOv8-seg (mask) =====
static float output_mask80[num_mask80_out];
static float output_mask40[num_mask40_out];
static float output_mask20[num_mask20_out];
static float output_proto[num_proto_out];

static uint64_t capture_address;
static uint8_t buf_id;
static Image img;     // left
static Image img1;    // right
static DFL dfl;
Mode md;

/*AI Inference for DRPAI*/
/* DRP-AI TVM[*1] Runtime object */
MeraDrpRuntimeWrapper runtime;
/* Pre-processing Runtime object */
PreRuntime preruntime;

/* OpenCVA */
unsigned long OCA_list[OCA_LIST_NUM];

static double drpai_time = 0;
#ifdef DISP_AI_FRAME_RATE
static double ai_fps = 0;
static double cap_fps = 0;
static double proc_time_capture = 0;
static uint32_t array_cap_time[30] = {1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000};
#endif /* DISP_AI_FRAME_RATE */
static uint32_t disp_time = 0;
static uint32_t array_drp_time[30] = {1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000};
static uint32_t array_disp_time[30] = {1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000};

static int32_t drp_max_freq;
static int32_t drpai_freq;
#if END_DET_TYPE
static int8_t display_state=0;
#endif

static Wayland wayland;

static vector<detection_seg> det;
static vector<detection_seg> det_ai_stereo;
static vector<detection_seg> det_display;
static vector<std::array<float, NUM_MASK>> det_mask_coeff;


#ifndef INPUT_IMAGE  // INPUT_CAMERA
static Camera2* g_capture = NULL;
static Camera2* g_capture1 = NULL;

typedef struct {
    Camera2 *capture;
    Camera2 *capture1;
} capture_arg_t;

#else
static Input_Files* g_capture = NULL;
static Input_Files* g_capture1 = NULL;

typedef struct {
    Input_Files *capture;
    Input_Files *capture1;
} capture_arg_t;
#endif

static capture_arg_t cap_arg;

static double pre_time = 0;
static double post_time = 0;
static double ai_time = 0;
static double stereo_time = 0;
static double pre_stereo_time = 0;
static double post_stereo_time = 0;
static double wait_stereo_time = 0;

/* disparity (stereo_out) visualization shared between threads */
static std::mutex disp_mtx;
static cv::Mat disparity_bgra_cam;   // CV_8UC4, size = CAM_IMAGE_WIDTH x CAM_IMAGE_HEIGHT
static cv::Mat disparity_u8_cam;     // CV_8UC1, CAM_IMAGE WxH (for calculate average)

/*****************************************
* Function Name     : float16_to_float32
* Description       : Function by Edgecortex. Cast uint16_t a into float value.
* Arguments         : a = uint16_t number
* Return value      : float = float32 number
******************************************/
float float16_to_float32(uint16_t a)
{
    return __extendXfYf2__<uint16_t, uint16_t, 10, float, uint32_t, 23>(a);
}

inline void cast_array_fp16_to_fp32( uint16_t * __restrict in_tensor, float_t * __restrict out_tensor, size_t size)
{
    // for (uint32_t i = 0; i < size; i++) 
    // {
    //     out_tensor[i] = float16_to_float32(in_tensor[i]);
    // }
    size_t i = 0;
    for (; i + 8 <= size; i += 8) {
        uint16x4_t u16a = vld1_u16(&in_tensor[i+0]);
        uint16x4_t u16b = vld1_u16(&in_tensor[i+4]);
        float32x4_t f4a = vcvt_f32_f16(vreinterpret_f16_u16(u16a));
        float32x4_t f4b = vcvt_f32_f16(vreinterpret_f16_u16(u16b));
        vst1q_f32(&out_tensor[i+0], f4a);
        vst1q_f32(&out_tensor[i+4], f4b);
    }
    for (; i < size; ++i) 
    {
        out_tensor[i] = float16_to_float32(in_tensor[i]);
    }
}

/*****************************************
* Function Name : timedifference_msec
* Description   : compute the time differences in ms between two moments
* Arguments     : t0 = start time
*                 t1 = stop time
* Return value  : the time difference in ms
******************************************/
static double timedifference_msec(struct timespec t0, struct timespec t1)
{
    return (t1.tv_sec - t0.tv_sec) * 1000.0 + (t1.tv_nsec - t0.tv_nsec) / 1000000.0;
}

/*****************************************
* Function Name : wait_join
* Description   : waits for a fixed amount of time for the thread to exit
* Arguments     : p_join_thread = thread that the function waits for to Exit
*                 join_time = the timeout time for the thread for exiting
* Return value  : 0 if successful
*                 not 0 otherwise
******************************************/
static int8_t wait_join(pthread_t *p_join_thread, uint32_t join_time)
{
    int8_t ret_err;
    struct timespec join_timeout;
    ret_err = clock_gettime(CLOCK_REALTIME, &join_timeout);
    if ( 0 == ret_err )
    {
        join_timeout.tv_sec += join_time;
        ret_err = pthread_timedjoin_np(*p_join_thread, NULL, &join_timeout);
    }
    return ret_err;
}

/*****************************************
* Function Name : get_result
* Description   : Get DRP-AI Output from memory via DRP-AI Driver
* Arguments     : -
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
int8_t get_result()
{
    int8_t ret = 0;

    // Load pointer
    uint16_t* data_class80_ptr = reinterpret_cast<uint16_t*>(std::get<1>(runtime.GetOutput(0)));
    uint16_t* data_dfl80_ptr   = reinterpret_cast<uint16_t*>(std::get<1>(runtime.GetOutput(1)));
    uint16_t* data_mask80_ptr  = reinterpret_cast<uint16_t*>(std::get<1>(runtime.GetOutput(2)));

    uint16_t* data_class40_ptr = reinterpret_cast<uint16_t*>(std::get<1>(runtime.GetOutput(3)));
    uint16_t* data_dfl40_ptr   = reinterpret_cast<uint16_t*>(std::get<1>(runtime.GetOutput(4)));
    uint16_t* data_mask40_ptr  = reinterpret_cast<uint16_t*>(std::get<1>(runtime.GetOutput(5)));

    uint16_t* data_class20_ptr = reinterpret_cast<uint16_t*>(std::get<1>(runtime.GetOutput(6)));
    uint16_t* data_dfl20_ptr   = reinterpret_cast<uint16_t*>(std::get<1>(runtime.GetOutput(7)));
    uint16_t* data_mask20_ptr  = reinterpret_cast<uint16_t*>(std::get<1>(runtime.GetOutput(8)));

    uint16_t* data_proto_ptr   = reinterpret_cast<uint16_t*>(std::get<1>(runtime.GetOutput(9)));

    cast_array_fp16_to_fp32( data_class80_ptr, output_class80, num_class80_out );
    cast_array_fp16_to_fp32( data_class40_ptr, output_class40, num_class40_out );
    cast_array_fp16_to_fp32( data_class20_ptr, output_class20, num_class20_out );

    cast_array_fp16_to_fp32( data_dfl80_ptr, output_dfl80, num_dfl80_out );
    cast_array_fp16_to_fp32( data_dfl40_ptr, output_dfl40, num_dfl40_out );
    cast_array_fp16_to_fp32( data_dfl20_ptr, output_dfl20, num_dfl20_out );

    cast_array_fp16_to_fp32( data_mask80_ptr, output_mask80, num_mask80_out );
    cast_array_fp16_to_fp32( data_mask40_ptr, output_mask40, num_mask40_out );
    cast_array_fp16_to_fp32( data_mask20_ptr, output_mask20, num_mask20_out );

    cast_array_fp16_to_fp32( data_proto_ptr, output_proto, num_proto_out );

    return ret;
}

/*****************************************
* Function Name : sigmoid_mat
* Description   : sigmoid
* Arguments     : const cv::Mat& x
* Return value  : cv::Mat y
******************************************/
static inline cv::Mat sigmoid_mat(const cv::Mat& x)
{
    cv::Mat y;
    cv::exp(-x, y);
    y = 1.0 / (1.0 + y);
    return y;
}

/*****************************************
* Function Name : topk_seg
* Description   : topk
* Arguments     : det, top_k
* Return value  : top_k
******************************************/
static void topk_seg(std::vector<detection_seg>& det_buff, size_t top_k)
{
    if (top_k == 0 || det_buff.size() <= top_k)
    {
        return;
    }

    std::partial_sort(
        det_buff.begin(),
        det_buff.begin() + top_k,
        det_buff.end(),
        [](const detection_seg& a, const detection_seg& b)
        {
            return a.prob > b.prob;
        }
    );

    det_buff.resize(top_k);
}

/*****************************************
* Function Name : R_Post_Proc_seg
* Description   : Process CPU post-processing for Yolov8
* Arguments     : -
* Return value  : -
******************************************/
void R_Post_Proc_Seg(float* dfl80, float* dfl40, float* dfl20, float* class80, float* class40, float* class20,
                     float* mask80, float* mask40, float* mask20)
{
    std::vector<detection_seg> det_buff;

    dfl.DFL_Proc_Seg(dfl80, dfl40, dfl20, class80, class40, class20, mask80, mask40, mask20, &det_buff);

    /* ---- Pre-NMS Top-K ---- */
    topk_seg(det_buff, TOP_K_SEG);

    /* Non-Maximum Supression filter */
    filter_boxes_nms_seg(det_buff, det_buff.size(), TH_NMS);

    const float scale_x = float(DRPAI_IN_WIDTH)  / float(MODEL_IN_W);
    const float scale_y = float(DRPAI_IN_HEIGHT) / float(MODEL_IN_H);

    cv::Mat proto_mat(NUM_MASK, PROTO_H * PROTO_W, CV_32F, (void*)output_proto);

    std::vector<detection_seg> det_final;
    det_final.reserve(det_buff.size());

    // PROTO<->DRPAI input scale
    const float proto_sx = float(PROTO_W) / float(DRPAI_IN_WIDTH);
    const float proto_sy = float(PROTO_H) / float(DRPAI_IN_HEIGHT);

    /* Log Output */
    int iBoxCount=0;
    for (size_t i = 0; i < det_buff.size(); i++)
    {
        det_buff[i].mask = cv::Mat::zeros(CAM_IMAGE_HEIGHT, CAM_IMAGE_WIDTH, CV_8UC1);
        /* Skip the overlapped bounding boxes */
        if (det_buff[i].prob == 0) continue;

        float cx = det_buff[i].bbox.x * scale_x;
        float cy = det_buff[i].bbox.y * scale_y;
        float bw = det_buff[i].bbox.w * scale_x;
        float bh = det_buff[i].bbox.h * scale_y;

        spdlog::info(" Bounding Box Number : {}",i+1);
        spdlog::info(" Bounding Box        : (X, Y, W, H) = ({}, {}, {}, {})", (int)cx, (int)cy, (int)bw, (int)bh);
        spdlog::info(" Detected Class      : {} (Class {})", label_file_map[det_buff[i].c].c_str(), det_buff[i].c);
        spdlog::info(" Probability         : {} %", (std::round((det_buff[i].prob*100) * 10) / 10));
        iBoxCount++;

        /* Mask data */
        int x1 = (int)round(cx - bw / 2.0f);
        int y1 = (int)round(cy - bh / 2.0f);
        int x2 = (int)round(cx + bw / 2.0f);
        int y2 = (int)round(cy + bh / 2.0f);

        x1 = std::max(0, std::min(CAM_IMAGE_WIDTH  - 1, x1));
        y1 = std::max(0, std::min(CAM_IMAGE_HEIGHT - 1, y1));
        x2 = std::max(0, std::min(CAM_IMAGE_WIDTH  - 1, x2));
        y2 = std::max(0, std::min(CAM_IMAGE_HEIGHT - 1, y2));

        int rw = std::max(1, x2 - x1 + 1);
        int rh = std::max(1, y2 - y1 + 1);
        cv::Rect rect(x1, y1, rw, rh);

        cv::Mat coeff(1, NUM_MASK, CV_32F, (void*)det_buff[i].mask_coeff.data());
        cv::Mat mask_flat = coeff * proto_mat;
        cv::Mat mask_160  = mask_flat.reshape(1, PROTO_H);
        cv::Mat mask_sig  = sigmoid_mat(mask_160);

        int px1 = (int)floorf(x1 * proto_sx);
        int py1 = (int)floorf(y1 * proto_sy);
        int px2 = (int)ceilf((x2 + 1) * proto_sx); // +1 for inclusive
        int py2 = (int)ceilf((y2 + 1) * proto_sy);
        px1 = std::max(0, std::min(PROTO_W - 1, px1));
        py1 = std::max(0, std::min(PROTO_H - 1, py1));
        px2 = std::max(px1 + 1, std::min(PROTO_W, px2));
        py2 = std::max(py1 + 1, std::min(PROTO_H, py2));
        cv::Rect prect(px1, py1, px2 - px1, py2 - py1);

        cv::Mat roi160 = mask_sig(prect);
        cv::Mat roi_bbox_mask;
        cv::resize(roi160, roi_bbox_mask, cv::Size(rw, rh), 0, 0, cv::INTER_LINEAR);
        //cv::resize(roi160, roi_bbox_mask, cv::Size(rw, rh), 0, 0, cv::INTER_NEAREST);

        cv::Mat bin = cv::Mat::zeros(CAM_IMAGE_HEIGHT, CAM_IMAGE_WIDTH, CV_8UC1);
        cv::Mat roi_bin;
        cv::threshold(roi_bbox_mask, roi_bin, MASK_THRESH, 255, cv::THRESH_BINARY);
        roi_bin.convertTo(bin(rect), CV_8UC1);

        /* invalid erea clear */
        cv::Rect invalid_mask(0, 0, CAM_IMAGE_WIDTH / md.pr.width * md.pr.disparity, CAM_IMAGE_HEIGHT);
        bin(invalid_mask).setTo(cv::Scalar(0, 0, 0, 0));
        
        det_buff[i].bbox = {cx, cy, bw, bh};
        det_buff[i].mask = bin.clone();
        det_final.push_back(det_buff[i]);
    }
    spdlog::info(" Bounding Box Count  : {}", iBoxCount);

    /* Update the global detection result */
    mtx.lock();
    det = det_final;
    mtx.unlock();
}

/*****************************************
* Function Name : calc_distance_avg
* Description   : Process CPU post-processing for Yolov8
* Arguments     : -
* Return value  : -
******************************************/
void calc_distance_avg(std::vector<detection_seg>& det_buff)
{
    cv::Mat disparity_s16_cam;
    
    mtx.lock();
    disparity_s16_cam = img.get_disparity_s16_cam().clone();
    mtx.unlock();

    for (size_t i = 0; i < det_buff.size(); i++)
    {
        if (det_buff[i].prob == 0) continue;
        
        cv::Mat valid_disp;
        cv::compare(disparity_s16_cam, 0, valid_disp, cv::CMP_GT);

        cv::Mat valid_mask;
        cv::bitwise_and(det_buff[i].mask, valid_disp, valid_mask);

        int valid_count = cv::countNonZero(valid_mask);
        if (valid_count == 0) {
            det_buff[i].avg_disparity = 0.0f;
            det_buff[i].avg_distance_m = -1.0f;
            continue;
        }
        
        float mean_16s = cv::mean(disparity_s16_cam, valid_mask)[0];
        float distance = (CAM_BASE * CAM_F_LENGTH) / (mean_16s * CAM_IMAGE_WIDTH / md.pr.width);
 
        det_buff[i].avg_disparity = mean_16s;
        det_buff[i].avg_distance_m = distance;
    }
}

/*****************************************
* Function Name : calc_distance_median
* Description   : Process CPU post-processing for Yolov8
* Arguments     : -
* Return value  : -
******************************************/
void calc_distance_median(std::vector<detection_seg>& det_buff)
{
    cv::Mat disparity_s16_cam;
    cv::Mat masked;

    mtx.lock();
    disparity_s16_cam = img.get_disparity_s16_cam().clone();
    mtx.unlock();

    for (size_t i = 0; i < det_buff.size(); i++)
    {
        if (det_buff[i].prob == 0) continue;

        // initial
        masked = cv::Mat::zeros(disparity_s16_cam.size(), disparity_s16_cam.type()); 
        disparity_s16_cam.copyTo(masked, det_buff[i].mask);

        cv::Mat flat = masked.reshape(1, 1);

        std::vector<int> idx;
        for (int j = 0; j < flat.cols; j++)
        {
            if (flat.at<short>(j) > 0) idx.push_back(j);
        }

        cv::Mat valid(1, idx.size(), CV_16S);
        for (size_t k = 0; k < idx.size(); k++)
        {
            valid.at<short>(k) = flat.at<short>(idx[k]);
        }

        cv::sort(valid, valid, cv::SORT_ASCENDING);

        if (valid.cols == 0) {
            det_buff[i].median_disparity = 0.0f;
            det_buff[i].median_distance_m = -1.0f;
            continue;
        }

        float median_16s = 0.0f;
        if (valid.cols > 0)
        {
            median_16s = valid.at<short>(valid.cols / 2);
        }
        float distance = (CAM_BASE * CAM_F_LENGTH) / (median_16s * CAM_IMAGE_WIDTH / md.pr.width);
 
        det_buff[i].median_disparity = median_16s;
        det_buff[i].median_distance_m = distance;
    }
}

/*****************************************
* Function Name : draw_segmentation_mask
* Description   : Draw segmentation on image.
* Arguments     : det_buff
* Return value  : 0 if succeeded
*               not 0 otherwise
******************************************/
void draw_segmentation_mask(const std::vector<detection_seg>& det_buff)
{
    int W = img.get_W();
    int H = img.get_H();
    uint8_t id = img.get_buf_id();
    cv::Mat frame_mask(H, W, CV_8UC4, img.get_img(id));

    if (!det_buff.empty())
    {
        cv::Mat mean_color_bgr;
        cv::Mat mean_color_bgra;

        for (size_t i = 0; i < det_buff.size(); i++)
        {
            if (det_buff[i].prob == 0) continue;

            //float mean_16s_0_255 = std::min(det_buff[i].avg_disparity * 256.0 / md.pr.disparity, 255.0);
            float mean_16s_0_255 = std::min(det_buff[i].median_disparity * 256.0 / md.pr.disparity, 255.0);
            cv::Mat mean_pix(1, 1, CV_8UC1, cv::Scalar((uint8_t)std::lround(mean_16s_0_255)));
            cv::applyColorMap(mean_pix, mean_color_bgr, md.pr.heat_map);
            cv::cvtColor(mean_color_bgr, mean_color_bgra, cv::COLOR_BGR2BGRA);
            cv::Vec4b color = mean_color_bgra.at<cv::Vec4b>(0, 0);

            cv::Mat color_layer(H, W, CV_8UC4, cv::Scalar(color[0], color[1], color[2], 255));
            cv::Mat blended;
            cv::addWeighted(frame_mask, 1.0f - MASK_ALPHA, color_layer, MASK_ALPHA, 0.0, blended);
            blended.copyTo(frame_mask, det_buff[i].mask);
        }
    }

    cv::Mat invalid_mask(H, W, CV_8UC4, cv::Scalar(0, 0, 0, 255));
    cv::Rect rect(0, 0, (int)(W / md.pr.width * md.pr.disparity), H);
    invalid_mask(rect).setTo(cv::Scalar(255, 255, 255, 255));

    cv::Mat blended;

#ifdef DISPLAY_CUT_INVALID_EREA
    blended = cv::Mat(H, W, CV_8UC4, cv::Scalar(0,0,0,255));
#else
    cv::Mat color_layer(H, W, CV_8UC4, cv::Scalar(128, 128, 128, 255));
    cv::addWeighted(frame_mask, 1.0f - MASK_ALPHA, color_layer, MASK_ALPHA, 0.0, blended);
#endif  

    blended.copyTo(frame_mask, invalid_mask);

    return;
}

/*****************************************
* Function Name : draw_bounding_box
* Description   : Draw bounding box on image.
* Arguments     : -
* Return value  : 0 if succeeded
*               not 0 otherwise
******************************************/
void draw_bounding_box(const std::vector<detection_seg>& det_buff)
{
    stringstream stream;
    string result_str;
    size_t i = 0;
    uint32_t color=0;
 
    /* Draw bounding box on RGB image. */
    for (i = 0; i < det_buff.size(); i++)
    {
        /* Skip the overlapped bounding boxes */
        if (det_buff[i].prob == 0) continue;
        
        color = box_color[det_buff[i].c];

        // --- text for display ---
        stream.str("");
        stream.clear();
        stream << label_file_map[det_buff[i].c]
               << " "
               << std::fixed
               << std::setprecision(2)
               // << det_buff[i].avg_distance_m
               << det_buff[i].median_distance_m
               << " m";

        // --- text on BBox ---
        std::string text_str = stream.str();        
        img.draw_rect((int)det_buff[i].bbox.x, (int)det_buff[i].bbox.y, (int)det_buff[i].bbox.w, (int)det_buff[i].bbox.h, text_str.c_str(), color);
    }
    return;
}

/*****************************************
* Function Name : gray_left_disparity
* Description   : 
* Arguments     : 
* Return value  : -
******************************************/
void gray_left_disparity(uint8_t* img,
                         int width,
                         int height,
                         int channels,
                         int disparity)
{
    int stride = width * channels;

    for (int y = 0; y < height; y++)
    {
        uint8_t* line = img + y * stride;

        for (int x = 0; x < disparity; x++)
        {
            line[x * channels + 0] = 114;
            line[x * channels + 2] = 114;

            line[x * channels + 1] = 128;
            line[x * channels + 3] = 128;
        }
    }
}

/*****************************************
* Function Name : BGRtoYUYV
* Description   : BGR image to YUYV image
* Arguments     : bgr = cv::Mat read image
                  yuyvBuffer = vector<uint8_t>
* Return value  : -
******************************************/
void BGRtoYUYV(const cv::Mat& bgr, std::vector<uint8_t>& yuyvBuffer) 
{
    int width = bgr.cols;
    int height = bgr.rows;
    yuyvBuffer.resize(width * height * 2);
    uint8_t* yuyvPtr = yuyvBuffer.data();
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; j += 2) {
            cv::Vec3b bgr1 = bgr.at<cv::Vec3b>(i, j);
            cv::Vec3b bgr2 = bgr.at<cv::Vec3b>(i, j + 1);

            uint8_t Y1 = static_cast<uint8_t>(0.299 * bgr1[2] + 0.587 * bgr1[1] + 0.114 * bgr1[0]);
            uint8_t Y2 = static_cast<uint8_t>(0.299 * bgr2[2] + 0.587 * bgr2[1] + 0.114 * bgr2[0]);
            uint8_t U = static_cast<uint8_t>(-0.169 * bgr1[2] - 0.331 * bgr1[1] + 0.5 * bgr1[0] + 128);
            uint8_t V = static_cast<uint8_t>(0.5 * bgr1[2] - 0.419 * bgr1[1] - 0.081 * bgr1[0] + 128);

            *yuyvPtr++ = Y1;
            *yuyvPtr++ = U;
            *yuyvPtr++ = Y2;
            *yuyvPtr++ = V;
        }
    }
}

/*****************************************
* Function Name : print_result
* Description   : print the result on display.
* Arguments     : -
* Return value  : 0 if succeeded
*               not 0 otherwise
******************************************/
int8_t print_result(const std::vector<detection_seg>& det_buff, Image* img)
{
#ifdef DEBUG_TIME_FLG
    using namespace std;
    chrono::system_clock::time_point start, end;
    start = chrono::system_clock::now();
#endif // DEBUG_TIME_FLG
    stringstream stream;
    string str = "";
    double total_time = ai_time + pre_time + post_time;
    double ai_inf_thread_time = ai_time + pre_time + post_time + wait_stereo_time + post_stereo_time;
    double stereo_thread_time = stereo_time + pre_stereo_time;
   
    /* Draw AI INF THREAD Time Result on RGB image.*/
    stream.str("");
    stream << "AI Inf Thread Time : " << std::setw(3) << std::fixed << std::setprecision(1) << std::round(ai_inf_thread_time * 10) / 10 << "msec";
    str = stream.str();
    img->write_string_rgb(str, 2, TEXT_WIDTH_OFFSET,  LINE_HEIGHT_OFFSET + (LINE_HEIGHT * 1), CHAR_SCALE_LARGE, 0x00FF00u);

    /* Draw Total Time Result on RGB image.*/
    stream.str("");
    stream << "Total AI Time : " << std::setw(3) << std::fixed << std::setprecision(1) << std::round(total_time * 10) / 10 << "msec";
    str = stream.str();
    img->write_string_rgb(str, 2, TEXT_WIDTH_OFFSET,  LINE_HEIGHT_OFFSET + (LINE_HEIGHT * 2), CHAR_SCALE_LARGE, 0xFFF000u);
 
    /* Draw Inference Time on RGB image.*/
    stream.str("");
    stream << "  Inference   : " << std::setw(3) << std::fixed << std::setprecision(1) << std::round(ai_time * 10) / 10 << "msec";
    str = stream.str();
    img->write_string_rgb(str, 2, TEXT_WIDTH_OFFSET, LINE_HEIGHT_OFFSET + (LINE_HEIGHT * 3), CHAR_SCALE_LARGE, 0xFFF000u);

    /* Draw PreProcess Time on RGB image.*/
    stream.str("");
    stream << "  PreProcess  : " << std::setw(3) << std::fixed << std::setprecision(1) << std::round(pre_time * 10) / 10 << "msec";
    str = stream.str();
    img->write_string_rgb(str, 2, TEXT_WIDTH_OFFSET, LINE_HEIGHT_OFFSET + (LINE_HEIGHT * 4), CHAR_SCALE_LARGE, 0xFFF000u);

    /* Draw PostProcess Time on RGB image.*/
    stream.str("");
    stream << "  PostProcess : " << std::setw(3) << std::fixed << std::setprecision(1) << std::round(post_time * 10) / 10 << "msec";
    str = stream.str();
    img->write_string_rgb(str, 2, TEXT_WIDTH_OFFSET, LINE_HEIGHT_OFFSET + (LINE_HEIGHT * 5), CHAR_SCALE_LARGE, 0xFFF000u);

    /* WaitStereoSGM Time on RGB image.*/
    stream.str("");
    stream << "  WaitStereo : " << std::setw(3) << std::fixed << std::setprecision(1) << std::round(wait_stereo_time * 10) / 10 << "msec";
    str = stream.str();
    img->write_string_rgb(str, 2, TEXT_WIDTH_OFFSET, LINE_HEIGHT_OFFSET + (LINE_HEIGHT * 7), CHAR_SCALE_LARGE, 0xFFF000u);
    /* PostStereoSGM Time on RGB image.*/
    stream.str("");
    stream << "  PostStereo : " << std::setw(3) << std::fixed << std::setprecision(1) << std::round(post_stereo_time * 10) / 10 << "msec";
    str = stream.str();
    img->write_string_rgb(str, 2, TEXT_WIDTH_OFFSET, LINE_HEIGHT_OFFSET + (LINE_HEIGHT * 8), CHAR_SCALE_LARGE, 0xFFF000u);

    /* StreoDraw AI INF THREAD Time Result on RGB image.*/
    stream.str("");
    stream << "Stereo Thread Time : " << std::setw(3) << std::fixed << std::setprecision(1) << std::round(stereo_thread_time * 10) / 10 << "msec";
    str = stream.str();
    img->write_string_rgb(str, 2, TEXT_WIDTH_OFFSET,  LINE_HEIGHT_OFFSET + (LINE_HEIGHT * 10), CHAR_SCALE_LARGE, 0x00FF00u);

    /* StereoSGM Time on RGB image.*/
    stream.str("");
    stream << "  StereoSGM : " << std::setw(3) << std::fixed << std::setprecision(1) << std::round(stereo_time * 10) / 10 << "msec";
    str = stream.str();
    img->write_string_rgb(str, 2, TEXT_WIDTH_OFFSET, LINE_HEIGHT_OFFSET + (LINE_HEIGHT * 11), CHAR_SCALE_LARGE, 0xFFF000u);

    /* PreStereoSGM Time on RGB image.*/
    stream.str("");
    stream << "  PreStereo : " << std::setw(3) << std::fixed << std::setprecision(1) << std::round(pre_stereo_time * 10) / 10 << "msec";
    str = stream.str();
    img->write_string_rgb(str, 2, TEXT_WIDTH_OFFSET, LINE_HEIGHT_OFFSET + (LINE_HEIGHT * 12), CHAR_SCALE_LARGE, 0xFFF000u);


    /* Display TSU value */
    FILE *fp;
    char buff[16]="";
    float tsu_value;
    fp = fopen("/sys/class/thermal/thermal_zone1/temp", "r");
    if (fgets(buff, 16, fp) == nullptr)
    {
        fprintf(stderr, "[ERROR] Failed to read thermal Value\n");
        tsu_value = 999.9;
    } else {
        tsu_value = (float)atoi(buff)/1000;
    }
    fclose(fp);

    stream.str("");
    stream << "Temperature : " << std::setw(3) << std::fixed << std::setprecision(1) << std::round(tsu_value * 10) / 10 << "degC";
    str = stream.str();
    img->write_string_rgb(str, 2, TEXT_WIDTH_OFFSET, LINE_HEIGHT_OFFSET + (LINE_HEIGHT * 14), CHAR_SCALE_LARGE, 0xFFF000u);


#ifdef DISP_AI_FRAME_RATE
    /* Draw AI/Camera Frame Rate on RGB image.*/
    stream.str("");
    stream << "AI/Camera Frame Rate: " << std::setw(3) << (uint32_t)ai_fps << "/" << (uint32_t)cap_fps << "fps";
    str = stream.str();
    img->write_string_rgb(str, 2, TEXT_WIDTH_OFFSET, LINE_HEIGHT_OFFSET + (LINE_HEIGHT * 16), CHAR_SCALE_LARGE, 0xFFF000u);
#endif /* DISP_AI_FRAME_RATE */

#ifdef DEBUG_TIME_FLG
    end = chrono::system_clock::now();
    double time = static_cast<double>(chrono::duration_cast<chrono::microseconds>(end - start).count() / 1000.0);
    printf("Draw Text Time            : %lf[ms]\n", time);
#endif // DEBUG_TIME_FLG
//#ifdef CAM_INPUT_VGA
#if defined(CAM_INPUT_VGA) || defined(CAM_INPUT_HD)
    /* Draw the detected results*/
    for (size_t i = 0, num=1; i < det_buff.size(); i++)
    {   
        uint32_t color = box_color[det_buff[i].c];
        if (det_buff[i].prob != 0)
        {
            stream.str("");
            stream << label_file_map[det_buff[i].c].c_str() << " " << std::setw(5) << std::fixed << std::setprecision(1) << round(det_buff[i].prob*100) << "%";
            //stream << " " << std::setprecision(2) << det_buff[i].avg_distance_m << "m";
            stream << " " << std::setprecision(2) << det_buff[i].median_distance_m << "m";
            str = stream.str();
            img->write_string_rgb(str, 1, TEXT_WIDTH_OFFSET*5, LINE_HEIGHT_OFFSET + (LINE_HEIGHT * num), CHAR_SCALE_SMALL, color);
            num++;
        }
    }
#endif
    return 0;
}

/*****************************************
* Function Name : R_Inf_Thread
* Description   : Executes the DRP-AI inference thread
* Arguments     : threadid = thread identification
* Return value  : -
******************************************/
void *R_Inf_Thread(void *threadid)
{
    /*Semaphore Variable*/
    int32_t inf_sem_check = 0;
    int32_t inf_cnt = -1;
    
    /*Variable for getting Inference output data*/
    void* output_ptr;
    uint32_t out_size;
    /*Variable for Pre-processing parameter configuration*/
    s_preproc_param_t in_param;

    /*Variable for checking return value*/
    int8_t ret = 0;
    /*Variable for Performance Measurement*/

    static struct timespec inf_start_time;
    static struct timespec inf_end_time;
    static struct timespec pre_start_time;
    static struct timespec pre_end_time;
    static struct timespec post_start_time;
    static struct timespec post_end_time;
    static struct timespec drp_prev_time = { .tv_sec = 0, .tv_nsec = 0, };

    static struct timespec wait_stereo_start_time;
    static struct timespec wait_stereo_end_time;
    static struct timespec post_stereo_start_time;
    static struct timespec post_stereo_end_time;

    printf("Inference Thread Starting\n");
    printf("Inference Loop Starting\n");
    /*Inference Loop Start*/
    while(1)
    {
        inf_cnt++;
        spdlog::info("[START] Start DRP-AI Inference...");
        spdlog::info("Inference ----------- No. {}", (inf_cnt + 1));

        while(1)
        {
            /*Gets the Termination request semaphore value. If different then 1 Termination was requested*/
            /*Checks if sem_getvalue is executed wihtout issue*/
            errno = 0;
            ret = sem_getvalue(&terminate_req_sem, &inf_sem_check);
            if (0 != ret)
            {
                fprintf(stderr, "[ERROR] Failed to get Semaphore Value: errno=%d\n", errno);
                goto err;
            }
            /*Checks the semaphore value*/
            if (1 != inf_sem_check)
            {
                goto ai_inf_end;
            }
            /*Checks if image frame from Capture Thread is ready.*/
            if (inference_start.load())
            {
                break;
            }
            usleep(WAIT_TIME);
        }

        in_param.pre_in_addr    = capture_address;
        in_param.input_copy_enabled = false;
        
        /*Gets Pre-process starting time*/
        ret = timespec_get(&pre_start_time, TIME_UTC);
        if (0 == ret)
        {
            fprintf(stderr, "[ERROR] Failed to get Pre-process Start Time\n");
            goto err;
        }
        ret = preruntime.Pre(&in_param, &output_ptr, &out_size);
        if (0 < ret)
        {
            fprintf(stderr, "[ERROR] Failed to run Pre-processing Runtime Pre()\n");
            goto err;
        }
        /*Gets AI Pre-process End Time*/
        ret = timespec_get(&pre_end_time, TIME_UTC);
        if ( 0 == ret)
        {
            fprintf(stderr, "[ERROR] Failed to Get Pre-process End Time\n");
            goto err;
        }
        /*Set Pre-processing output to be inference input. */
        runtime.SetInput(0, (float*)output_ptr);

        /*Pre-process Time Result*/
        pre_time = (timedifference_msec(pre_start_time, pre_end_time) * TIME_COEF);

        /*Gets inference starting time*/
        ret = timespec_get(&inf_start_time, TIME_UTC);
        if (0 == ret)
        {
            fprintf(stderr, "[ERROR] Failed to get Inference Start Time\n");
            goto err;
        }

        runtime.Run(drpai_freq);

        /*Gets AI Inference End Time*/
        ret = timespec_get(&inf_end_time, TIME_UTC);
        if ( 0 == ret)
        {
            fprintf(stderr, "[ERROR] Failed to Get Inference End Time\n");
            goto err;
        }
        /*Inference Time Result*/
        ai_time = (timedifference_msec(inf_start_time, inf_end_time) * TIME_COEF);

        /*Gets Post-process starting time*/
        ret = timespec_get(&post_start_time, TIME_UTC);
        if (0 == ret)
        {
            fprintf(stderr, "[ERROR] Failed to get Post-process Start Time\n");
            goto err;
        }

        /*Process to read the DRPAI output data.*/
        ret = get_result();
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to get result from memory.\n");
            goto err;
        }

        /*Preparation for Post-Processing*/
        /*CPU Post-Processing For YOLOv8-seg */
        R_Post_Proc_Seg(output_dfl80,  output_dfl40,    output_dfl20,
                        output_class80, output_class40, output_class20,
                        output_mask80,  output_mask40,  output_mask20);

        /* R_Post_Proc time end*/
        ret = timespec_get(&post_end_time, TIME_UTC);
        if (0 == ret)
        {
            fprintf(stderr, "[ERROR] Failed to Get R_Post_Proc End Time\n");
            goto err;
        }
        post_time = (timedifference_msec(post_start_time, post_end_time)*TIME_COEF);

        /*Gets Wait for stereo time*/
        ret = timespec_get(&wait_stereo_start_time, TIME_UTC);
        if (0 == ret)
        {
            fprintf(stderr, "[ERROR] Failed to Get Wait stereo Start Time\n");
            goto err;
        }

        while(stereo_start.load() == 1){}

        /* Wait for stereo time end*/
        ret = timespec_get(&wait_stereo_end_time, TIME_UTC);
        if (0 == ret)
        {
            fprintf(stderr, "[ERROR] Failed to Get Wait stereo End Time\n");
            goto err;
        }
        wait_stereo_time = (timedifference_msec(wait_stereo_start_time, wait_stereo_end_time)*TIME_COEF);

        /*Gets Post stereo time*/
        ret = timespec_get(&post_stereo_start_time, TIME_UTC);
        if (0 == ret)
        {
            fprintf(stderr, "[ERROR] Failed to Get Post stereo Start Time\n");
            goto err;
        }
        
        //calc_distance_avg(det);
        calc_distance_median(det);

        inference_start.store(0);

        mtx.lock();
        det_ai_stereo = det;  // Snapshot for segmentation and bounding
        mtx.unlock();
#ifdef INPUT_IMAGE
        post_proc_end.store(1);

        if( inf_1st_ready > 0 )
        {
                inf_1st_ready--;
        }
#endif

        /* Post stereo time end*/
        ret = timespec_get(&post_stereo_end_time, TIME_UTC);
        if (0 == ret)
        {
            fprintf(stderr, "[ERROR] Failed to Get Post stereo End Time\n");
            goto err;
        }
        post_stereo_time = (timedifference_msec(post_stereo_start_time, post_stereo_end_time)*TIME_COEF);
        // post_time = 0.05*(timedifference_msec(post_start_time, post_end_time)*TIME_COEF)+0.95*post_time;

        /*Display Processing Time On Log File*/
        drpai_time = timedifference_msec(inf_start_time, inf_end_time) * TIME_COEF;
        int idx = inf_cnt % SIZE_OF_ARRAY(array_drp_time);
        array_drp_time[idx] = ai_time;
        drp_prev_time = inf_end_time;
        double total_time = ai_time + pre_time + post_time;
        spdlog::info("Total AI Time  : {} [ms]", std::round(total_time * 10) / 10);
        spdlog::info("PreProcess     : {} [ms]", std::round(pre_time   * 10) / 10);
        spdlog::info("Inference      : {} [ms]", std::round(ai_time    * 10) / 10);
        spdlog::info("PostProcess: {} [ms]", std::round(post_time * 10) / 10);

#ifdef DISP_AI_FRAME_RATE
        int arraySum = std::accumulate(array_drp_time, array_drp_time + SIZE_OF_ARRAY(array_drp_time), 0);
        double arrayAvg = 1.0 * arraySum / SIZE_OF_ARRAY(array_drp_time);
        ai_fps = 1.0 / arrayAvg * 1000.0 + 0.5;
        spdlog::info("AI Frame Rate {} [fps]", (int32_t)ai_fps);
#endif /* DISP_AI_FRAME_RATE */
    }
    /*End of Inference Loop*/

/*Error Processing*/
err:
    /*Set Termination Request Semaphore to 0*/
    sem_trywait(&terminate_req_sem);
    goto ai_inf_end;
/*AI Thread Termination*/
ai_inf_end:

#ifdef INPUT_IMAGE
    post_proc_end.store(1);
#endif

    /*To terminate the loop in Capture Thread.*/
    printf("AI Inference Thread Terminated\n");
    pthread_exit(NULL);
}

/*****************************************
 * Function Name : R_Stereo_Thread
 * Description   : Executes the DRP-AI stereo thread
 * Arguments     : threadid = thread identification
 * Return value  : -
 ******************************************/
void *R_Stereo_Thread(void *threadid)
{
    /*Semaphore Variable*/
    int32_t stereo_sem_check = 0;

    /*Variable for checking return value*/
    int8_t ret = 0;

    static struct timespec stereo_start_time;
    static struct timespec stereo_end_time;
    static struct timespec pre_stereo_start_time;
    static struct timespec pre_stereo_end_time;

    printf("Stereo Thread Starting\n");

    int sgbm_mode[5] = {cv::StereoSGBM::MODE_SGBM, cv::StereoSGBM::MODE_HH, cv::StereoSGBM::MODE_SGBM_3WAY,
                        cv::StereoSGBM::MODE_HH4, cv::StereoSGBM::MODE_SGM_DRP};

    int mode;
    int minDisparity = 0;
    int numDisparities = md.pr.disparity;
    int blockSize = md.pr.win_x;  // (=win_y)
    int P1 = md.pr.p1;
    int P2 = md.pr.p2;
    int disp12MaxDiff = 0;
    int preFilterCap = 0;
    int uniquenessRatio = md.pr.uniquenessRatio;
    int speckleWindowSize = 0;
    int speckleRange = 0;
    cv::Mat src1_gray(md.pr.height, md.pr.width, CV_8UC1);
    cv::Mat src2_gray(md.pr.height, md.pr.width, CV_8UC1);
    cv::Mat stereo_out(md.pr.height, md.pr.width, CV_16SC1); /* disparity */


    if(md.mode == "oca")
    {
        md.mode_param = 1.0;
        mode = sgbm_mode[4];
        OCA_list[OCA_FUNC_STEREOSGM] = OPENCVA_FUNC_ENABLE;
    } else {
        md.mode_param = 16.0;
        mode = sgbm_mode[0];
        OCA_list[OCA_FUNC_STEREOSGM] = OPENCVA_FUNC_DISABLE;
    }
    OCA_Activate(&OCA_list[0]);
    
    cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(minDisparity, numDisparities, blockSize, P1, P2,
                                                            disp12MaxDiff, preFilterCap, uniquenessRatio, speckleWindowSize, speckleRange, mode);

    while(1)
    {
        spdlog::info("[START] Start Stereo...");
 
        while(1)
        {
            /*Gets the Termination request semaphore value. If different then 1 Termination was requested*/
            /*Checks if sem_getvalue is executed wihtout issue*/
            errno = 0;
            ret = sem_getvalue(&terminate_req_sem, &stereo_sem_check);
            if (0 != ret)
            {
                fprintf(stderr, "[ERROR] Failed to get Semaphore Value: errno=%d\n", errno);
                goto err;
            }
            /*Checks the semaphore value*/
            if (1 != stereo_sem_check)
            {
                goto stereo_end;
            }
            /*Checks if image frame from Capture Thread is ready.*/
            if (stereo_start.load())
            {
                break;
            }
            usleep(WAIT_TIME);
        }

        ret = timespec_get(&pre_stereo_start_time, TIME_UTC);
        img.convert_format_s_image(md.df.cam_calib_en, 0);
        img1.convert_format_s_image(md.df.cam_calib_en, 1);
            
        src1_gray.data = img.get_s_image();
        src2_gray.data = img1.get_s_image();
        ret = timespec_get(&pre_stereo_end_time, TIME_UTC);
        pre_stereo_time = (timedifference_msec(pre_stereo_start_time, pre_stereo_end_time) * TIME_COEF);

        ret = timespec_get(&stereo_start_time, TIME_UTC);
        stereo->compute(src1_gray, src2_gray, stereo_out);
        img.set_stereo_out(stereo_out);
        
        ret = timespec_get(&stereo_end_time, TIME_UTC);
        stereo_time = (timedifference_msec(stereo_start_time, stereo_end_time) * TIME_COEF);

#ifndef INPUT_IMAGE // INPUT_CAMERA
        if(md.pr.focus_en == 1) {
            if(md.focus_cnt == 0)                        md.focus_st = 0;
            else if(md.focus_cnt <  md.pr.focus_cnt_max) md.focus_st = 1;
            else if(md.focus_cnt == md.pr.focus_cnt_max) md.focus_st = 2;
            else                                         md.focus_st = 3; 

            if(md.focus_st < 3) {
                img.adjust_focus();
                md.focus_cnt++;
            }
        }
#endif

        stereo_start.store(0);      
    }
    /*End of Inference Loop*/

/*Error Processing*/
err:
    /*Set Termination Request Semaphore to 0*/
    sem_trywait(&terminate_req_sem);
    goto stereo_end;
/*Stereo Thread Termination*/
stereo_end:
    stereo_start.store(0);

    /*To terminate the loop in Capture Thread.*/
    printf("Stereo Thread Terminated\n");
    pthread_exit(NULL);
}
    
/*****************************************
* Function Name : R_Capture_Thread
* Description   : Executes the V4L2 capture with Capture thread.
* Arguments     : threadid = thread identification
* Return value  : -
******************************************/
void *R_Capture_Thread(void *arg)
{
#ifndef INPUT_IMAGE  // INPUT_CAMERA
    capture_arg_t *p = (capture_arg_t *)arg;
    Camera2* capture  = p->capture;
    Camera2* capture1 = p->capture1;
    uint8_t capture_stabe_cnt = 8;  // Counter to wait for the camera to stabilize
#else
    capture_arg_t *p = (capture_arg_t *)arg;
    Input_Files* capture  = p->capture;
    Input_Files* capture1 = p->capture1;
    uint8_t capture_stabe_cnt = 0;
    size_t idx_in = 0;
    cv::Mat bgr_cam_size(CAM_IMAGE_HEIGHT, CAM_IMAGE_WIDTH, CV_8UC3);
    cv::Mat bgr_cam_size1(CAM_IMAGE_HEIGHT, CAM_IMAGE_WIDTH, CV_8UC3);
    std::vector<uint8_t> yuyvBuffer;
    std::vector<uint8_t> yuyvBuffer1;
#endif
    
    /*Semaphore Variable*/
    int32_t capture_sem_check = 0;
    /*First Loop Flag*/
    uint64_t capture_addr = 0;
    uint64_t capture_addr1 = 0;
    int8_t ret = 0;
    uint8_t * img_buffer;
    uint8_t * img_buffer0;
    uint8_t * img_buffer1;

#ifdef DISP_AI_FRAME_RATE
    int32_t cap_cnt = -1;
    static struct timespec capture_time;
    static struct timespec capture_time_prev = { .tv_sec = 0, .tv_nsec = 0, };
#endif /* DISP_AI_FRAME_RATE */

    printf("Capture Thread Starting\n");

    img_buffer0 = (uint8_t *)capture->drpai_buf->mem;

#if (1) == DRPAI_INPUT_PADDING
    /** Fill buffer with the brightness 114. */
    for( uint32_t i = 0; i < CAM_IMAGE_WIDTH * CAM_IMAGE_WIDTH * CAM_IMAGE_CHANNEL_YUY2; i += 4 )
    {
        /// Y =  0.299R + 0.587G + 0.114B
        img_buffer0[i]   = 114;    
        img_buffer0[i+2] = 114;
        /// U = -0.169R - 0.331G + 0.500B + 128
        img_buffer0[i+1] = 128;
        /// V =  0.500R - 0.419G - 0.081B + 128
        img_buffer0[i+3] = 128;
    }
#endif  /* (1) == DRPAI_INPUT_PADDING */
    capture_address = capture->drpai_buf->phy_addr;

    while(1)
    {
        /*Gets the Termination request semaphore value. If different then 1 Termination was requested*/
        /*Checks if sem_getvalue is executed wihtout issue*/
        errno = 0;
        ret = sem_getvalue(&terminate_req_sem, &capture_sem_check);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to get Semaphore Value: errno=%d\n", errno);
            goto err;
        }
        /*Checks the semaphore value*/
        if (1 != capture_sem_check)
        {
            goto capture_end;
        }

#ifndef INPUT_IMAGE
        /* Capture USB camera image and stop updating the capture buffer */
        capture_addr  = (uint32_t)capture->capture_image();
        capture_addr1 = (uint32_t)capture1->capture_image();
#else
        capture->read_file(idx_in, bgr_cam_size);
        capture1->read_file(idx_in, bgr_cam_size1);
        BGRtoYUYV(bgr_cam_size,yuyvBuffer);
        BGRtoYUYV(bgr_cam_size1,yuyvBuffer1);
        capture_addr = (uint32_t)0xffffffff;
        capture_addr1 = (uint32_t)0xffffffff;
#endif
        
#ifdef DISP_AI_FRAME_RATE
        cap_cnt++;
        ret = timespec_get(&capture_time, TIME_UTC);
        proc_time_capture = (timedifference_msec(capture_time_prev, capture_time) * TIME_COEF);
        capture_time_prev = capture_time;

        int idx = cap_cnt % SIZE_OF_ARRAY(array_cap_time);
        array_cap_time[idx] = (uint32_t)proc_time_capture;
        int arraySum = std::accumulate(array_cap_time, array_cap_time + SIZE_OF_ARRAY(array_cap_time), 0);
        double arrayAvg = 1.0 * arraySum / SIZE_OF_ARRAY(array_cap_time);
        cap_fps = 1.0 / arrayAvg * 1000.0 + 0.5;
#endif /* DISP_AI_FRAME_RATE */


        if ((capture_addr == 0) || (capture_addr1 == 0))
        {
            fprintf(stderr, "[ERROR] Failed to capture image from camera.\n");
            goto err;
        }
        else
        {
            /* Do not process until the camera stabilizes, because the image is unreliable until the camera stabilizes. */
            if( capture_stabe_cnt > 0 )
            {
                capture_stabe_cnt--;
            }
            else
            {
#ifndef INPUT_IMAGE
                img_buffer = capture->get_img();
                img_buffer1 = capture1->get_img();
#else
                img_buffer = yuyvBuffer.data();
                img_buffer1 = yuyvBuffer1.data();
#endif                
                if (!inference_start.load() && !stereo_start.load())
                {
                    img.camera_to_s_image(img_buffer, capture->get_size());
                    img1.camera_to_s_image(img_buffer1, capture1->get_size());                    
                    stereo_start.store(1); /* Flag for AI Inference Thread. */

                    /* Copy captured image to Image object. This will be used in Display Thread. */
                    memcpy(img_buffer0, img_buffer, capture->get_size());

#ifdef DISPLAY_CUT_INVALID_EREA
                    gray_left_disparity(img_buffer0, CAM_IMAGE_WIDTH, CAM_IMAGE_HEIGHT, CAM_IMAGE_CHANNEL_YUY2,
                                        CAM_IMAGE_WIDTH / md.pr.width * md.pr.disparity);
#endif
                
                    /* Flush capture image area cache */
                    ret = buffer_flush_dmabuf(capture->drpai_buf->idx, capture->drpai_buf->size);
                    if (0 != ret)
                    {
                        goto err;
                    }
                    inference_start.store(1); /* Flag for AI Inference Thread. */
                }

#ifdef INPUT_IMAGE
                if(md.pr.inf_wait_en == 1)
                {
                    while(post_proc_end.load() == 0){}
                    post_proc_end.store(0);
                }
                if(output_en == 0) {
                    idx_in++;
                    if(idx_in >= capture->get_files_size() ) idx_in = 0;
                }
#endif

                if (!img_obj_ready.load())
                {
                    img.camera_to_image(img_buffer, capture->get_size());
                    ret = buffer_flush_dmabuf(capture->wayland_buf->idx, capture->wayland_buf->size);
                    if (0 != ret)
                    {
                        goto err;
                    }
                    img_obj_ready.store(1); /* Flag for Display Thread. */
                }
            }
        }
        
#ifndef INPUT_IMAGE
        /* IMPORTANT: Place back the image buffer to the capture queue */
        ret = capture->capture_qbuf();
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to enqueue capture buffer.\n");
            goto err;
        }
        ret = capture1->capture_qbuf();
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to enqueue capture buffer.\n");
            goto err;
        }
#endif                
    } /*End of Loop*/

/*Error Processing*/
err:
    sem_trywait(&terminate_req_sem);
    goto capture_end;

capture_end:
    /*To terminate the loop in AI Inference Thread.*/
    inference_start.store(1);
    //stereo_start.store(1);
    stereo_start.store(0);

    printf("Capture Thread Terminated\n");
    pthread_exit(NULL);
}

/*****************************************
* Function Name : R_Img_Thread
* Description   : Executes img proc with img thread
* Arguments     : threadid = thread identification
* Return value  : -
******************************************/
void *R_Img_Thread(void *threadid)
{
    /*Semaphore Variable*/
    int32_t hdmi_sem_check = 0;
    /*Variable for checking return value*/
    int8_t ret = 0;
    bool padding = false;
//#ifdef CAM_INPUT_VGA
#if defined(CAM_INPUT_VGA) || defined(CAM_INPUT_HD)
    padding = true;
#endif // CAM_INPUT_VGA
    timespec start_time;
    timespec end_time;

    printf("Image Thread Starting\n");
    while(1)
    {
        /*Gets The Termination Request Semaphore Value, If Different Then 1 Termination Is Requested*/
        /*Checks If sem_getvalue Is Executed Without Issue*/
        errno = 0;
        ret = sem_getvalue(&terminate_req_sem, &hdmi_sem_check);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to get Semaphore Value: errno=%d\n", errno);
            goto err;
        }
        /*Checks the semaphore value*/
        if (1 != hdmi_sem_check)
        {
            goto hdmi_end;
        }
        /* Check img_obj_ready flag which is set in Capture Thread. */
        if (img_obj_ready.load())
        {
            ret = timespec_get(&start_time, TIME_UTC);
            if (0 == ret)
            {
                fprintf(stderr, "[ERROR] Failed to get Display Start Time\n");
                goto err;
            }
                        
            mtx.lock();
            det_display = det_ai_stereo;  // Snapshot for segmentation and bounding
            mtx.unlock();

            /* Convert YUYV image to BGRA format. */
            img.convert_format();

            /* Sort */
            std::sort(det_display.begin(), det_display.end(), [](const detection_seg& a, const detection_seg& b)
            {
                // return a.avg_distance_m > b.avg_distance_m;
                return a.median_distance_m > b.median_distance_m;
            });

            /* Draw segmentation mask on image. */
            draw_segmentation_mask(det_display);
    
            /* Draw bounding box on image. */
            draw_bounding_box(det_display);
            
            /* Convert output image size. */
            img.convert_size(CAM_IMAGE_WIDTH, CAM_RESIZED_WIDTH, CAM_IMAGE_HEIGHT, CAM_RESIZED_HEIGHT, padding);

            /* Draw stereo on image. */
            img.draw_stereo_out();

            /* Sort */
            std::sort(det_display.begin(), det_display.end(), [](const detection_seg& a, const detection_seg& b)
            {
                // return a.avg_distance_m < b.avg_distance_m;
                return a.median_distance_m < b.median_distance_m;
            });
            
        	/*displays AI Inference Results on display.*/
            print_result(det_display, &img);

            buf_id = img.get_buf_id();

#ifdef INPUT_IMAGE
            /* output image. */
            if((output_en == 1) && (inf_1st_ready == 0))
            {
                std::cout << "Output file : " << md.df.output << std::endl;
                cv::Mat out_image(IMAGE_OUTPUT_HEIGHT, IMAGE_OUTPUT_WIDTH, CV_8UC4, img.get_img(buf_id));
                cv::imwrite(md.df.output, out_image);
                output_en = 0;
            }
#endif
            
            img_obj_ready.store(0);

            if (!hdmi_obj_ready.load())
            {
                hdmi_obj_ready.store(1); /* Flag for AI Inference Thread. */
            }
            
            ret = timespec_get(&end_time, TIME_UTC);
            if (0 == ret)
            {
                fprintf(stderr, "[ERROR] Failed to Get Display End Time\n");
                goto err;
            }
            
#ifdef DEBUG_TIME_FLG
            double img_proc_time = (timedifference_msec(start_time, end_time) * TIME_COEF);
            printf("Img Proc Time             : %lf[ms]\n", img_proc_time);
#endif
        }
        usleep(WAIT_TIME); //wait 1 tick time
    } /*End Of Loop*/

/*Error Processing*/
err:
    /*Set Termination Request Semaphore To 0*/
    sem_trywait(&terminate_req_sem);
    goto hdmi_end;

hdmi_end:
    /*To terminate the loop in Capture Thread.*/
    img_obj_ready.store(0);
    printf("Img Thread Terminated\n");
    pthread_exit(NULL);
}

/*****************************************
* Function Name : R_Display_Thread
* Description   : Executes the HDMI Display with Display thread
* Arguments     : threadid = thread identification
* Return value  : -
******************************************/
void *R_Display_Thread(void *threadid)
{
    /*Semaphore Variable*/
    int32_t hdmi_sem_check = 0;
    /*Variable for checking return value*/
    int8_t ret = 0;
    int32_t disp_cnt = 0;

    timespec start_time;
    timespec end_time;
    static struct timespec disp_prev_time = { .tv_sec = 0, .tv_nsec = 0, };

    /* Initialize waylad */
    ret = wayland.init(g_capture->wayland_buf->idx, IMAGE_OUTPUT_WIDTH, IMAGE_OUTPUT_HEIGHT, IMAGE_CHANNEL_BGRA);

    if(0 != ret)
    {
        fprintf(stderr, "[ERROR] Failed to initialize Image for Wayland\n");
        goto err;
    }

    printf("Display Thread Starting\n");
    while(1)
    {
        /*Gets The Termination Request Semaphore Value, If Different Then 1 Termination Is Requested*/
        /*Checks If sem_getvalue Is Executed Without Issue*/
        errno = 0;
        ret = sem_getvalue(&terminate_req_sem, &hdmi_sem_check);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to get Semaphore Value: errno=%d\n", errno);
            goto err;
        }
        /*Checks the semaphore value*/
        if (1 != hdmi_sem_check)
        {
            goto hdmi_end;
        }
        /* Check hdmi_obj_ready flag which is set in Capture Thread. */
        if (hdmi_obj_ready.load())
        {
            ret = timespec_get(&start_time, TIME_UTC);
            if (0 == ret)
            {
                fprintf(stderr, "[ERROR] Failed to get Display Start Time\n");
                goto err;
            }
            /*Update Wayland*/
            wayland.commit(img.get_img(buf_id), NULL);

#if END_DET_TYPE // To display the app_pointer_det in front of this application.
            if (display_state == 0) 
            {
                display_state = 1;
            }
#endif

            hdmi_obj_ready.store(0);
            ret = timespec_get(&end_time, TIME_UTC);
            if (0 == ret)
            {
                fprintf(stderr, "[ERROR] Failed to Get Display End Time\n");
                goto err;
            }
            disp_time = (uint32_t)((timedifference_msec(disp_prev_time, end_time) * TIME_COEF));
            int idx = disp_cnt++ % SIZE_OF_ARRAY(array_disp_time);
            array_disp_time[idx] = disp_time;
            disp_prev_time = end_time;
#ifdef DEBUG_TIME_FLG
            double disp_proc_time = (timedifference_msec(start_time, end_time) * TIME_COEF);
            /* Draw Disp Frame Rate on RGB image.*/
            int arraySum = std::accumulate(array_disp_time, array_disp_time + SIZE_OF_ARRAY(array_disp_time), 0);
            double arrayAvg = 1.0 * arraySum / SIZE_OF_ARRAY(array_disp_time);
            double disp_fps = 1.0 / arrayAvg * 1000.0;

            printf("Disp Proc Time            : %lf[ms]\n", disp_proc_time);
            printf("Disp Frame Rate           : %lf[fps]\n", disp_fps);
            printf("Dipslay ------------------------------ No. %d\n", disp_cnt);
#endif
        }
        usleep(WAIT_TIME); //wait 1 tick time
    } /*End Of Loop*/

/*Error Processing*/
err:
    /*Set Termination Request Semaphore To 0*/
    sem_trywait(&terminate_req_sem);
    goto hdmi_end;

hdmi_end:
    /*To terminate the loop in Capture Thread.*/
    hdmi_obj_ready.store(0);
    printf("Display Thread Terminated\n");
    pthread_exit(NULL);
}

/*****************************************
* Function Name : R_Kbhit_Thread
* Description   : Executes the Keyboard hit thread (checks if enter key is hit)
* Arguments     : threadid = thread identification
* Return value  : -
******************************************/
void *R_Kbhit_Thread(void *threadid)
{
    /*Semaphore Variable*/
    int32_t kh_sem_check = 0;
    /*Variable to store the getchar() value*/
    int32_t c = 0;
    /*Variable for checking return value*/
    int8_t ret = 0;

    printf("Key Hit Thread Starting\n");

    printf("************************************************\n");
    printf("* Press ENTER key to quit. *\n");
    printf("************************************************\n");

    /*Set Standard Input to Non Blocking*/
    errno = 0;
    ret = fcntl(0, F_SETFL, O_NONBLOCK);
    if (-1 == ret)
    {
        fprintf(stderr, "[ERROR] Failed to run fctnl(): errno=%d\n", errno);
        goto err;
    }

    while(1)
    {
        /*Gets the Termination request semaphore value. If different then 1 Termination was requested*/
        /*Checks if sem_getvalue is executed wihtout issue*/
        errno = 0;
        ret = sem_getvalue(&terminate_req_sem, &kh_sem_check);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to get Semaphore Value: errno=%d\n", errno);
            goto err;
        }
        /*Checks the semaphore value*/
        if (1 != kh_sem_check)
        {
            goto key_hit_end;
        }

#if END_DET_TYPE 
        // 1. Receive the end command via named pipe /tmp/appdetect from app_pointer_det.
        // 2. Send the end command via named pipe /tmp/gui to app_rzv2h_demo
        int fd;
        char str[BUF_SIZE];
        char str_end[BUF_SIZE] = "end";
        ssize_t size;
        mkfifo("/tmp/appdetect", 0666);
        fd = open("/tmp/appdetect", O_RDWR);
        size = read(fd, str, BUF_SIZE);
        if (size > 0)
        {
            /* When mouse clicked. */
            printf("mouse clicked. : %s\n", str);
            str[size] = '\n';

            if (strcmp(str, str_end) == 0)
            {
                if (system("echo \"end\" > /tmp/gui") == -1)
                {
                    printf("[ERROR] Failed to send command\n");
                }
                goto err;
            }
        }
        close(fd);
#else
        c = getchar();
        if (EOF != c)
        {
            if(c == 'w')
            {
                md.pr.ty = md.pr.ty - 0.5;
                std::cout << "up : " << md.pr.ty << std::endl;
                c = getchar();
                continue;
            } else if (c == 'x') {
                md.pr.ty = md.pr.ty + 0.5;
                std::cout << "down" << md.pr.ty << std::endl;
                c = getchar();
                continue;
            } else if (c == 'a') {
                md.pr.angle = md.pr.angle + 0.02;
                std::cout << "left" << md.pr.angle << std::endl;
                c = getchar();
                continue;
            } else if (c == 's') {
                std::cout << "right" << md.pr.angle << std::endl;
                md.pr.angle = md.pr.angle - 0.02;
                c = getchar();
                continue;
            }

            /* When key is pressed. */
            printf("key Detected.\n");
            goto err;
        }
#endif // END_DET_TYPE

        /* When nothing is detected. */
        usleep(WAIT_TIME);
    }

/*Error Processing*/
err:
    /*Set Termination Request Semaphore to 0*/
    sem_trywait(&terminate_req_sem);
    goto key_hit_end;

key_hit_end:
    printf("Key Hit Thread Terminated\n");
    pthread_exit(NULL);
}

/*****************************************
* Function Name : R_Main_Process
* Description   : Runs the main process loop
* Arguments     : -
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
int8_t R_Main_Process()
{
    /*Main Process Variables*/
    int8_t main_ret = 0;
    /*Semaphore Related*/
    int32_t sem_check = 0;
    /*Variable for checking return value*/
    int8_t ret = 0;

    printf("Main Loop Starts\n");
    while(1)
    {
        /*Gets the Termination request semaphore value. If different then 1 Termination was requested*/
        errno = 0;
        ret = sem_getvalue(&terminate_req_sem, &sem_check);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to get Semaphore Value: errno=%d\n", errno);
            goto err;
        }
        /*Checks the semaphore value*/
        if (1 != sem_check)
        {
            goto main_proc_end;
        }

#if END_DET_TYPE // To launch app_pointer_det.
        if (display_state == 1)
        {
            if (system("./../app_pointer_det & ") == -1)
            {
                printf("Command Error\n");
                goto main_proc_end;
            }
            display_state = 2;
        }
#endif
        /*Wait for 1 TICK.*/
        usleep(WAIT_TIME);
    }

/*Error Processing*/
err:
    sem_trywait(&terminate_req_sem);
    main_ret = 1;
    goto main_proc_end;
/*Main Processing Termination*/
main_proc_end:
    printf("Main Process Terminated\n");
    return main_ret;
}

#if (1) //TVM
/*****************************************
* Function Name : get_drpai_start_addr
* Description   : Function to get the start address of DRPAImem.
* Arguments     : drpai_fd: DRP-AI file descriptor
* Return value  : If non-zero, DRP-AI memory start address.
*                 0 is failure.
******************************************/
#ifdef V2H
uint64_t get_drpai_start_addr(int drpai_fd)
#else
uint32_t get_drpai_start_addr(int drpai_fd)
#endif
{
    int ret = 0;
    drpai_data_t drpai_data;

    errno = 0;

    /* Get DRP-AI Memory Area Address via DRP-AI Driver */
    ret = ioctl(drpai_fd , DRPAI_GET_DRPAI_AREA, &drpai_data);
    if (-1 == ret)
    {
        std::cerr << "[ERROR] Failed to get DRP-AI Memory Area : errno=" << errno << std::endl;
        return 0;
    }

    return drpai_data.address;
}

/*****************************************
* Function Name : set_drp_freq
* Description   : Function to set the DRP frequency.
* Arguments     : drpai_fd: DRP-AI file descriptor
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
int set_drp_freq(int drpai_fd)
{
    int ret = 0;
    uint32_t data;

    errno = 0;
    data = drp_max_freq;
    ret = ioctl(drpai_fd , DRPAI_SET_DRP_MAX_FREQ, &data);
    if (-1 == ret)
    {
        std::cerr << "[ERROR] Failed to set DRP Max Frequency : errno=" << errno << std::endl;
        return -1;
    }

    return 0;
}

/*****************************************
* Function Name : init_drpai
* Description   : Function to initialize DRP-AI.
* Arguments     : drpai_fd: DRP-AI file descriptor
* Return value  : If non-zero, DRP-AI memory start address.
*                 0 is failure.
******************************************/
#ifdef V2H
uint64_t init_drpai(int drpai_fd)
#else
uint32_t init_drpai(int drpai_fd)
#endif
{
    int ret = 0;
#ifdef V2H
    uint64_t drpai_addr = 0;
#else
    uint32_t drpai_addr = 0;
#endif

    /*Get DRP-AI memory start address*/
    drpai_addr = get_drpai_start_addr(drpai_fd);
    if (drpai_addr == 0)
    {
        return 0;
    }

    /*Set DRP-AI frequency*/
    ret = set_drp_freq(drpai_fd);
    if (ret != 0)
    {
        return 0;
    }

    return drpai_addr;
}
#endif  //TVM

int32_t main(int32_t argc, char * argv[])
{
    /* Log File Setting */
    auto now = std::chrono::system_clock::now();
    auto tm_time = spdlog::details::os::localtime(std::chrono::system_clock::to_time_t(now));
    char date_buf[64];
    char time_buf[128];
    memset(time_buf,0,sizeof(time_buf));
    std::strftime(date_buf, sizeof(date_buf), "%Y-%m-%d_%H-%M-%S", &tm_time);
    sprintf(time_buf,"logs/%s_app_yolov8_seg_stereo_cam.log",date_buf);
    auto logger = spdlog::basic_logger_mt("logger", time_buf);
    spdlog::set_default_logger(logger);

    /*Disable OpenCV Accelerator due to the use of multithreading */
#ifdef OPENCVA_ALL_ENABLE
    for (int i = 0; i < OCA_LIST_NUM; i++)
        OCA_list[i] = OPENCVA_FUNC_ENABLE;
#else
    for (int i = 0; i < OCA_LIST_NUM; i++)
        OCA_list[i] = OPENCVA_FUNC_DISABLE;
#endif
    OCA_Activate(&OCA_list[0]);
    
    /* DRP-AI Frequency Setting */
    drp_max_freq = 2;
    drpai_freq = 2;

    /*Setting parameter of application */
    std::cout << "************************************************" << std::endl;
    if (argc == 1)
    {
      std::cout << md.mode << " Default parameter mode" << std::endl;
      std::cout << "Stereo calculation : " << md.mode << std::endl;
    }
    else if (argc == 2)
    {
        if((strcmp(argv[1],"cpu") == 0) || (strcmp(argv[1],"oca") == 0))
        {
             md.mode = argv[1];
        } else {
             std::cout << "!!!Error!!! Check parameter" << std::endl;
        }
        std::cout << "Default parameter mode" << std::endl;
        std::cout << "Stereo calculation : " << md.mode << std::endl;
    }
    else if (argc == 3)
    {
        if((strcmp(argv[1],"cpu") == 0) || (strcmp(argv[1],"oca") == 0))
        {
             md.mode = argv[1];
        } else {
             std::cout << "!!!Error!!! Check parameter" << std::endl;
        }
        md.set_simple(argv);
        std::cout << "Simple parameter mode" << std::endl;
        std::cout << "Stereo calculation : " << md.mode << std::endl;
    }
    else if (argc == 17)
    {
        if((strcmp(argv[1],"cpu") == 0) || (strcmp(argv[1],"oca") == 0))
        {
             md.mode = argv[1];         
        } else {
             std::cout << "!!!Error!!! Check parameter" << std::endl;
        }
        md.set_detail(argv);
        std::cout << "Detail parameter mode" << std::endl;
        std::cout << "Stereo calculation : " << md.mode << std::endl;
    }
    else
    {
        std::cout << "!!!Error!!! Check parameter" << std::endl;
    }
    std::cout << "************************************************" << std::endl;

    md.print_param();
        
    int8_t main_proc = 0;
    int8_t ret = 0;
    int8_t ret_main = 0;
    /*Multithreading Variables*/
    int32_t create_thread_ai = -1;
    int32_t create_thread_stereo = -1;
    int32_t create_thread_key = -1;
    int32_t create_thread_capture = -1;
    int32_t create_thread_img = -1;
    int32_t create_thread_hdmi = -1;
    int32_t sem_create = -1;

#if (1) // TVM
    InOutDataType input_data_type;
    bool runtime_status = false;
#endif  // TVM

    printf("RZ/V2H DRP-AI Sample Application\n");
    printf("Model : Ultralytics Detection YOLOv8_seg_stereo | %s\n", model_dir.c_str());
    printf("Input : %s\n", INPUT_CAM_NAME);
    spdlog::info("************************************************");
    spdlog::info("  RZ/V2H DRP-AI Sample Application");
    spdlog::info("  Model : Ultralytics Detection YOLOv8_seg_stereo | {}", model_dir.c_str());
    spdlog::info("  Input : {}", INPUT_CAM_NAME);
    spdlog::info("************************************************");
    printf("Argument : <DRP0_max_freq_factor> = %d\n", drp_max_freq);
    printf("Argument : <AI-MAC_freq_factor> = %d\n", drpai_freq);

#if (1) // TVM
    uint64_t drpaimem_addr_start = 0;
    
    errno = 0;
    int drpai_fd = open("/dev/drpai0", O_RDWR);
    if (0 > drpai_fd)
    {
        fprintf(stderr, "[ERROR] Failed to open DRP-AI Driver : errno=%d\n", errno);
        goto end_main;
    }
    
    /*Initialize DRP-AI (Get DRP-AI memory address and set DRP-AI frequency)*/
    drpaimem_addr_start = init_drpai(drpai_fd);
    if (drpaimem_addr_start == 0)
    {
        goto end_close_drpai;
    }

    /*Load pre_dir object to DRP-AI */
    ret = preruntime.Load(pre_dir);
    if (0 < ret)
    {
        fprintf(stderr, "[ERROR] Failed to run Pre-processing Runtime Load().\n");
        goto end_close_drpai;
    }

    runtime_status = runtime.LoadModel(model_dir, drpaimem_addr_start);

    if(!runtime_status)
    {
        fprintf(stderr, "[ERROR] Failed to load model.\n");
        goto end_close_drpai;
    }

    /*Get input data */
    input_data_type = runtime.GetInputDataType(0);
    if (InOutDataType::FLOAT32 == input_data_type)
    {
        /*Do nothing*/
    }
    else if (InOutDataType::FLOAT16 == input_data_type)
    {
        fprintf(stderr, "[ERROR] Input data type : FP16.\n");
        /*If your model input data type is FP16, use std::vector<uint16_t> for reading input data. */
        goto end_close_drpai;
    }
    else
    {
        fprintf(stderr, "[ERROR] Input data type : neither FP32 nor FP16.\n");
        goto end_close_drpai;
    }
#endif  // TVM

#ifndef INPUT_IMAGE  // INPUT_CAMERA
    /* Create Camera Instance */
    g_capture  = new Camera2();
    g_capture1 = new Camera2();

    /* Init and Start Camera */
    ret = g_capture->start_s_camera(md.df.left);
    if (0 != ret)
    {
        fprintf(stderr, "[ERROR] Failed to initialize Camera Left.\n");
        delete g_capture;
        ret_main = ret;
        goto end_main;
    }
    ret = g_capture1->start_s_camera(md.df.right);
    if (0 != ret)
    {
        fprintf(stderr, "[ERROR] Failed to initialize Camera Right.\n");
        delete g_capture1;
        ret_main = ret;
        goto end_main;
    }
#else
    /* Create Camera Instance */
    g_capture = new Input_Files();
    if (g_capture->init(md.df.left) != 0)
    {
        printf("[ERROR] Failed to initialize Input_Files\n");
        goto end_main;
    }
    g_capture1 = new Input_Files();
    if (g_capture1->init(md.df.right) != 0)
    {
        printf("[ERROR] Failed to initialize Input_Files\n");
        goto end_main;
    }

    /* Init and Start Camera */
    ret = g_capture->allocate_buf_memory();
    if (0 != ret)
    {
        fprintf(stderr, "[ERROR] Failed to initialize Camera.\n");
        delete g_capture;
        ret_main = ret;
        goto end_main;
    }
    ret = g_capture1->allocate_buf_memory();
    if (0 != ret)
    {
        fprintf(stderr, "[ERROR] Failed to initialize Camera.\n");
        delete g_capture;
        ret_main = ret;
        goto end_main;
    }
    
#endif

    /*Initialize Image object.*/
    ret = img.init(CAM_IMAGE_WIDTH, CAM_IMAGE_HEIGHT, CAM_IMAGE_CHANNEL_YUY2, IMAGE_OUTPUT_WIDTH, IMAGE_OUTPUT_HEIGHT, IMAGE_CHANNEL_BGRA, g_capture->wayland_buf->mem);
    if (0 != ret)
    {
        fprintf(stderr, "[ERROR] Failed to initialize Image object.\n");
        ret_main = ret;
        goto end_close_camera;
    }
    ret = img1.init(CAM_IMAGE_WIDTH, CAM_IMAGE_HEIGHT, CAM_IMAGE_CHANNEL_YUY2, IMAGE_OUTPUT_WIDTH, IMAGE_OUTPUT_HEIGHT, IMAGE_CHANNEL_BGRA, g_capture1->wayland_buf->mem);
    if (0 != ret)
    {
        fprintf(stderr, "[ERROR] Failed to initialize Image object.\n");
        ret_main = ret;
        goto end_close_camera;
    }

    if(md.df.cam_calib_en == 1) {
        img.init_undistort_map("left");
        img1.init_undistort_map("right");
    }
    
    /*Termination Request Semaphore Initialization*/
    /*Initialized value at 1.*/
    sem_create = sem_init(&terminate_req_sem, 0, 1);
    if (0 != sem_create)
    {
        fprintf(stderr, "[ERROR] Failed to Initialize Termination Request Semaphore.\n");
        ret_main = -1;
        goto end_threads;
    }

    /*Create Key Hit Thread*/
    create_thread_key = pthread_create(&kbhit_thread, NULL, R_Kbhit_Thread, NULL);
    if (0 != create_thread_key)
    {
        fprintf(stderr, "[ERROR] Failed to create Key Hit Thread.\n");
        ret_main = -1;
        goto end_threads;
    }

    /*Create Inference Thread*/
    create_thread_ai = pthread_create(&ai_inf_thread, NULL, R_Inf_Thread, NULL);
    if (0 != create_thread_ai)
    {
        sem_trywait(&terminate_req_sem);
        fprintf(stderr, "[ERROR] Failed to create AI Inference Thread.\n");
        ret_main = -1;
        goto end_threads;
    }

    /*Create Stereo Thread*/
     create_thread_stereo = pthread_create(&stereo_thread, NULL, R_Stereo_Thread, NULL);
    if (0 != create_thread_stereo)
    {
        sem_trywait(&terminate_req_sem);
        fprintf(stderr, "[ERROR] Failed to create Stereo Thread.\n");
        ret_main = -1;
        goto end_threads;
    }

    /*Create Capture Thread*/
    cap_arg.capture  = g_capture;
    cap_arg.capture1 = g_capture1;
    create_thread_capture = pthread_create(&capture_thread, NULL, R_Capture_Thread, &cap_arg);
    if (0 != create_thread_capture)
    {
        sem_trywait(&terminate_req_sem);
        fprintf(stderr, "[ERROR] Failed to create Capture Thread.\n");
        ret_main = -1;
        goto end_threads;
    }

    /*Create Image Thread*/
    create_thread_img = pthread_create(&img_thread, NULL, R_Img_Thread, NULL);
    if(0 != create_thread_img)
    {
        sem_trywait(&terminate_req_sem);
        fprintf(stderr, "[ERROR] Failed to create Image Thread.\n");
        ret_main = -1;
        goto end_threads;
    }

    /*Create Display Thread*/
    create_thread_hdmi = pthread_create(&hdmi_thread, NULL, R_Display_Thread, NULL);
    if(0 != create_thread_hdmi)
    {
        sem_trywait(&terminate_req_sem);
        fprintf(stderr, "[ERROR] Failed to create Display Thread.\n");
        ret_main = -1;
        goto end_threads;
    }

    /*Main Processing*/
    main_proc = R_Main_Process();
    if (0 != main_proc)
    {
        fprintf(stderr, "[ERROR] Error during Main Process\n");
        ret_main = -1;
    }

    goto end_threads;

end_threads:
    if(0 == create_thread_hdmi)
    {
        ret = wait_join(&hdmi_thread, DISPLAY_THREAD_TIMEOUT);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to exit Display Thread on time.\n");
            ret_main = -1;
        }
    }
    if(0 == create_thread_img)
    {
        ret = wait_join(&img_thread, DISPLAY_THREAD_TIMEOUT);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to exit Image Thread on time.\n");
            ret_main = -1;
        }
    }
    if (0 == create_thread_capture)
    {
        ret = wait_join(&capture_thread, CAPTURE_TIMEOUT);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to exit Capture Thread on time.\n");
            ret_main = -1;
        }
    }
    if (0 == create_thread_ai)
    {
        ret = wait_join(&ai_inf_thread, AI_THREAD_TIMEOUT);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to exit AI Inference Thread on time.\n");
            ret_main = -1;
        }
    }
    if (0 == create_thread_stereo)
    {
        ret = wait_join(&stereo_thread, STEREO_THREAD_TIMEOUT);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to exit Stereo Thread on time.\n");
            ret_main = -1;
        }
    }
    if (0 == create_thread_key)
    {
        ret = wait_join(&kbhit_thread, KEY_THREAD_TIMEOUT);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to exit Key Hit Thread on time.\n");
            ret_main = -1;
        }
    }

    /*Delete Terminate Request Semaphore.*/
    if (0 == sem_create)
    {
        sem_destroy(&terminate_req_sem);
    }

    /* Exit waylad */
    wayland.exit();
    goto end_close_camera;

end_close_camera:

#ifndef INPUT_IMAGE
    /*Close USB Camera.*/
    ret = g_capture->close_camera();
    if (0 != ret)
    {
        fprintf(stderr, "[ERROR] Failed to close Camera.\n");
        ret_main = -1;
    }
    ret = g_capture1->close_camera();
    if (0 != ret)
    {
        fprintf(stderr, "[ERROR] Failed to close Camera.\n");
        ret_main = -1;
    }
#else
    /*Close buf_memory Camera.*/
    ret = g_capture->close_buf_memory();
    if (0 != ret)
    {
        fprintf(stderr, "[ERROR] Failed to close buf_memory.\n");
        ret_main = -1;
    }
    ret = g_capture1->close_buf_memory();
    if (0 != ret)
    {
        fprintf(stderr, "[ERROR] Failed to close buf_memory.\n");
        ret_main = -1;
    }
#endif
    delete g_capture;
    delete g_capture1;
    goto end_close_drpai;


end_close_drpai:
    /*Close DRP-AI Driver.*/
    if (0 < drpai_fd)
    {
        errno = 0;
        ret = close(drpai_fd);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to close DRP-AI Driver: errno=%d\n", errno);
            ret_main = -1;
        }
    }
    goto end_main;

end_main:
    printf("Application End\n");
    return ret_main;
}
