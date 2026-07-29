/***********************************************************************************************************************
* Copyright (C) 2023-2026 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name    : image_yolov8_seg_stereo.h
* Version      : R2026-07
* Description  : RZ/V2H DRP-AI Sample Application for Ultralytics Detection YOLOv8 with MIPI/USB Camera
***********************************************************************************************************************/

#ifndef IMAGE_SEG_STEREO_H
#define IMAGE_SEG_STEREO_H

#include "define.h"
#include "ascii.h"
#include "mode_seg_stereo.h"

class Image
{
    public:
        Image();
        ~Image();

        /* warpAffine */  
        cv::Point2d aff_ctr;
        cv::Mat aff_mat;

        double ty_min;
        double val_min;
        double angle_min;
  
        int focus_en;

        uint8_t* img_buffer[WL_BUF_NUM];
        uint8_t* overlay_buffer[WL_BUF_NUM];
        uint8_t get_buf_id();
        //void write_string_rgb(const std::string& str, uint32_t align_type, uint32_t x, uint32_t y, float size, uint32_t color);
        void write_string_rgb(std::string str, uint32_t align_type, uint32_t x, uint32_t y, float size, uint32_t color);
        void write_string_rgb_boundingbox(std::string str, uint32_t align_type,  uint32_t x_min, uint32_t y_min, uint32_t x_max, uint32_t y_max,float scale, uint32_t color);

        uint32_t get_H();
        uint32_t get_W();
        uint32_t get_C();
        uint8_t* get_img(uint8_t id);
        uint8_t* get_overlay_img(uint8_t id);
        uint8_t at(int32_t a);
        void set(int32_t a, uint8_t val);

        uint8_t init(uint32_t w, uint32_t h, uint32_t c, uint32_t ow, uint32_t oh, uint32_t oc, void *mem);
        uint8_t init(uint32_t w, uint32_t h, uint32_t c, uint32_t ow, uint32_t oh, uint32_t oc);
        //void draw_rect(int32_t x, int32_t y, int32_t w, int32_t h, const std::string& str,uint32_t color);
        void draw_rect(int32_t x, int32_t y, int32_t w, int32_t h, const char* str,uint32_t color);
        void reset_overlay_img();
        void convert_format();
        void convert_size(int in_w, int resize_w, int in_h, int resize_h, bool is_padding);
        void camera_to_image(const uint8_t* buffer, int32_t size);

        void init_undistort_map(std::string side);
        void camera_to_s_image(const uint8_t* buffer, int32_t size);
        void convert_format_s_image(int rem_en, int aff_en);
        uint8_t* get_s_image();
        void set_stereo_out(const cv::Mat &input_mat);
        void draw_stereo_out();
        void adjust_focus();

        cv::Mat get_stereo_out();
        cv::Mat get_disparity_u8_cam();
        cv::Mat get_disparity_s16_cam();

        double cm[9];
        double dc[5];

        cv::Mat cameraMatrix;
        cv::Mat distCoeffs;
        cv::Mat calibration_maps;
  
    private:
        uint8_t buf_id = 0;

        /* Input Image (YUYV from V4L2) Information */
        uint32_t img_h;
        uint32_t img_w;
        uint32_t img_c;
        /* Output Image (BGRA for Wayland) Information */
        uint32_t out_h;
        uint32_t out_w;
        uint32_t out_c;

        uint32_t front_color        = BLACK_DATA;
        uint32_t back_color         = WHITE_DATA;
        uint8_t font_w              = FONTDATA_WIDTH;
        uint8_t font_h              = FONTDATA_HEIGHT;
        void draw_point_yuyv(int32_t x, int32_t y, uint32_t color);
        void draw_line(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint32_t color);
        void write_char(char code,  uint32_t x,  uint32_t y, uint32_t color, uint32_t backcolor);
        void write_string(const char * pcode, uint32_t x, uint32_t y, uint32_t color, uint32_t backcolor);
        uint8_t Clip(int value);

        uint8_t img_s_yuyv_buf[CAM_IMAGE_HEIGHT * CAM_IMAGE_WIDTH * CAM_IMAGE_CHANNEL_YUY2];
        uint8_t img_s_gray_buf[CAM_IMAGE_HEIGHT * CAM_IMAGE_WIDTH * 1];  // max size  STEREO_IN < CAM_IMAGE

        cv::Mat img_s_yuyv;
        cv::Mat img_s_gray;
        cv::Mat disparity_h;
        cv::Mat disparity_u8_cam;
        cv::Mat disparity_s16_cam;
        cv::Mat stereo_out_s16;
};

#endif
