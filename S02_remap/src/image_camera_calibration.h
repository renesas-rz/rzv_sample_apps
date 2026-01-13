/***********************************************************************************************************************
* Copyright (C) 2023-2025 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name    : image_camera_calibration.h
* Version      : 1.00
* Description  : RZ/V2N DRP-AI Sample Application for Remap with MIPI Camera
***********************************************************************************************************************/

#ifndef IMAGE_H
#define IMAGE_H

#include "define.h"
#include "camera.h"
#include <opencv2/opencv.hpp>


class Image
{
    public:
        Image();
        ~Image();

        uint8_t* img_buffer[WL_BUF_NUM];
        uint8_t* overlay_buffer[WL_BUF_NUM];
        uint8_t get_buf_id();
        void write_string_rgb(std::string str, uint32_t align_type, uint32_t x, uint32_t y, float scale, uint32_t color);
        int8_t draw_depth_map(uint8_t* buffer, Camera *capture, size_t depth_w, size_t depth_h, size_t resize_w, size_t resize_h);
        void resizeWithAspectRatio(int targetWidth, int targetHeight);
        void padding_img(int resized_w, int resized_h, cv::Mat image);
        int8_t create_side_by_side(int resized_w, int resized_h, bool is_padding, int flip);
        int8_t create_only_img(int resized_w, int resized_h, bool is_padding, int flip);
        cv::Mat convert_to_grayscale(int img_w, int img_h);
        cv::Mat calibrate_camera_img(cv::Mat marge_maps, int img_w, int img_h);
        void convert_calib_img_size(cv::Mat calib_ret_img, int resize_w, int resize_h);

        uint32_t get_H();
        uint32_t get_W();
        uint32_t get_C();
        uint8_t* get_img(uint8_t id);
        uint8_t* get_overlay_img(uint8_t id);
        uint8_t at(int32_t a);
        void set(int32_t a, uint8_t val);

        uint8_t init(uint32_t w, uint32_t h, uint32_t c, uint32_t ow, uint32_t oh, uint32_t oc, void *mem_wayland, void *mem_overlay);
        
        void reset_overlay_img(uint8_t id);
        void convert_format();
        void convert_size(int in_w, int resize_w, int in_h, int resize_h);
        void camera_to_image(const uint8_t* buffer, int32_t size);

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

        uint8_t Clip(int value);

};

#endif
