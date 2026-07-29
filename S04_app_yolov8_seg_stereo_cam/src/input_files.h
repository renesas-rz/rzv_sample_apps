/***********************************************************************************************************************
* Copyright (C) 2023-2026 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name    : input_files.h
* Version      : R2026-06
* Description  : RZ/V2H DRP-AI Sample Application for Ultralytics Detection YOLOv8-seg with MIPI/USB Camera
***********************************************************************************************************************/

#ifndef INPUT_FILES_H
#define INPUT_FILES_H

#include <linux/videodev2.h>
#include "define.h"
/*DMA control*/
#include "dmabuf.h"
/* This block of code is only accessible from C code. */
#ifdef __cplusplus
extern "C" {
#endif
#include "mmngr_user_public.h"
#include "mmngr_buf_user_public.h"
#ifdef __cplusplus
}
#endif

class Input_Files
{
    public:
        Input_Files();
        ~Input_Files();

        dma_buffer *wayland_buf;
        dma_buffer *overlay_buf;
        dma_buffer *drpai_buf;

        int8_t allocate_buf_memory();
        int8_t close_buf_memory();
        int8_t init(const std::string& dir_path);
        int8_t read_file(int idx_in, cv::Mat& bgr);

        uint8_t * get_img();
        int32_t get_size();
        int32_t get_w();
        void set_w(int32_t w);
        int32_t get_h();
        void set_h(int32_t h);
        int32_t get_c();
        void set_c(int32_t c);
        size_t get_files_size();

    private:
        std::string device;
        int32_t camera_width;   // input image width
        int32_t camera_height;  // input image height
        int32_t camera_color;   // input image color
        int m_fd;
        uint8_t *buffer[CAP_BUF_NUM];

        std::string images_dir;
        std::vector<std::filesystem::path> png_files;

        #define WAYLANDBUF      (IMAGE_OUTPUT_WIDTH * IMAGE_OUTPUT_HEIGHT * IMAGE_CHANNEL_BGRA * WL_BUF_NUM)
        #define CAPTUREBUF      (CAM_IMAGE_WIDTH * CAM_IMAGE_HEIGHT * CAM_IMAGE_CHANNEL_YUY2)
#if (1) == DRPAI_INPUT_PADDING /* Only use for yolox */
        #define DRPAIBUF        (CAM_IMAGE_WIDTH * CAM_IMAGE_WIDTH * CAM_IMAGE_CHANNEL_YUY2)
#else   /* (1) == DRPAI_INPUT_PADDING */
        #define DRPAIBUF        (CAM_IMAGE_WIDTH * CAM_IMAGE_HEIGHT * CAM_IMAGE_CHANNEL_YUY2)
#endif   /* (1) == DRPAI_INPUT_PADDING */


        struct v4l2_buffer buf_capture;
        dma_buffer *dma_buf;

};

#endif
