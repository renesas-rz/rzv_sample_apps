/***********************************************************************************************************************
 * Copyright 2026 Renesas Electronics Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ***********************************************************************************************************************/
/***********************************************************************************************************************
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
 * other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
 * applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
 * EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
 * SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
 * this software. By using this software, you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Copyright (C) 2024-2026 Renesas Electronics Corporation. All rights reserved.
 ***********************************************************************************************************************/
/***********************************************************************************************************************
 * File Name    : capture.h
 * Version      : v3.10
 * Description  : RZ/V2H AI SDK Sample Application for StereoSGMn
 ***********************************************************************************************************************/

#ifndef CAPTURE_H
#define CAPTURE_H

#include "define.h"

class Capture
{
public:
    Capture();
    ~Capture();

    cv::Mat img_mat;

    uint32_t get_H();
    uint32_t get_W();
    uint32_t get_C();
    std::string get_gstreamer();
    std::string get_media_ctl(int index);

    // uint8_t init(uint32_t w, uint32_t h, uint32_t c, uint32_t ow, uint32_t oh, uint32_t oc);
    void init(std::string side, std::string image, std::string drv, std::string form, uint fps, uint w, uint h, uint c);

    void convert_size(int in_w, int resize_w, bool is_padding);
    void set_mat(const cv::Mat &input_mat);
    cv::Mat get_mat();

private:
    /* Input Image (BGR from camera) Information */
    uint32_t img_h;
    uint32_t img_w;
    uint32_t img_c;

    std::string cam_side;
    std::string cam_type;
    std::string cam_image;
    std::string cam_drv;
    std::string cam_form;
    uint32_t cam_fps;
    std::string cam_jpeg;
    ;

    std::stringstream ss;
    std::string v4l2src_str;

    std::string media_ctl[6];
};

#endif
