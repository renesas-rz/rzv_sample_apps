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
 * File Name    : display.h
 * Version      : v3.20
 * Description  : RZ/V2H AI SDK Sample Application for StereoSGM
 ***********************************************************************************************************************/

#ifndef DISPLAY_H
#define DISPLAY_H

#include "define.h"

class Display
{
public:
    Display();
    ~Display();

    cv::Mat src1;
    cv::Mat src2;
    cv::Mat src1_gray;
    cv::Mat src2_gray;
    cv::Mat stereo_out;
    cv::Mat disparity;
    cv::Mat gamma;
    cv::Mat disparity_h;
    cv::Mat time_mat;

    void set_stereo_out(const cv::Mat &input_mat);
    void set_src1(const cv::Mat &input_mat);
    void set_src2(const cv::Mat &input_mat);
    void set_src1_gray(const cv::Mat &input_mat);
    void set_src2_gray(const cv::Mat &input_mat);
    void set_gamma(const cv::Mat &input_mat);

    void set_capture_time(double proc_time);
    void set_imageL_time(double proc_time);
    void set_imageR_time(double proc_time);
    void set_stereo_time(double proc_time);
    void set_display_time(double proc_time);
    void set_frame_time(double proc_time);

    void post_process();
    void init(uint w, uint h, uint c, uint ow, uint oh, uint oc, uint disp, uint heat_map);
    void print_time();
    void plot_time();
    void plot_time_mat();
    void show_image(std::string s_title, std::string s_graph);

private:
    /* Input Image (BGR from camera) Information */
    uint32_t img_h;
    uint32_t img_w;
    uint32_t img_c;
    /* Calculate Image (GrayScale for Opencv) Information */
    uint32_t img_calc_h;
    uint32_t img_calc_w;
    uint32_t img_calc_c;

    uint32_t img_disp;

    uint32_t heat_no;
    /* cv::COLORMAP_AUTUMN = 0,
       cv::COLORMAP_BONE = 1,
       cv::COLORMAP_JET = 2,
       cv::COLORMAP_WINTER = 3,
       cv::COLORMAP_RAINBOW = 4,
       cv::COLORMAP_OCEAN = 5,
       cv::COLORMAP_SUMMER = 6,
       cv::COLORMAP_SPRING = 7,
       cv::COLORMAP_COOL = 8,
       cv::COLORMAP_HSV = 9,
       cv::COLORMAP_PINK = 10,
       cv::COLORMAP_HOT = 11,
       cv::COLORMAP_PARULA = 12,
       cv::COLORMAP_MAGMA = 13,
       cv::COLORMAP_INFERNO = 14,
       cv::COLORMAP_PLASMA = 15,
       cv::COLORMAP_VIRIDIS = 16,
       cv::COLORMAP_CIVIDIS = 17,
       cv::COLORMAP_TWILIGHT = 18,
       cv::COLORMAP_TWILIGHT_SHIFTED = 19,
       cv::COLORMAP_TURBO = 20,
       cv::COLORMAP_DEEPGREEN = 21 */

    uint32_t frame_cnt;

    /*Processing Time*/
    std::vector<double> capture_time;
    std::vector<double> imageL_time;
    std::vector<double> imageR_time;
    std::vector<double> stereo_time;
    std::vector<double> display_time;
    std::vector<double> frame_time;

    double capture_time_avg;
    double imageL_time_avg;
    double imageR_time_avg;
    double stereo_time_avg;
    double display_time_avg;
    double frame_time_avg;
};
#endif
