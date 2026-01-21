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
 * File Name    : image.h
 * Version      : v3.10
 * Description  : RZ/V2H AI SDK Sample Application for StereoSGM
 ***********************************************************************************************************************/

#ifndef IMAGE_H
#define IMAGE_H

#include "define.h"

class Image
{
public:
    Image();
    ~Image();

    std::string cam_mat_path;
    std::string cam_dis_path;
    std::string cam_gamma_path;

    cv::Mat img_mat;
    cv::Mat img_calc;

    cv::Point2d aff_ctr;
    cv::Mat aff_mat;
    double tx;
    double ty;
    double angle;


    uint8_t init(std::string img, std::string fmt,
                 uint w, uint h, uint c, uint ow, uint oh, uint oc, uint trim_x, uint trim_y,
                 uint remap, uint gamma, uint sharp);

    void set_mat(const cv::Mat &input_mat);
    cv::Mat get_calc();
    cv::Mat get_img_gamma();

    void init_undistort_map(std::string side);
    void init_gamma();
    void pre_process();

private:
    std::string image;  // file, usb, ec22, ec25, stereo
    std::string format; // data, yuyv, mjpg
    /* Input Image (BGR from camera) Information */
    uint32_t img_h;
    uint32_t img_w;
    uint32_t img_c;
    /* Calculate Image (GrayScale for Opencv) Information */
    uint32_t img_calc_h;
    uint32_t img_calc_w;
    uint32_t img_calc_c;

    uint32_t rect_x;
    uint32_t rect_y;
    uint32_t rect_w;
    uint32_t rect_h;

    uint32_t i_remap;
    uint32_t i_gamma;
    uint32_t i_sharp;

    double cm[9];
    double dc[5];

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Mat calibration_maps;

    cv::Mat img_gamma;
    cv::Mat lut_mat;

    cv::Mat filter;

    int16_t fl[5][9] = {
        {0, 0, 0, 0, 1, 0, 0, 0, 0},         // dummy
        {-1, -1, -1, -1, 9, -1, -1, -1, -1}, // 8 derecton Sharpening Filter
        {0, -1, 0, -1, 5, -1, 0, -1, 0},     // 4 derecton Sharpening Filter
        {1, 1, 1, 1, -8, 1, 1, 1, 1},        // 8 derecton Laplacian filter
        {0, 1, 0, 1, -4, 1, 0, 1, 0}         // 4 derecton Laplacian filter
    };

    int16_t fl_sel[9];
};

#endif
