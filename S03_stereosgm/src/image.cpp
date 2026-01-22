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
 * File Name    : image.cpp
 * Version      : v3.20
 * Description  : RZ/V2H AI SDK Sample Application for StereoSGM
 ***********************************************************************************************************************/

/*****************************************
 * Includes
 ******************************************/
#include "image.h"
Image::Image()
{
    cam_gamma_path = STEREO_GAMMA_PATH;

    cameraMatrix = cv::Mat(3, 3, CV_64FC1, cm);
    distCoeffs = cv::Mat(5, 1, CV_64FC1, dc);
    filter = cv::Mat(3, 3, CV_16SC1, fl_sel);

    img_gamma = cv::Mat(256, 256, CV_8UC1, cv::Scalar{0});
    lut_mat = cv::Mat(1, 256, CV_8UC1, cv::Scalar{0});
}

Image::~Image()
{
}

/*****************************************
 * Function Name : get_calc
 ******************************************/
cv::Mat Image::get_calc()
{
    return img_calc;
}
/*****************************************
 * Function Name : get_img_gamma
 ******************************************/
cv::Mat Image::get_img_gamma()
{
    return img_gamma;
}
/*****************************************
 * Function Name : set_mat
 ******************************************/
void Image::set_mat(const cv::Mat &input_mat)
{
    img_mat = input_mat.clone();
}

/*****************************************
 * Function Name : init
 ******************************************/
uint8_t Image::init(std::string img, std::string fmt,
                    uint w, uint h, uint c,
                    uint ow, uint oh, uint oc,
                    uint trim_x, uint trim_y, uint remap, uint gamma, uint sharp)
{
    image = img;
    format = fmt;
    /*Initialize input image information */
    img_w = w;
    img_h = h;
    img_c = c;
    /*Initialize output image information*/
    img_calc_w = ow;
    img_calc_h = oh;
    img_calc_c = oc;

    rect_x = trim_x;
    rect_y = trim_y;
    rect_w = w - trim_x * 2;
    rect_h = h - trim_y * 2;

    i_remap = remap;
    i_gamma = gamma;
    i_sharp = sharp;

    for (int ii = 0; ii < 9; ii++)
        fl_sel[ii] = fl[i_sharp][ii];

    /* warpAffine */
    aff_ctr.x = img_w / 2.0;
    aff_ctr.y = img_h / 2.0;
    angle = 0.0;
    tx = 0.0;
    ty = 0.0;

    return 0;
}

/*****************************************
 * Function Name : init_undistort_map
 ******************************************/
void Image::init_undistort_map(std::string side)
{
    cv::Mat mapX;
    cv::Mat mapY;

    std::string str_buf;
    std::string str_conma_buf;

    std::string img_up = image;
    std::transform(image.begin(), image.end(), img_up.begin(), ::toupper);

    cam_mat_path = "./calib_data/camera_" + img_up + "_" + side + "_" + format + "_"
                   + std::to_string(img_w) + "_" + std::to_string(img_h) + ".csv";
    cam_dis_path = "./calib_data/dist_" + img_up + "_" + side + "_" + format + "_"
                   + std::to_string(img_w) + "_" + std::to_string(img_h) + ".csv";

    std::cout << cam_mat_path.c_str() << std::endl;
    std::cout << cam_dis_path.c_str() << std::endl;

    std::ifstream ifs_mat_file(cam_mat_path);
    std::ifstream ifs_dis_file(cam_dis_path);

    for (int ii = 0; ii < 3; ii++)
    {
        std::getline(ifs_mat_file, str_buf);
        std::istringstream i_stream(str_buf);
        for (int jj = 0; jj < 3; jj++)
        {
            std::getline(i_stream, str_conma_buf, ',');
            cm[ii * 3 + jj] = std::stod(str_conma_buf);
        }
    }

    for (int ii = 0; ii < 1; ii++)
    {
        std::getline(ifs_dis_file, str_buf);
        std::istringstream i_stream(str_buf);
        for (int jj = 0; jj < 5; jj++)
        {
            std::getline(i_stream, str_conma_buf, ',');
            dc[ii * 3 + jj] = std::stod(str_conma_buf);
        }
    }

    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cv::Mat(), cv::Size(img_w, img_h), CV_32FC1, mapX, mapY);
    cv::convertMaps(mapX, mapY, calibration_maps, cv::noArray(), CV_32FC2);
}
/*****************************************
 * Function Name : init_gamma
 ******************************************/
void Image::init_gamma()
{

    std::ifstream ifs(cam_gamma_path);
    std::vector<std::vector<int>> gamma_data;

    std::cout << "Read Gamma data file: " << cam_gamma_path << std::endl;

    if (ifs)
    {
        std::string line;

        while (getline(ifs, line))
        {
            std::vector<int> datvec;

            std::istringstream stream(line);
            std::string field;
            std::vector<std::string> result;
            while (getline(stream, field, ','))
                result.push_back(field);

            for (auto &&s : result)
                datvec.push_back(std::stoi(s));
            gamma_data.push_back(datvec);
        }

        for (size_t j = 0; j < gamma_data[i_gamma].size(); j++)
        {
            lut_mat.data[j] = gamma_data[i_gamma][j];
        }

        /* for Debug */
        int tmp;
        uint8_t ulut[256];
        for (size_t j = 0; j < gamma_data[i_gamma].size(); j++)
        {
            ulut[j] = gamma_data[i_gamma][j];
            img_gamma.at<uchar>(255 - ulut[j], j) = 255;
            tmp = (255 - ulut[j] + 1 > 255) ? 255 : 255 - ulut[j] + 1;
            img_gamma.at<uchar>(tmp, j) = 255;
            tmp = (255 - ulut[j] - 1 < 0) ? 0 : 255 - ulut[j] - 1;
            img_gamma.at<uchar>(tmp, j) = 255;
        }
    }
}
/*****************************************
 * Function Name : pre_process
 ******************************************/
void Image::pre_process()
{
    cv::Mat img_rem;
    cv::Mat img_aff;
    cv::Mat img_tmp;
    cv::Mat img_res;
    cv::Mat img_gray;

    if (i_remap != 0)
    {
        cv::remap(img_mat, img_rem, calibration_maps, cv::Mat(), cv::INTER_LINEAR);
    }
    else
    {
        img_rem = img_mat;
    }
    
    aff_mat = cv::getRotationMatrix2D(aff_ctr, angle, 1.0);
	aff_mat.at<double>(0,2) = tx;
	aff_mat.at<double>(1,2) = ty;
    cv::warpAffine(img_rem, img_aff, aff_mat, img_rem.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
 
    img_tmp = cv::Mat(img_aff, cv::Rect(rect_x, rect_y, rect_w, rect_h));
    cv::resize(img_tmp, img_res, cv::Size(img_calc_w, img_calc_h));
    cv::cvtColor(img_res, img_gray, cv::COLOR_BGR2GRAY);
    if (i_sharp != 0)
        cv::filter2D(img_gray, img_gray, -1, filter);
    if (i_gamma != 0)
        cv::LUT(img_gray, lut_mat, img_gray);

    img_calc = img_gray.clone();
}
