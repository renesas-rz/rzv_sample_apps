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
 * File Name    : display.cpp
 * Version      : v3.20
 * Description  : RZ/V2H AI SDK Sample Application for StereoSGM
 ***********************************************************************************************************************/

/*****************************************
 * Includes
 ******************************************/
#include "display.h"

Display::Display()
{
    capture_time.resize(AVG_TIME_CNT);
    imageL_time.resize(AVG_TIME_CNT);
    imageR_time.resize(AVG_TIME_CNT);
    stereo_time.resize(AVG_TIME_CNT);
    display_time.resize(AVG_TIME_CNT);
    frame_time.resize(AVG_TIME_CNT);

    std::fill(capture_time.begin(), capture_time.end(), 0.0);
    std::fill(imageL_time.begin(), imageL_time.end(), 0.0);
    std::fill(imageR_time.begin(), imageR_time.end(), 0.0);
    std::fill(stereo_time.begin(), stereo_time.end(), 0.0);
    std::fill(display_time.begin(), display_time.end(), 0.0);
    std::fill(frame_time.begin(), frame_time.end(), 0.0);

    frame_cnt = AVG_TIME_CNT;
}

Display::~Display()
{
}

/*****************************************
 * Function Name : set_stereo_out
 ******************************************/
void Display::set_stereo_out(const cv::Mat &input_mat)
{
    stereo_out = input_mat.clone();
}
/*****************************************
 * Function Name : set_src1
 ******************************************/
void Display::set_src1(const cv::Mat &input_mat)
{
    src1 = input_mat.clone();
}
/*****************************************
 * Function Name : set_src2
 ******************************************/
void Display::set_src2(const cv::Mat &input_mat)
{
    src2 = input_mat.clone();
}
/*****************************************
 * Function Name : set_src1_gray
 ******************************************/
void Display::set_src1_gray(const cv::Mat &input_mat)
{
    src1_gray = input_mat.clone();
}
/*****************************************
 * Function Name : set_src2_gray
 ******************************************/
void Display::set_src2_gray(const cv::Mat &input_mat)
{
    src2_gray = input_mat.clone();
}
/*****************************************
 * Function Name : set_gamma
 ******************************************/
void Display::set_gamma(const cv::Mat &input_mat)
{
    gamma = input_mat.clone();
}
/*****************************************
 * Function Name : set_capture_time
 ******************************************/
void Display::set_capture_time(double proc_time)
{
    capture_time.erase(capture_time.begin());
    capture_time.push_back(proc_time);
    capture_time_avg = std::accumulate(capture_time.begin(), capture_time.end(), 0.0) / capture_time.size();
    capture_time_avg = (frame_cnt == 0) ? capture_time_avg : 0;
}
/*****************************************
 * Function Name : set_imageL_time
 ******************************************/
void Display::set_imageL_time(double proc_time)
{
    imageL_time.erase(imageL_time.begin());
    imageL_time.push_back(proc_time);
    imageL_time_avg = std::accumulate(imageL_time.begin(), imageL_time.end(), 0.0) / imageL_time.size();
    imageL_time_avg = (frame_cnt == 0) ? imageL_time_avg : 0;
}
/*****************************************
 * Function Name : set_imageR_time
 ******************************************/
void Display::set_imageR_time(double proc_time)
{
    imageR_time.erase(imageR_time.begin());
    imageR_time.push_back(proc_time);
    imageR_time_avg = std::accumulate(imageR_time.begin(), imageR_time.end(), 0.0) / imageR_time.size();
    imageR_time_avg = (frame_cnt == 0) ? imageR_time_avg : 0;
}

/*****************************************
 * Function Name : set_stereo_time
 ******************************************/
void Display::set_stereo_time(double proc_time)
{
    stereo_time.erase(stereo_time.begin());
    stereo_time.push_back(proc_time);
    stereo_time_avg = std::accumulate(stereo_time.begin(), stereo_time.end(), 0.0) / stereo_time.size();
    stereo_time_avg = (frame_cnt == 0) ? stereo_time_avg : 0;
}
/*****************************************
 * Function Name : set_display_time
 ******************************************/
void Display::set_display_time(double proc_time)
{
    display_time.erase(display_time.begin());
    display_time.push_back(proc_time);
    display_time_avg = std::accumulate(display_time.begin(), display_time.end(), 0.0) / display_time.size();
    display_time_avg = (frame_cnt == 0) ? display_time_avg : 0;
}
/*****************************************
 * Function Name : set_frame_time
 ******************************************/
void Display::set_frame_time(double proc_time)
{
    frame_time.erase(frame_time.begin());
    frame_time.push_back(proc_time);
    frame_time_avg = std::accumulate(frame_time.begin(), frame_time.end(), 0.0) / frame_time.size();
    frame_time_avg = (frame_cnt == 0) ? frame_time_avg : 0;

    frame_cnt = (frame_cnt > 0) ? frame_cnt - 1 : 0;
}
/*****************************************
 * Function Name : init
 ******************************************/
void Display::init(uint w, uint h, uint c,
                   uint ow, uint oh, uint oc, uint disp, uint heat_map)
{
    /*Initialize input image information */
    img_w = w;
    img_h = h;
    img_c = c;
    /*Initialize output image information*/
    img_calc_w = ow;
    img_calc_h = oh;
    img_calc_c = oc;

    img_disp = disp;

    heat_no = heat_map;

    src1 = cv::Mat::zeros(img_h, img_w, CV_8UC3);
    src2 = cv::Mat::zeros(img_h, img_w, CV_8UC3);
    src1_gray = cv::Mat::zeros(img_calc_h, img_calc_w, CV_8UC1);
    src2_gray = cv::Mat::zeros(img_calc_h, img_calc_w, CV_8UC1);
    disparity = cv::Mat::zeros(img_calc_h, img_calc_w, CV_8UC1);
    disparity_h = cv::Mat::zeros(img_calc_h, img_calc_w, CV_8UC3);    
}
/*****************************************
 * Function Name : post_process
 ******************************************/
void Display::post_process()
{
    double min;
    double max;
    cv::Mat stereo_out_tmp;

    cv::normalize(stereo_out, stereo_out_tmp, 0, 255, cv::NORM_MINMAX);
    stereo_out_tmp.convertTo(disparity, CV_8UC1);
    cv::applyColorMap(disparity, disparity_h, heat_no);
}
/*****************************************
 * Function Name : print_time
 ******************************************/
void Display::print_time()
{
    std::cout << "capture_time : " << capture_time_avg << std::endl;
    std::cout << "imageL_time  : " << imageL_time_avg << std::endl;
    std::cout << "imageR_time  : " << imageR_time_avg << std::endl;
    std::cout << "stereo_time  : " << stereo_time_avg << std::endl;
    std::cout << "display_time : " << display_time_avg << std::endl;
    std::cout << "frame_time   : " << frame_time_avg << std::endl;
}
/*****************************************
 * Function Name : plot_time
 ******************************************/
void Display::plot_time()
{
    std::ostringstream oss;
    std::string s_capture_time;
    std::string s_image_l_time;
    std::string s_image_r_time;
    std::string s_stereo_time;
    std::string s_display_time;
    std::string s_frame_time;

    oss.str("");
    oss << std::fixed << std::setprecision(1) << capture_time_avg;
    s_capture_time = "CAPTURE:" + oss.str() + "[ms]";

    oss.str("");
    oss << std::fixed << std::setprecision(1) << imageL_time_avg;
    s_image_l_time = "IMAGE_L:" + oss.str() + "[ms]";

    oss.str("");
    oss << std::fixed << std::setprecision(1) << imageR_time_avg;
    s_image_r_time = "IMAGE_R:" + oss.str() + "[ms]";

    oss.str("");
    oss << std::fixed << std::setprecision(1) << stereo_time_avg;
    s_stereo_time = "STEREO :" + oss.str() + "[ms]";

    oss.str("");
    oss << std::fixed << std::setprecision(1) << display_time_avg;
    s_display_time = "DISPLAY:" + oss.str() + "[ms]";

    oss.str("");
    oss << std::fixed << std::setprecision(1) << frame_time_avg;
    s_frame_time = "FRAME  :" + oss.str() + "[ms]";

    cv::Point p1(0, 0);
    cv::Point p2(220, 160);
    cv::rectangle(disparity_h, p1, p2, cv::Scalar(128, 128, 128), -1, cv::LINE_4);
    cv::putText(disparity_h, s_capture_time, cv::Point(0, 20),  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    cv::putText(disparity_h, s_image_l_time, cv::Point(0, 45),  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    cv::putText(disparity_h, s_image_r_time, cv::Point(0, 70),  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    cv::putText(disparity_h, s_stereo_time,  cv::Point(0, 95),  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    cv::putText(disparity_h, s_display_time, cv::Point(0, 115), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    cv::putText(disparity_h, s_frame_time,   cv::Point(0, 140), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
}
/*****************************************
 * Function Name : plot_time_mat
 ******************************************/
void Display::plot_time_mat()
{
    std::ostringstream oss;
    std::string s_capture_time;
    std::string s_image_l_time;
    std::string s_image_r_time;
    std::string s_stereo_time;
    std::string s_display_time;
    std::string s_frame_time;
    std::string s_tsu_value;

    oss.str("");
    oss << std::fixed << std::setprecision(1) << capture_time_avg;
    s_capture_time = "CAPTURE:" + oss.str() + "[ms]";

    oss.str("");
    oss << std::fixed << std::setprecision(1) << imageL_time_avg;
    s_image_l_time = "IMAGE_L:" + oss.str() + "[ms]";

    oss.str("");
    oss << std::fixed << std::setprecision(1) << imageR_time_avg;
    s_image_r_time = "IMAGE_R:" + oss.str() + "[ms]";

    oss.str("");
    oss << std::fixed << std::setprecision(1) << stereo_time_avg;
    s_stereo_time = "STEREO :" + oss.str() + "[ms]";

    oss.str("");
    oss << std::fixed << std::setprecision(1) << display_time_avg;
    s_display_time = "DISPLAY:" + oss.str() + "[ms]";

    oss.str("");
    oss << std::fixed << std::setprecision(1) << frame_time_avg;
    s_frame_time = "FRAME  :" + oss.str() + "[ms]";


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

    oss.str("");
    oss << std::fixed << std::setprecision(1) << tsu_value;
    s_tsu_value = "TEMP : " + oss.str() + "[C]";

    
    time_mat = cv::Mat(200, 240, CV_8UC3, cv::Scalar(128,128,128));
   
    cv::putText(time_mat, s_capture_time, cv::Point(10, 25),  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    cv::putText(time_mat, s_image_l_time, cv::Point(10, 50),  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    cv::putText(time_mat, s_image_r_time, cv::Point(10, 75),  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    cv::putText(time_mat, s_stereo_time,  cv::Point(10, 100), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    cv::putText(time_mat, s_display_time, cv::Point(10, 125), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    cv::putText(time_mat, s_frame_time,   cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    cv::putText(time_mat, s_tsu_value,    cv::Point(10, 175), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
}
/*****************************************
 * Function Name : show_image
 ******************************************/
void Display::show_image(std::string s_title, std::string s_graph)
{
    if (s_graph == "disparity_h")
    {
        cv::imshow(s_title.c_str(), disparity_h);
    }
    else if (s_graph == "disparity")
    {
        cv::imshow(s_title.c_str(), disparity);
    }
    else if (s_graph == "stereo_out")
    {
        cv::imshow(s_title.c_str(), stereo_out);
    }
    else if (s_graph == "gamma")
    {
        cv::imshow(s_title.c_str(), gamma);
    }
    else if (s_graph == "src1")
    {
        cv::imshow(s_title.c_str(), src1);
    }
    else if (s_graph == "src2")
    {
        cv::imshow(s_title.c_str(), src2);
    }
    else if (s_graph == "src1_gray")
    {
        cv::imshow(s_title.c_str(), src1_gray);
    }
    else if (s_graph == "src2_gray")
    {
        cv::imshow(s_title.c_str(), src2_gray);
    }
    else if (s_graph == "time_mat")
    {
        cv::imshow(s_title.c_str(), time_mat);
    }
    else
    {
        printf("ERROR imshow \n");
    }
}
