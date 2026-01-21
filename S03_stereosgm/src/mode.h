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
 * File Name    : mode.h
 * Version      : v3.10
 * Description  : RZ/V2H AI SDK Sample Application for StereoSGM
 ***********************************************************************************************************************/

#ifndef MODE_H
#define MODE_H

#include "define.h"
#include <opencv2/opencv.hpp>

class Mode
{
public:
    Mode();
    ~Mode();

    void set_simple(char mode[], uint item_num);
    void set_detail(char *argv[]);
    void print_param();

    typedef struct data_size
    {
        uint in_width;
        uint in_height;
        uint in_ch;
        uint cal_width;
        uint cal_height;
        uint cal_ch;
        uint trim_x;
        uint trim_y;
    } data_size_st;

    typedef struct app_param
    {
        std::string mode;   // drp, cpu
        std::string image;  // file, usb, ec22, ec25, stereo
        std::string format; // data, yuyv, mjpg
        uint fps;           // file:0  camera:30,60,120
        std::string left;   // left_folder_name, video0, video2
        std::string right;  // right_folder_name, video1, video3
        uint size_no;       // image size number
        uint p1;
        uint p2;
        uint disparity;
        uint win_x;
        uint win_y;
        uint remap;
        uint uniquenessRatio;
        uint gamma;
        uint sharp;
        uint heat_map;
    } app_param_st;

    data_size_st sz;
    app_param_st pr;

private:
    uint i_num;

    data_size_st data_size_tbl[14] = {
        /*+--input---+ +-calc/output-+ +-trim(x,y)-+ */
        { 640,  480, 3,  384,  288,  1,     0,     0},  /* 00 */
        { 640,  480, 3,  640,  480,  1,     0,     0},  /* 01 */
	
        {1280,  720, 3,  384,  288,  1, 320/2,     0},  /* 02 */
        {1280,  720, 3,  640,  480,  1, 320/2,     0},  /* 03 */
        {1280,  720, 3, 1280,  720,  1,     0,     0},  /* 04 */
	
        {1920, 1080, 3,  384,  288,  1, 480/2,     0},  /* 05 */
        {1920, 1080, 3,  640,  480,  1, 480/2,     0},  /* 06 */
        {1920, 1080, 3, 1280,  720,  1,     0,     0},  /* 07 */
        {1920, 1080, 3, 1920, 1080,  1,     0,     0},  /* 08 */
	
        { 881,  400, 3,  384,  288,  1, 348/2,     0},  /* 09 */
        { 881,  400, 3,  700,  320,  1,   6/2,     0},  /* 10 */

        {1280,  960, 3,  384,  288,  1, 320/2, 240/2},  /* 11 */
        {1280,  960, 3,  640,  480,  1,     0,     0},  /* 12 */
        {1280,  960, 3, 1280,  720,  1,     0, 240/2},  /* 13 */
    };


    app_param_st app_param_tbl[100] = {
        /* mode image format fps  left                                       right                                   size  p1  p2 dep win_x win_y remap unRatio gamma sharp heatmap */
        /* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
        {"xxx","usb" ,"YUYV", 30,"video0",                                  "video2"                                 ,  0, 10, 30, 64, 9,    9,    0,     0,     0,     0,      20},  /* 00 */
        {"xxx","usb" ,"YUYV", 30,"video0",                                  "video2"                                 ,  1, 10, 30, 64, 9,    9,    0,     0,     0,     0,       1},  /* 01 */
                                                                                                   
        {"xxx","usb" ,"YUYV", 30,"video0",                                  "video2"                                 ,  2, 10, 30, 64, 9,    9,    0,     0,     0,     0,      20},  /* 02 */
        {"xxx","usb" ,"YUYV", 30,"video0",                                  "video2"                                 ,  3, 10, 30, 64, 9,    9,    0,     0,     0,     0,       1},  /* 03 */
        {"xxx","usb" ,"YUYV", 30,"video0",                                  "video2"                                 ,  4, 10, 30, 96, 9,    9,    0,     0,     0,     0,       1},  /* 04 */
                                                                                                   
        {"xxx","usb" ,"YUYV", 30,"video0",                                  "video2"                                 ,  5, 10, 30, 64, 9,    9,    0,     0,     0,     0,      20},  /* 05 */
        {"xxx","usb" ,"YUYV", 30,"video0",                                  "video2"                                 ,  6, 10, 30, 64, 9,    9,    0,     0,     0,     0,       1},  /* 06 */
        {"xxx","usb" ,"YUYV", 30,"video0",                                  "video2"                                 ,  7, 10, 30, 96, 9,    9,    0,     0,     0,     0,       1},  /* 07 */
        {"xxx","usb" ,"YUYV", 30,"video0",                                  "video2"                                 ,  8, 10, 30,128, 9,    9,    0,     0,     0,     0,       1},  /* 08 */
                                                                                                   
        {"xxx",""    ,""    ,  0,""                                         ,""                                      ,  0,  0,  0,  0, 0,    0,    0,     0,     0,     0,       0},  /* 09 */
        /* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
        {"xxx","usb" ,"MJPG", 30,"video0",                                  "video2"                                 ,  0, 10, 30, 64, 9,    9,    0,     0,     0,     0,      20},  /* 10 */
        {"xxx","usb" ,"MJPG", 30,"video0",                                  "video2"                                 ,  1, 10, 30, 64, 9,    9,    0,     0,     0,     0,       1},  /* 11 */
                                                                                                  
        {"xxx","usb" ,"MJPG", 30,"video0",                                  "video2"                                 ,  2, 10, 30, 64, 9,    9,    0,     0,     0,     0,      20},  /* 12 */
        {"xxx","usb" ,"MJPG", 30,"video0",                                  "video2"                                 ,  3, 10, 30, 64, 9,    9,    0,     0,     0,     0,       1},  /* 13 */
        {"xxx","usb" ,"MJPG", 30,"video0",                                  "video2"                                 ,  4, 10, 30, 96, 9,    9,    0,     0,     0,     0,       1},  /* 14 */
                                                                                                   
        {"xxx","usb" ,"MJPG", 30,"video0",                                  "video2"                                 ,  5, 10, 30, 64, 9,    9,    0,     0,     0,     0,      20},  /* 15 */
        {"xxx","usb" ,"MJPG", 30,"video0",                                  "video2"                                 ,  6, 10, 30, 64, 9,    9,    0,     0,     0,     0,       1},  /* 16 */
        {"xxx","usb" ,"MJPG", 30,"video0",                                  "video2"                                 ,  7, 10, 30, 96, 9,    9,    0,     0,     0,     0,       1},  /* 17 */
        {"xxx","usb" ,"MJPG", 30,"video0",                                  "video2"                                 ,  8, 10, 30,128, 9,    9,    0,     0,     0,     0,       1},  /* 18 */
        {"xxx",""    ,""    ,  0,""                                         ,""                                      ,  0,  0,  0,  0, 0,    0,    0,     0,     0,     0,       0},  /* 19 */
        /* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
        {"xxx","data","file",  0,"images_stereo/take3-L-640",               "images_stereo/take3-R-640"              ,  0, 10, 30, 64, 9,    9,    0,     0,     0,     0,      20},  /* 20 */
        {"xxx","data","file",  0,"images_stereo/take3-L-640",               "images_stereo/take3-R-640"              ,  1, 10, 30, 64, 9,    9,    0,     0,     0,     0,       1},  /* 21 */
                                                                                                              
        {"xxx","data","file",  0,"images_stereo/take3-L-1280",              "images_stereo/take3-R-1280"             ,  2, 10, 30, 64, 9,    9,    0,     0,     0,     0,      20},  /* 22 */
        {"xxx","data","file",  0,"images_stereo/take3-L-1280",              "images_stereo/take3-R-1280"             ,  3, 10, 30, 64, 9,    9,    0,     0,     0,     0,       1},  /* 23 */
        {"xxx","data","file",  0,"images_stereo/take3-L-1280",              "images_stereo/take3-R-1280"             ,  4, 10, 30, 96, 9,    9,    0,     0,     0,     0,       1},  /* 24 */
                                                                                                              
        {"xxx","data","file",  0,"images_stereo/take3-L-1920",              "images_stereo/take3-R-1920"             ,  5, 10, 30, 64, 9,    9,    0,     0,     0,     0,      20},  /* 25 */
        {"xxx","data","file",  0,"images_stereo/take3-L-1920",              "images_stereo/take3-R-1920"             ,  6, 10, 30, 64, 9,    9,    0,     0,     0,     0,       1},  /* 26 */
        {"xxx","data","file",  0,"images_stereo/take3-L-1920",              "images_stereo/take3-R-1920"             ,  7, 10, 30, 96, 9,    9,    0,     0,     0,     0,       1},  /* 27 */
        {"xxx","data","file",  0,"images_stereo/take3-L-1920",              "images_stereo/take3-R-1920"             ,  8, 10, 30,128, 9,    9,    0,     0,     0,     0,       1},  /* 28 */
                                                                                                   
        {"xxx",""    ,""    ,  0,""                                         ,""                                      ,  0,  0,  0,  0, 0,    0,    0,     0,     0,     0,       0},  /* 29 */
        /* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
        {"xxx","data","file",  0,"drivingstereo/2018-07-11-14-48-52-L-811", "drivingstereo/2018-07-11-14-48-52-R-811",  9, 10, 30, 64, 9,    9,    0,     0,     0,     0,      20},  /* 30 */
        {"xxx","data","file",  0,"drivingstereo/2018-07-11-14-48-52-L-811", "drivingstereo/2018-07-11-14-48-52-R-811", 10, 10, 30, 96, 9,    9,    0,     0,     0,     0,       1},  /* 31 */
        /* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
        {"xxx",""    ,""    ,  0,""                                         ,""                                      ,  0,  0,  0,  0, 0,    0,    0,     0,     0,     0,       0},  /* 32 */
        {"xxx",""    ,""    ,  0,""                                         ,""                                      ,  0,  0,  0,  0, 0,    0,    0,     0,     0,     0,       0},  /* 33 */
        {"xxx",""    ,""    ,  0,""                                         ,""                                      ,  0,  0,  0,  0, 0,    0,    0,     0,     0,     0,       0},  /* 34 */
        {"xxx",""    ,""    ,  0,""                                         ,""                                      ,  0,  0,  0,  0, 0,    0,    0,     0,     0,     0,       0},  /* 35 */
        {"xxx",""    ,""    ,  0,""                                         ,""                                      ,  0,  0,  0,  0, 0,    0,    0,     0,     0,     0,       0},  /* 36 */
        {"xxx",""    ,""    ,  0,""                                         ,""                                      ,  0,  0,  0,  0, 0,    0,    0,     0,     0,     0,       0},  /* 37 */
        {"xxx",""    ,""    ,  0,""                                         ,""                                      ,  0,  0,  0,  0, 0,    0,    0,     0,     0,     0,       0},  /* 38 */
        {"xxx",""    ,""    ,  0,""                                         ,""                                      ,  0,  0,  0,  0, 0,    0,    0,     0,     0,     0,       0},  /* 39 */
        /* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
        {"xxx","ec22","YUYV", 30,"video0",                                  "video1"                                 ,  0, 10, 30, 64, 9,    9,    0,     0,     0,     0,      20},  /* 40 */
        {"xxx","ec22","YUYV", 30,"video0",                                  "video1"                                 ,  1, 10, 30, 64, 9,    9,    0,     0,     0,     0,       1},  /* 41 */
                                                                                                                 
        {"xxx","ec22","YUYV", 30,"video0",                                  "video1"                                 ,  2, 10, 30, 64, 9,    9,    0,     0,     0,     0,      20},  /* 42 */
        {"xxx","ec22","YUYV", 30,"video0",                                  "video1"                                 ,  3, 10, 30, 64, 9,    9,    0,     0,     0,     0,       1},  /* 43 */
        {"xxx","ec22","YUYV", 30,"video0",                                  "video1"                                 ,  4, 10, 30, 96, 9,    9,    0,     0,     0,     0,       1},  /* 44 */
                                                                                                                 
        {"xxx","ec22","YUYV", 30,"video0",                                  "video1"                                 ,  5, 10, 30, 64, 9,    9,    0,     0,     0,     0,      20},  /* 45 */
        {"xxx","ec22","YUYV", 30,"video0",                                  "video1"                                 ,  6, 10, 30, 64, 9,    9,    0,     0,     0,     0,       1},  /* 46 */
        {"xxx","ec22","YUYV", 30,"video0",                                  "video1"                                 ,  7, 10, 30, 96, 9,    9,    0,     0,     0,     0,       1},  /* 47 */
        {"xxx","ec22","YUYV", 30,"video0",                                  "video1"                                 ,  8, 10, 30,128, 9,    9,    0,     0,     0,     0,       1},  /* 48 */
                                                                                                   
        {"xxx",""    ,""    ,  0,""                                         ,""                                      ,  0,  0,  0,  0, 0,    0,    0,     0,     0,     0,       0},  /* 49 */
        /* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
        {"xxx","ec25","YUYV", 30,"video0",                                  "video1"                                 ,  0, 10, 30, 64, 9,    9,    0,     0,     0,     0,      20},  /* 50 */
        {"xxx","ec25","YUYV", 30,"video0",                                  "video1"                                 ,  1, 10, 30, 64, 9,    9,    0,     0,     0,     0,       1},  /* 51 */
                                                                                                             
        {"xxx","ec25","YUYV", 30,"video0",                                  "video1"                                 ,  2, 10, 30, 64, 9,    9,    0,     0,     0,     0,      20},  /* 52 */
        {"xxx","ec25","YUYV", 30,"video0",                                  "video1"                                 ,  3, 10, 30, 64, 9,    9,    0,     0,     0,     0,       1},  /* 53 */
        {"xxx","ec25","YUYV", 30,"video0",                                  "video1"                                 ,  4, 10, 30, 96, 9,    9,    0,     0,     0,     0,       1},  /* 54 */
                                                                                                             
        {"xxx","ec25","YUYV", 30,"video0",                                  "video1"                                 ,  5, 10, 30, 64, 9,    9,    0,     0,     0,     0,      20},  /* 55 */
        {"xxx","ec25","YUYV", 30,"video0",                                  "video1"                                 ,  6, 10, 30, 64, 9,    9,    0,     0,     0,     0,       1},  /* 56 */
        {"xxx","ec25","YUYV", 30,"video0",                                  "video1"                                 ,  7, 10, 30, 96, 9,    9,    0,     0,     0,     0,       1},  /* 57 */
        {"xxx","ec25","YUYV", 30,"video0",                                  "video1"                                 ,  8, 10, 30,128, 9,    9,    0,     0,     0,     0,       1},  /* 58 */
                                                                                                  
        {"xxx",""    ,""    ,  0,""                                         ,""                                      ,  0,  0,  0,  0, 0,    0,    0,     0,     0,     0,       0},  /* 59 */
        /* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
        {"xxx","stereo","YUYV", 120,"video0",                                "tmp"                                   ,  0, 10, 30, 64, 9,    9,    0,     0,     0,     0,      20},  /* 60 */
        {"xxx","stereo","YUYV", 120,"video0",                                "tmp"                                   ,  1, 10, 30, 64, 9,    9,    0,     0,     0,     0,       1},  /* 61 */

        {"xxx","stereo","YUYV",  30,"video0",                                "tmp"                                   , 11, 10, 30, 64, 9,    9,    0,     0,     0,     0,      20},  /* 62 */
        {"xxx","stereo","YUYV",  30,"video0",                                "tmp"                                   , 12, 10, 30, 64, 9,    9,    0,     0,     0,     0,       1},  /* 63 */
        {"xxx","stereo","YUYV",  30,"video0",                                "tmp"                                   , 13, 10, 30, 96, 9,    9,    0,     0,     0,     0,       1},  /* 64 */
                                                                                                                 
        {"xxx",""    ,""    ,  0,""                                         ,""                                      ,  0,  0,  0,  0, 0,    0,    0,     0,     0,     0,       0},  /* 65 */
        {"xxx",""    ,""    ,  0,""                                         ,""                                      ,  0,  0,  0,  0, 0,    0,    0,     0,     0,     0,       0},  /* 66 */
        {"xxx",""    ,""    ,  0,""                                         ,""                                      ,  0,  0,  0,  0, 0,    0,    0,     0,     0,     0,       0},  /* 67 */
        {"xxx",""    ,""    ,  0,""                                         ,""                                      ,  0,  0,  0,  0, 0,    0,    0,     0,     0,     0,       0},  /* 68 */
                                                                                                   
        {"xxx",""    ,""    ,  0,""                                         ,""                                      ,  0,  0,  0,  0, 0,    0,    0,     0,     0,     0,       0},  /* 69 */
        /* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
    };
};

#endif
