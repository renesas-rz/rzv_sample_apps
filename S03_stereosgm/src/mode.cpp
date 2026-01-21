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
 * File Name    : mode.cpp
 * Version      : v3.10
 * Description  : RZ/V2H AI SDK Sample Application for StereoSGM
 ***********************************************************************************************************************/

/*****************************************
 * Includes
 ******************************************/
#include "mode.h"
Mode::Mode()
{
}

Mode::~Mode()
{
}

/*****************************************
 * Function Name : print_param
 ******************************************/
void Mode::print_param()
{
  std::cout << "-------------------------------" << std::endl;
  std::cout << "./stereo_app " << " "
            << pr.mode << " "
            << pr.image << " "
            << pr.format << " "
            << pr.fps << " "
            << pr.left << " "
            << pr.right << " "
            << pr.size_no << " "
            << pr.p1 << " "
            << pr.p2 << " "
            << pr.disparity << " "
            << pr.win_x << " "
            << pr.win_y << " "
            << pr.remap << " "
            << pr.uniquenessRatio << " "
            << pr.gamma << " "
            << pr.sharp << " "
            << pr.heat_map << " "
            << std::endl;

  std::cout << "-------------------------------" << std::endl;
  std::cout << "in_width:" << " "
            << sz.in_width << " "
            << "in_height:" << " "
            << sz.in_height << " "
            << "in_channel:" << " "
            << sz.in_ch << " "
            << "cal_width:" << " "
            << sz.cal_width << " "
            << "cal_height:" << " "
            << sz.cal_height << " "
            << "cal_channel:" << " "
            << sz.cal_ch << " "
            << "trm_width:" << " "
            << sz.trim_x << " "
            << "trm_height:" << " "
            << sz.trim_y << " "
            << std::endl;
}
/*****************************************
 * Function Name : set_simple
 ******************************************/
void Mode::set_simple(char mode[], uint item_num)
{
  i_num = item_num;
  pr = app_param_tbl[i_num];
  pr.mode = mode;

  sz = data_size_tbl[pr.size_no];

  std::cout << "item_num : " << i_num << std::endl;

  print_param();
}
/*****************************************
 * Function Name : set_detail
 ******************************************/
void Mode::set_detail(char *argv[])
{
  pr.mode = argv[1];
  pr.image = argv[2];
  pr.format = argv[3];
  pr.fps = atoi(argv[4]);
  pr.left = argv[5];
  pr.right = argv[6];
  pr.size_no = atoi(argv[7]);
  pr.p1 = atoi(argv[8]);
  pr.p2 = atoi(argv[9]);
  pr.disparity = atoi(argv[10]);
  pr.win_x = atoi(argv[11]);
  pr.win_y = atoi(argv[12]);
  pr.remap = atoi(argv[13]);
  pr.uniquenessRatio = atoi(argv[14]);
  pr.gamma = atoi(argv[15]);
  pr.sharp = atoi(argv[16]);
  pr.heat_map = atoi(argv[17]);

  sz = data_size_tbl[pr.size_no];

  print_param();
}
