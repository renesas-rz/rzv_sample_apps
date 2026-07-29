/***********************************************************************************************************************
* Copyright (C) 2023-2026 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name    : mode_seg_stereo.cpp
* Version      : R2026-07
* Description  : RZ/V2H DRP-AI Sample Application for Ultralytics Detection YOLOv8-seg and Stereo with MIPI/USB Camera
***********************************************************************************************************************/

/*****************************************
 * Includes
 ******************************************/
#include "mode_seg_stereo.h"

Mode::Mode()
{
    mode                 = STEREO_CALC_MODE;  // OCA, CPU

    df.cam_name          = INPUT_CAM_NAME;
    df.cam_resolution    = CAM_RESOLUTION;
    df.left              = IMAGE_LEFT_DR;
    df.right             = IMAGE_RIGHT_DR;
    df.output            = IMAGE_OUTPUT;

    df.cam_calib_en      = CAM_CALIB_EN;
    df.cam_left_calib    = CAM_LEFT_CALIB;
    df.cam_right_calib   = CAM_RIGHT_CALIB;
    df.cam_left_dist     = CAM_LEFT_DIST;
    df.cam_right_dist    = CAM_RIGHT_DIST;

    uint i_num = 25;
    pr = (df.cam_resolution == "1280x720")  ? app_param_tbl_hd[i_num] : app_param_tbl_vga[i_num];

    focus_cnt         = 0;
    focus_st          = 0;
}

Mode::~Mode()
{
}

/*****************************************
 * Function Name : set_simple
 ******************************************/
void Mode::set_simple(char *argv[])
{
  uint i_num = atoi(argv[2]);

  pr = (df.cam_resolution == "1280x720")  ? app_param_tbl_hd[i_num] : app_param_tbl_vga[i_num];
  std::cout << "item_num : " << i_num << std::endl;
}

/*****************************************
 * Function Name : set_detail
 ******************************************/
void Mode::set_detail(char *argv[])
{
  pr.width       = atoi(argv[2]);
  pr.height      = atoi(argv[3]);
  pr.p1          = atoi(argv[4]);
  pr.p2          = atoi(argv[5]);
  pr.disparity   = atoi(argv[6]);
  pr.win_x       = atoi(argv[7]);
  pr.win_y       = atoi(argv[8]);
  pr.uniquenessRatio = atoi(argv[9]);
  pr.heat_map    = atoi(argv[10]);
  pr.focus_en   = atoi(argv[11]);
  pr.focus_cnt_max = atoi(argv[12]);
  pr.tx          = atof(argv[13]);
  pr.ty          = atof(argv[14]);
  pr.angle       = atof(argv[15]);
  
  pr.inf_wait_en = atoi(argv[16]);
}

/*****************************************
 * Function Name : print_param
 ******************************************/
void Mode::print_param()
{
  std::cout << "------------------------------------------------" << std::endl;
  std::cout << "<<< Input / Output setting >>>" << std::endl;
  std::cout << "Mode: " << df.cam_name << ", "
            << "Left: " << df.left << ", "
            << "Right: " << df.right << ", "
            << "Output: " << df.output << " (When using Input Image Mode)"
            << std::endl;
  std::cout << "------------------------------------------------" << std::endl;
  std::cout << "<<< Stereo Command / Parameter setting >>>" << std::endl;
  std::cout << "./app_yolov8_seg_stereo_cam " << " "
            << mode << " "

            << pr.width << " "
            << pr.height << " "
            << pr.p1 << " "
            << pr.p2 << " "
            << pr.disparity << " "
            << pr.win_x << " "
            << pr.win_y << " "
            << pr.uniquenessRatio << " "
            << pr.heat_map << " "
    
            << pr.focus_en << " "
            << pr.focus_cnt_max << " "
            << pr.tx << " "
            << pr.ty << " "
            << pr.angle << " "

            << pr.inf_wait_en << " "
            << std::endl;
  std::cout << "------------------------------------------------" << std::endl;
}

