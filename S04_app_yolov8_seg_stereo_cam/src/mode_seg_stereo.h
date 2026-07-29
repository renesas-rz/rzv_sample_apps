/***********************************************************************************************************************
* Copyright (C) 2023-2026 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name    : mode_seg_stereo.h
* Version      : R2026-07
* Description  : RZ/V2H DRP-AI Sample Application for Ultralytics Detection YOLOv8-seg abd Stereo with MIPI/USB Camera
***********************************************************************************************************************/

#ifndef MODE_SEG_STEREO_H
#define MODE_SEG_STEREO_H

#include "define.h"
#include <opencv2/opencv.hpp>

class Mode
{
public:
    Mode();
    ~Mode();

    void print_param();
    void set_detail(char *argv[]);
    void set_simple(char *argv[]);

    typedef struct app_define
    {
        std::string cam_name;       // usb, mipi, file
        std::string cam_resolution; // 640x480, 1280x720, 1920x1080
        std::string left;           // video0(mipi), video0(usb), left_folder_name,
        std::string right;          // video1(mipi), video2(usb), right_folder_name,
        std::string output;         // output file name
        uint        cam_calib_en;
        std::string cam_left_calib;
        std::string cam_right_calib;
        std::string cam_left_dist;
        std::string cam_right_dist;
    } app_define_st;

  typedef struct app_param
    {
        uint width;         // stereo in width
        uint height;

        uint p1;
        uint p2;
        uint disparity;
        uint win_x;
        uint win_y;
        uint uniquenessRatio;
        uint heat_map;

        uint focus_en;     // 0:manual, 1:auto
        int focus_cnt_max;
        double tx;
        double ty;
        double angle;

        uint inf_wait_en;     
    } app_param_st;

    app_define_st df;
    app_param_st pr;

    std::string mode;    // oca, cpu
    double mode_param;   // oca:1.0, cpu:16.0
    int focus_cnt;
    int focus_st;

private:
    app_param_st app_param_tbl_vga[100] = {
       /* width height p1  p2  disp win_x win_y unRatio heatmap focus_en cam_tran_cnt_max  tx   ty  angle   wait */
       /* -------------------------------------------------------------------------------------------------------------------- */
         { 640,  480,  10, 30,  64,   7,    7,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 00 */
         { 640,  480,  10, 30, 128,   7,    7,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 01 */
         { 640,  480,  10, 30, 192,   7,    7,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 02 */
         { 640,  480,  10, 30, 256,   7,    7,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 03 */

         { 640,  480,  10, 30,  64,   9,    9,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 04 */
         { 640,  480,  10, 30, 128,   9,    9,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 05 */
         { 640,  480,  10, 30, 192,   9,    9,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 06 */
         { 640,  480,  10, 30, 256,   9,    9,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 07 */

         { 640,  480,  10, 30, 128,   9,    9,    10,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 08 */
         { 640,  480,  10, 30, 128,   9,    9,    20,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 09 */
       /* -------------------------------------------------------------------------------------------------------------------- */
         { 480,  360,  10, 30,  48,   7,    7,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 10 */
         { 480,  360,  10, 30,  96,   7,    7,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 11 */
         { 480,  360,  10, 30, 144,   7,    7,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 12 */
         { 480,  360,  10, 30, 192,   7,    7,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 13 */

         { 480,  360,  10, 30,  48,   9,    9,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 14 */
         { 480,  360,  10, 30,  96,   9,    9,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 15 */
         { 480,  360,  10, 30, 144,   9,    9,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 16 */
         { 480,  360,  10, 30, 192,   9,    9,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 17 */

         { 480,  360,  10, 30,  96,   9,    9,    10,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 18 */
         { 480,  360,  10, 30,  96,   9,    9,    20,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 19 */
       /* -------------------------------------------------------------------------------------------------------------------- */
         { 320,  240,  10, 30,  32,   7,    7,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 20 */
         { 320,  240,  10, 30,  64,   7,    7,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 21 */
         { 320,  240,  10, 30,  96,   7,    7,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 22 */
         { 320,  240,  10, 30, 128,   7,    7,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 23 */

         { 320,  240,  10, 30,  32,   9,    9,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 24 */
         { 320,  240,  10, 30,  64,   9,    9,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 25 */
         { 320,  240,  10, 30,  96,   9,    9,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 26 */
         { 320,  240,  10, 30, 128,   9,    9,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 27 */

         { 320,  240,  10, 30,  64,   9,    9,    10,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 28 */
         { 320,  240,  10, 30,  64,   9,    9,    20,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 29 */
       /* -------------------------------------------------------------------------------------------------------------------- */
         { 240,  180,  10, 30,  32,   7,    7,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 30 */
         { 240,  180,  10, 30,  64,   7,    7,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 31 */
         { 240,  180,  10, 30,  96,   7,    7,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 32 */
         { 240,  180,  10, 30, 128,   7,    7,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 33 */

         { 240,  180,  10, 30,  32,   9,    9,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 34 */
         { 240,  180,  10, 30,  64,   9,    9,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 35 */
         { 240,  180,  10, 30,  96,   9,    9,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 36 */
         { 240,  180,  10, 30, 128,   9,    9,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 37 */

         { 240,  180,  10, 30,  64,   9,    9,    10,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 38 */
         { 240,  180,  10, 30,  64,   9,    9,    20,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 39 */

    };
  
    app_param_st app_param_tbl_hd[100] = {
       /* width height p1  p2  disp win_x win_y unRatio heatmap focus_en cam_tran_cnt_max  tx   ty  angle   wait */
       /* -------------------------------------------------------------------------------------------------------------------- */
         { 640,  360,  10, 30,  64,   7,    7,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 00 */
         { 640,  360,  10, 30, 128,   7,    7,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 01 */
         { 640,  360,  10, 30, 192,   7,    7,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 02 */
         { 640,  360,  10, 30, 256,   7,    7,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 03 */

         { 640,  360,  10, 30,  64,   9,    9,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 04 */
         { 640,  360,  10, 30, 128,   9,    9,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 05 */
         { 640,  360,  10, 30, 192,   9,    9,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 06 */
         { 640,  360,  10, 30, 256,   9,    9,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 07 */

         { 640,  360,  10, 30, 128,   9,    9,    10,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 08 */
         { 640,  360,  10, 30, 128,   9,    9,    20,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 09 */
       /* -------------------------------------------------------------------------------------------------------------------- */
         { 480,  272,  10, 30,  48,   7,    7,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 10 */
         { 480,  272,  10, 30,  96,   7,    7,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 11 */
         { 480,  272,  10, 30, 144,   7,    7,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 12 */
         { 480,  272,  10, 30, 192,   7,    7,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 13 */

         { 480,  272,  10, 30,  48,   9,    9,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 14 */
         { 480,  272,  10, 30,  96,   9,    9,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 15 */
         { 480,  272,  10, 30, 144,   9,    9,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 16 */
         { 480,  272,  10, 30, 192,   9,    9,     0,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 17 */

         { 480,  272,  10, 30,  96,   9,    9,    10,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 18 */
         { 480,  272,  10, 30,  96,   9,    9,    20,      1,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 19 */
       /* -------------------------------------------------------------------------------------------------------------------- */
         { 320,  180,  10, 30,  32,   7,    7,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 20 */
         { 320,  180,  10, 30,  64,   7,    7,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 21 */
         { 320,  180,  10, 30,  96,   7,    7,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 22 */
         { 320,  180,  10, 30, 128,   7,    7,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 23 */

         { 320,  180,  10, 30,  32,   9,    9,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 24 */
         { 320,  180,  10, 30,  64,   9,    9,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 25 */
         { 320,  180,  10, 30,  96,   9,    9,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 26 */
         { 320,  180,  10, 30, 128,   9,    9,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 27 */

         { 320,  180,  10, 30,  64,   9,    9,    10,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 28 */
         { 320,  180,  10, 30,  64,   9,    9,    20,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 29 */
       /* -------------------------------------------------------------------------------------------------------------------- */
         { 240,  136,  10, 30,  32,   7,    7,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 30 */
         { 240,  136,  10, 30,  64,   7,    7,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 31 */
         { 240,  136,  10, 30,  96,   7,    7,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 32 */
         { 240,  136,  10, 30, 128,   7,    7,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 33 */

         { 240,  136,  10, 30,  32,   9,    9,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 34 */
         { 240,  136,  10, 30,  64,   9,    9,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 35 */
         { 240,  136,  10, 30,  96,   9,    9,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 36 */
         { 240,  136,  10, 30, 128,   9,    9,     0,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 37 */

         { 240,  136,  10, 30,  64,   9,    9,    10,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 38 */
         { 240,  136,  10, 30,  64,   9,    9,    20,     20,       1,  CAM_TRAN_CNT_MAX,  0.0, 0.0, 0.0,  INF_WAIT_EN },  /* 39 */
    };    
};

extern Mode md;

#endif
