/***********************************************************************************************************************
* Copyright (C) 2023-2026 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name    : define.h
* Version      : R2026-07
* Description  : RZ/V2H DRP-AI Sample Application for Ultralytics Detection YOLOv8-seg abd Stereo with MIPI/USB Camera
***********************************************************************************************************************/

#ifndef DEFINE_MACRO_H
#define DEFINE_MACRO_H

/*****************************************
* includes
******************************************/
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <errno.h>
#include <vector>
#include <map>
#include <fstream>
#include <math.h>
#include <iomanip>
#include <cstring>
#include <float.h>
#include <atomic>
#include <semaphore.h>
#include <numeric>
#include <array>
#include <iostream>
#include <string>
#include <algorithm>
#include <filesystem>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"

/*************************************************
* Macro for YOLOv8-seg and Stereo
**************************************************/
/*************************************************/
/* Input Camera support                          */
/* n = 0: USB Camera(VGA)                        */
/* n = 1: eCAM22(FHD)                            */
/* n = 2: eCAM25(FHD)                            */
/* n = 3: input image(wait for inference)        */
#define INPUT_CAM_TYPE 0
/*************************************************/

/* USB Camera VGA */
#if   INPUT_CAM_TYPE == 0
    #define INPUT_CAM_NAME   "USB Camera"
    #define CAP_BUF_NUM      (3)
    #define CAM_INPUT_VGA
    #define CAM_RESOLUTION   "640x480"
    #define CAM_BASE         (0.04f)    // [m] Distance between left and right camera
    //#define CAM_BASE       (0.08f)    // [m] Distance between left and right camera
    #define CAM_F_LENGTH     (680.0f)   // (503.0f)   // [pixel]
    #define CAM_TRAN_CNT_MAX (100)      // CAM_BASE=0.04 : 100, CAM_BASE=0.16 : 200 
    #define IMAGE_LEFT_DR    "video0"
    #define IMAGE_RIGHT_DR   "video2"
    #define INF_WAIT_EN      (0)        // Input Image mode only.
    #define CAM_CALIB_EN     (0)
    #define CAM_LEFT_CALIB   "calib_data/camera_USB_left_YUYV_640_480.csv";
    #define CAM_RIGHT_CALIB  CAM_LEFT_CALIB
    //#define CAM_RIGHT_CALIB  "calib_data/camera_USB_right_YUYV_640_480.csv";
    #define CAM_LEFT_DIST    "calib_data/dist_USB_left_YUYV_640_480.csv";
    #define CAM_RIGHT_DIST   CAM_LEFT_DIST
    //#define CAM_RIGHT_DIST   "calib_data/dist_USB_right_YUYV_640_480.csv";
    #define DISPLAY_CUT_INVALID_EREA

/* eCAM22 VGA */
#elif INPUT_CAM_TYPE == 1
    #define INPUT_CAM_NAME   "EC22 MIPI Camera"  // 1/2.6" Optical format CMOS image sensor
    #define CAP_BUF_NUM      (6)
    #define CAM_INPUT_VGA
    #define CAM_RESOLUTION   "640x480"
    #define MIPI_CAM_RES     CAM_RESOLUTION
    #define CAM_BASE         (0.04f)    // [m] Distance between left and right camera
    //#define CAM_BASE       (0.16f)    // [m] Distance between left and right camera
    #define CAM_F_LENGTH     (468.2f)   // [pixel] e-CAM22 : Focal Length 2.8mm, Pixel Size 2.9um
    #define CAM_TRAN_CNT_MAX (200)      // CAM_BASE=0.04 : 100, CAM_BASE=0.16 : 200 
    #define IMAGE_LEFT_DR    "video0"
    #define IMAGE_RIGHT_DR   "video1"
    #define INF_WAIT_EN      (0)        // Input Image mode only.
    #define CAM_CALIB_EN     (1)
    #define CAM_LEFT_CALIB   "calib_data/camera_EC22_left_YUYV_640_480.csv";
    #define CAM_RIGHT_CALIB  CAM_LEFT_CALIB 
    //#define CAM_RIGHT_CALIB  "calib_data/camera_EC22_right_YUYV_640_480.csv";
    #define CAM_LEFT_DIST    "calib_data/dist_EC22_left_YUYV_640_480.csv";
    #define CAM_RIGHT_DIST   CAM_LEFT_DIST
    //#define CAM_RIGHT_DIST   "calib_data/dist_EC22_right_YUYV_640_480.csv";
    #define DISPLAY_CUT_INVALID_EREA

/* eCAM25 HD */
#elif INPUT_CAM_TYPE == 2
#define INPUT_CAM_NAME   "EC25 MIPI Camera"   // 1/2.6" Optical format CMOS image sensor
    #define CAP_BUF_NUM      (6)
    #define CAM_INPUT_HD
    #define CAM_RESOLUTION   "1280x720"
    #define MIPI_CAM_RES     CAM_RESOLUTION
    //#define CAM_BASE       (0.04f)    // [m] Distance between left and right camera
    #define CAM_BASE         (0.16f)    // [m] Distance between left and right camera
    #define CAM_F_LENGTH     (731.2f)   // [pixel] e-CAM25 : Focal Length 3.0mm, Pixel Size 3.0um
    #define CAM_TRAN_CNT_MAX (200)      // CAM_BASE=0.04 : 100, CAM_BASE=0.16 : 200 
    #define IMAGE_LEFT_DR    "video0"
    #define IMAGE_RIGHT_DR   "video1"
    #define INF_WAIT_EN      (0)        // Input Image mode only.
    #define CAM_CALIB_EN     (1)
    #define CAM_LEFT_CALIB   "calib_data/camera_EC25_left_YUYV_1280_720.csv";
    #define CAM_RIGHT_CALIB  CAM_LEFT_CALIB
    //#define CAM_RIGHT_CALIB  "calib_data/camera_EC25_right_YUYV_1280_720.csv";
    #define CAM_LEFT_DIST    "calib_data/dist_EC25_left_YUYV_1280_720.csv";
    #define CAM_RIGHT_DIST   CAM_LEFT_DIST
    //#define CAM_RIGHT_DIST   "calib_data/dist_EC25_right_YUYV_1280_720.csv";
    #define DISPLAY_CUT_INVALID_EREA

#elif INPUT_CAM_TYPE == 3
    #define INPUT_CAM_NAME   "Input Image"
    #define CAP_BUF_NUM      (0)
    #define INPUT_IMAGE
    #define CAM_INPUT_VGA
    #define CAM_RESOLUTION   "640x480"
//    #define CAM_INPUT_HD
//    #define CAM_RESOLUTION   "1280x720"
    #define CAM_BASE         (0.20f)    // [m] Distance between left and right camera
    #define CAM_F_LENGTH     (500.0f)   // [pixel]
    #define CAM_TRAN_CNT_MAX (0)        // CAM_BASE=0.04 : 100, CAM_BASE=0.16 : 200 
    #define IMAGE_LEFT_DR    "images_left_"  CAM_RESOLUTION
    #define IMAGE_RIGHT_DR   "images_right_" CAM_RESOLUTION
    #define INF_WAIT_EN      (1)        // Input Image mode only.
    #define CAM_CALIB_EN     (0)
    #define CAM_LEFT_CALIB   "";
    #define CAM_RIGHT_CALIB  "";
    #define CAM_LEFT_DIST    "";
    #define CAM_RIGHT_DIST   "";
    #define DISPLAY_CUT_INVALID_EREA

#else
    #define INPUT_CAM_NAME   "USB Camera"
    #define CAP_BUF_NUM      (3)
    #define CAM_INPUT_VGA
    #define CAM_RESOLUTION   "640x480"
    #define CAM_BASE         (0.08f)   // [m] Distance between left and right camera
    #define CAM_F_LENGTH     (680.0f)   // (503.0f)   // [pixel]
    #define CAM_TRAN_CNT_MAX (100)      // CAM_BASE=0.04 : 100, CAM_BASE=0.16 : 200 
    #define IMAGE_LEFT_DR    "video0"
    #define IMAGE_RIGHT_DR   "video2"
    #define INF_WAIT_EN      (0)        // Input Image mode only.
    #define CAM_CALIB_EN     (0)
    #define CAM_LEFT_CALIB   "calib_data/camera_USB_left_YUYV_640_480.csv";
    #define CAM_RIGHT_CALIB  CAM_LEFT_CALIB
    //#define CAM_RIGHT_CALIB  "calib_data/camera_USB_right_YUYV_640_480.csv";
    #define CAM_LEFT_DIST    "calib_data/dist_USB_left_YUYV_640_480.csv";
    #define CAM_RIGHT_DIST   CAM_LEFT_DIST
    //#define CAM_RIGHT_DIST   "calib_data/dist_USB_right_YUYV_640_480.csv";
    //#define DISPLAY_CUT_INVALID_EREA

#endif

#define IMAGE_OUTPUT     "output_"       CAM_RESOLUTION ".jpg"

/*************************************************/

/* Output Camera Size */
#define IMAGE_OUTPUT_FHD

/*Time Measurement Flag*/
//#define DEBUG_TIME_FLG

/* Enable demonstration mode for combination with GUI Demo system */
#define END_DET_TYPE                (0)

/*Display AI frame rate*/
#undef DISP_AI_FRAME_RATE

/* Padding input mode to maintain the aspect ratio of DRP-AI input image.
   This mode requires the DRP-AI object file having the squared input size CAM_IMAGE_WIDTH x CAM_IMAGE_WIDTH.
   0: No padding, 
   1: With padding (maintains the aspect ratio) */
#define DRPAI_INPUT_PADDING         (1)

/* Enable or Disable the multi-threading for CPU DFL processing.
   n = 0: Disable (single-thread for CPU DFL)
   n = 1: Enable (multi-threads for CPU DFL)
   */ 
#define CPU_DFL_MULTI_THREAD        (1)

#define ONLY_PERSON                 (0)

#if(1)  // TVM
/* DRP-AI memory offset for model object file*/
#define DRPAI_MEM_OFFSET            (0X38E0000)
#endif  // TVM

/*****************************************
* Static Variables for YOLOv8-seg and Stereo
* Following variables need to be changed in order to custormize the AI model
*  - drpai_prefix0 = directory name of DRP-AI Object files (DRP-AI Translator output)
******************************************/
#if(1)  // TVM
/* Model Binary */
const static std::string model_dir = "yolov8_seg_cam";
/* Pre-processing Runtime Object */
const static std::string pre_dir = model_dir + "/preprocess";
#endif  // TVM

/*****************************************
* Macro for Yolov8-seg and Stereo
******************************************/
/* Number of class to be detected */
#define NUM_CLASS                   (80)
/* Number for [region] layer num parameter */
#define NUM_BB                      (1)
/* Number of output layers. This value MUST match with the length of num_grids[] below */
#define NUM_INF_OUT_LAYER           (3)
/* Number of grids in the image. The length of this array MUST match with the NUM_INF_OUT_LAYER */
const static uint8_t num_grids[] = { 80,40,20 };
/* Number of DFL channel (default:16) */
#define REG_MAX                     (16)

/* Number of DRP-AI output */

const static uint32_t num_grid80 = num_grids[0] * num_grids[0];
const static uint32_t num_grid40 = num_grids[1] * num_grids[1];
const static uint32_t num_grid20 = num_grids[2] * num_grids[2];
const static uint32_t num_dfl80_out = (REG_MAX * 4) * num_grids[0] * num_grids[0];
const static uint32_t num_dfl40_out = (REG_MAX * 4) * num_grids[1] * num_grids[1];
const static uint32_t num_dfl20_out = (REG_MAX * 4) * num_grids[2] * num_grids[2];
const static uint32_t num_class80_out = NUM_CLASS * num_grids[0] * num_grids[0];
const static uint32_t num_class40_out = NUM_CLASS * num_grids[1] * num_grids[1];
const static uint32_t num_class20_out = NUM_CLASS * num_grids[2] * num_grids[2];

const static uint32_t num_inf_out =  (NUM_CLASS + 4) * NUM_BB * num_grids[0] * num_grids[0]
                                   + (NUM_CLASS + 4) * NUM_BB * num_grids[1] * num_grids[1]
                                   + (NUM_CLASS + 4) * NUM_BB * num_grids[2] * num_grids[2];

const static uint32_t num_grid_points = num_grids[0] * num_grids[0] + num_grids[1] * num_grids[1] + num_grids[2] * num_grids[2];

/* Thresholds */
#define TH_PROB                     (0.5f)
#define TH_NMS                      (0.5f)
/* Size of input image to the model */
#define MODEL_IN_W                  (640)
#define MODEL_IN_H                  (640)

/*****************************************
* Macro for Yolov8-seg and Stereo
******************************************/
// YOLOv8-seg mask channels
#define NUM_MASK                    (32)
#define PROTO_H                     (160)
#define PROTO_W                     (160)

#define TOP_K_SEG                   (100)

#define MASK_THRESH                 (0.5f)
#define MASK_ALPHA                  (0.7f)

const static uint32_t num_mask80_out = NUM_MASK * num_grids[0] * num_grids[0];
const static uint32_t num_mask40_out = NUM_MASK * num_grids[1] * num_grids[1];
const static uint32_t num_mask20_out = NUM_MASK * num_grids[2] * num_grids[2];
const static uint32_t num_proto_out  = NUM_MASK * PROTO_H * PROTO_W;

/*****************************************
* Macro for Application
******************************************/
/*Camera:: Capture Image Information*/
#ifdef CAM_INPUT_VGA
#define CAM_IMAGE_WIDTH             (640)
#define CAM_IMAGE_HEIGHT            (480)
#define STEREO_OUTPUT_WIDTH         (480)
#define STEREO_OUTPUT_HEIGHT        (360)  
#else /* CAM_INPUT_HD */
  #ifdef CAM_INPUT_HD
  #define CAM_IMAGE_WIDTH           (1280)
  #define CAM_IMAGE_HEIGHT          (720)
  #define STEREO_OUTPUT_WIDTH       (480)
  #define STEREO_OUTPUT_HEIGHT      (270)  
  #else /* CAM_INPUT_FHD */
  #define CAM_IMAGE_WIDTH           (1920)
  #define CAM_IMAGE_HEIGHT          (1080)
  #define STEREO_OUTPUT_WIDTH       (480)
  #define STEREO_OUTPUT_HEIGHT      (360)  
  #endif
#endif

#define CAM_IMAGE_FPS               (30)

#define CAM_IMAGE_CHANNEL_YUY2      (2)
#define CAM_IMAGE_SIZE              (CAM_IMAGE_WIDTH * CAM_IMAGE_HEIGHT * CAM_IMAGE_CHANNEL_YUY2)

/*DRP-AI Input image information*/
#if (1) == DRPAI_INPUT_PADDING
/*** DRP-AI input is assigned to the buffer having the size of CAM_IMAGE_WIDTH^2 */
#define DRPAI_IN_WIDTH              (CAM_IMAGE_WIDTH)
#define DRPAI_IN_HEIGHT             (CAM_IMAGE_WIDTH) 
#define DRPAI_IN_CHANNEL_YUY2       (CAM_IMAGE_CHANNEL_YUY2)
#else
/** DRP-AI input is assigned to the buffer having the size of camera image. */
#define DRPAI_IN_WIDTH              (CAM_IMAGE_WIDTH)
#define DRPAI_IN_HEIGHT             (CAM_IMAGE_HEIGHT)  
#define DRPAI_IN_CHANNEL_YUY2       (CAM_IMAGE_CHANNEL_YUY2)
#endif

/** Stereo Parameter **/
#define STEREO_CALC_MODE            "oca"

/*Wayland:: Wayland Information */
#ifdef IMAGE_OUTPUT_HD
#define IMAGE_OUTPUT_WIDTH          (1280)
#define IMAGE_OUTPUT_HEIGHT         (720)
#else /* IMAGE_OUTPUT_FHD */
#define IMAGE_OUTPUT_WIDTH          (1920)
#define IMAGE_OUTPUT_HEIGHT         (1080)
#endif

/*Camera image size displayed on HDMI image.*/
#ifdef CAM_INPUT_VGA
#define CAM_RESIZED_WIDTH        (CAM_IMAGE_WIDTH*2)
#define CAM_RESIZED_HEIGHT       (CAM_IMAGE_HEIGHT*2)
#else
  #ifdef CAM_INPUT_HD
  #define CAM_RESIZED_WIDTH        (CAM_IMAGE_WIDTH)
  #define CAM_RESIZED_HEIGHT       (CAM_IMAGE_HEIGHT)
  #else /* CAM_INPUT_FHD */
  #define CAM_RESIZED_WIDTH        (IMAGE_OUTPUT_WIDTH)
  #define CAM_RESIZED_HEIGHT       (IMAGE_OUTPUT_HEIGHT)
#endif
#endif

#define IMAGE_CHANNEL_BGRA          (4)
#define WL_BUF_NUM                  (2)

/*Image:: Text information to be drawn on image*/
#define CHAR_SCALE_LARGE            (0.8)
#define CHAR_SCALE_SMALL            (0.7)
#define CHAR_THICKNESS              (2)
#define LINE_HEIGHT                 (30) /*in pixel*/
#define LINE_HEIGHT_OFFSET          (20) /*in pixel*/
#define TEXT_WIDTH_OFFSET           (10) /*in pixel*/
#ifdef CAM_INPUT_VGA
//#if defined(CAM_INPUT_VGA)
#define CHAR_THICKNESS_BOX          (1)  /*in pixel*/
#define BOX_LINE_SIZE               (2)  /*in pixel*/
#define BOX_HEIGHT_OFFSET           (15) /*in pixel*/
#define BOX_TEXT_HEIGHT_OFFSET      (5)  /*in pixel*/
#define CHAR_SCALE_FONT             (0.5)
#else
#define CHAR_THICKNESS_BOX          (2)  /*in pixel*/
#define BOX_LINE_SIZE               (3)  /*in pixel*/
#define BOX_HEIGHT_OFFSET           (30) /*in pixel*/
#define BOX_TEXT_HEIGHT_OFFSET      (8)  /*in pixel*/
#define CHAR_SCALE_FONT             (0.8)
#endif
#define WHITE_DATA                  (0xFFFFFFu) /* in RGB */
#define BLACK_DATA                  (0x000000u)

/*Waiting Time*/
#define WAIT_TIME                   (1000) /* microseconds */

/*Timer Related*/
#define CAPTURE_TIMEOUT             (20)  /* seconds */
#define AI_THREAD_TIMEOUT           (20)  /* seconds */
#define STEREO_THREAD_TIMEOUT       (20)  /* seconds */
#define DISPLAY_THREAD_TIMEOUT      (20)  /* seconds */
#define KEY_THREAD_TIMEOUT          (5)   /* seconds */
#define TIME_COEF                   (1)

/*Array size*/
#define SIZE_OF_ARRAY(array)        (sizeof(array)/sizeof(array[0]))

/*****************************************
* For image.cpp
******************************************/
/*For drawing the bounding box label on image*/
#define FONTDATA_WIDTH              (6)
#define FONTDATA_HEIGHT             (8)

/*****************************************
* For OpenCVA
******************************************/
/* OpenCVA Activate */
#define OPENCVA_FUNC_DISABLE (0)
#define OPENCVA_FUNC_ENABLE (1)
#define OPENCVA_FUNC_NOCHANGE (2)
// #define OPENCVA_ALL_ENABLE

#endif
