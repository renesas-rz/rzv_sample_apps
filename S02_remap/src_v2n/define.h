/***********************************************************************************************************************
* Copyright (C) 2024-2025 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name    : define.h
* Version      : 1.00
* Description  : RZ/V2N DRP-AI Sample Application for Remap with MIPI Camera
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
#include <signal.h>
#include <vector>
#include <map>
#include <fstream>
#include <errno.h>
#include <math.h>
#include <iomanip>
#include <atomic>
#include <semaphore.h>
#include <cstring>
#include <numeric>

/*****************************************
* Macro for TopFormer
******************************************/
/* Input Camera support */
/* n = 0: USB Camera, n = 1: eCAM22 */
#define INPUT_CAM_TYPE 1

/* Output Camera Size */
#define CAM_INPUT_VGA
#define IMAGE_OUTPUT_FHD

/*Time Measurement Flag*/
//#define DEBUG_TIME_FLG

/* Enable demonstration mode for combination with GUI Demo system */
#define END_DET_TYPE                (0)

/*Display AI frame rate*/
#undef DISP_AI_FRAME_RATE

/*****************************************
* Macro for Application
******************************************/
/* DRP_MAX_FREQ and DRPAI_FREQ are the   */
/* frequency settings for DRP-AI.        */
/* Basically use the default values      */

#define DRP_MAX_FREQ                (2)
/* DRP_MAX_FREQ can be set from 2 to 127 */
/* 2: 420MHz                             */
/* 3: 315MHz                             */
/* ...                                   */
/* 127: 9.84MHz                          */
/* Calculation Formula:                  */
/*     1260MHz /(DRP_MAX_FREQ + 1)       */

#define DRPAI_FREQ                  (2)
/* DRPAI_FREQ can be set from 1 to 127   */
/* 1,2: 1GHz                             */
/* 3: 630MHz                             */
/* 4: 420MHz                             */
/* 5: 315MHz                             */
/* ...                                   */
/* 127: 10MHz                            */
/* Calculation Formula:                  */
/*     1260MHz /(DRPAI_FREQ - 1)         */
/*     (When DRPAI_FREQ = 3 or more.)    */

/*Camera:: Capture Image Information*/
#ifdef CAM_INPUT_VGA
#define CAM_IMAGE_WIDTH             (640)
#define CAM_IMAGE_HEIGHT            (480)
#define MIPI_CAM_RES "640x480"
#else /* CAM_INPUT_FHD */
#define CAM_IMAGE_WIDTH             (1920)
#define CAM_IMAGE_HEIGHT            (1080)
#define MIPI_CAM_RES "1920x1080"
#endif

#define CAM_IMAGE_CHANNEL_YUY2      (2)

/*Camera:: Capture Information */
#if INPUT_CAM_TYPE == 1
#define CAP_BUF_NUM                 (6)
#define INPUT_CAM_NAME              "MIPI Camera"
#else /* INPUT_CAM_TYPE */
#define CAP_BUF_NUM                 (3)
#define INPUT_CAM_NAME              "USB Camera"
#endif /* INPUT_CAM_TYPE */

/*Wayland:: Wayland Information */
#ifdef IMAGE_OUTPUT_HD
#define IMAGE_OUTPUT_WIDTH          (1280)
#define IMAGE_OUTPUT_HEIGHT         (720)
#else /* IMAGE_OUTPUT_FHD */
#define IMAGE_OUTPUT_WIDTH          (1920)
#define IMAGE_OUTPUT_HEIGHT         (1080)
#endif

/*Camera image size displayed on HDMI image. */
#if defined(CAM_INPUT_VGA) && defined(IMAGE_OUTPUT_HD)     
#define CAM_RESIZED_WIDTH        (CAM_IMAGE_WIDTH)
#define CAM_RESIZED_HEIGHT       (CAM_IMAGE_HEIGHT)
#define CAM_RESIZED_PADDING      (1)
#elif defined(CAM_INPUT_VGA) // && IMAGE_OUTPUT_FHD
#define CAM_RESIZED_WIDTH        (CAM_IMAGE_WIDTH*1.5)
#define CAM_RESIZED_HEIGHT       (CAM_IMAGE_HEIGHT*1.5)
#define CAM_RESIZED_PADDING      (1)
#else // CAM_INPUT_FHD && (IMAGE_OUTPUT_FHD || IMAGE_OUTPUT_HD)
#define CAM_RESIZED_WIDTH        (IMAGE_OUTPUT_WIDTH/2.0)
#define CAM_RESIZED_HEIGHT       (IMAGE_OUTPUT_HEIGHT/1.5)
#define CAM_RESIZED_PADDING      (1)
#endif

#define IMAGE_CHANNEL_BGRA          (4)
#define WL_BUF_NUM                  (2)

/*Image:: Text information to be drawn on image*/
#define CHAR_SCALE_LARGE            (0.8)
#define CHAR_SCALE_SMALL            (0.6)
#define CHAR_THICKNESS              (2)
#define LINE_HEIGHT                 (30) /*in pixel*/
#define LINE_HEIGHT_OFFSET          (20) /*in pixel*/
#define TEXT_WIDTH_OFFSET           (10) /*in pixel*/
#define YELLOW_DATA                 (0xFF00FFu) /* in RGB */
#define BLUE_DATA                   (0x00FF00u) /* in RGB */
#define RED_DATA                    (0x0000FFu) /* in RGB */
#define WHITE_DATA                  (0xFFFFFFu) /* in RGB */
#define BLACK_DATA                  (0x000000u)

/*Waiting Time*/
#define WAIT_TIME                   (1000) /* microseconds */

/*Timer Related*/
#define CAPTURE_TIMEOUT             (20)  /* seconds */
#define AI_THREAD_TIMEOUT           (20)  /* seconds */
#define DISPLAY_THREAD_TIMEOUT      (20)  /* seconds */
#define KEY_THREAD_TIMEOUT          (5)   /* seconds */
#define TIME_COEF                   (1)

/*Buffer size for writing data to memory via DRP-AI Driver.*/
#define BUF_SIZE                    (1024)

/*Array size*/
#define SIZE_OF_ARRAY(array) (sizeof(array)/sizeof(array[0]))

/*Calibration Infomation*/
#define  CALIBRATION_COUNT_MAX            (25)

/*Display remap processing time for each this number of threads*/
#define  REMAP_TIME_THREAD_COUNT_MAX      (10)

/*Path to the output correction file */
#define CAMERA_FILE_PATH                "./camera.csv"
#define DIST_FILE_PATH                  "./dist.csv"

/*The camera calibration mat size */
#define CAMERA_FILE_MAT_WIDTH           (3) 
#define CAMERA_FILE_MAT_HEIGHT          (3)
#define DIST_FILE_MAT_WIDTH             (5)
#define DIST_FILE_MAT_HEIGHT            (1)

/*Chessboard Infomation*/
#define  SQUARE_SIZE                (23.0)     /* The size of one square of the chessboard (mm) */
#define  BOARD_WIDTH                (10)
#define  BOARD_HEIGHT               (7)

#define DRP_FUNC_NUM            (17)

#endif
