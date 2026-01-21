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
 * File Name    : define.h
 * Version      : v3.10
 * Description  : RZ/V2H AI SDK Sample Application for StereoSGM
 ***********************************************************************************************************************/

#ifndef DEFINE_MACRO_H
#define DEFINE_MACRO_H

/*Uncomment to display the camera framerate on application window. */
// #define DISP_CAM_FRAME_RATE
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
#include <iomanip>
#include <cstring>
#include <float.h>
#include <atomic>
#include <semaphore.h>
#include <math.h>
#include <fstream>
#include <sys/time.h>
#include <climits>
#include <numeric>
#include <algorithm>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

/*****************************************
 * Macro for Application
 ******************************************/

/*Camera Capture Image Information*/
#define CAPTURE_STABLE_COUNT (8)

/*Timer Related*/
#define CAPTURE_TIMEOUT (20)        /* seconds */
#define STEREO_TIMEOUT (20)         /* seconds */
#define IMAGE_THREAD_TIMEOUT (20)   /* seconds */
#define DISPLAY_THREAD_TIMEOUT (20) /* seconds */
#define KEY_THREAD_TIMEOUT (5)      /* seconds */

/*Waiting Time*/
#define WAIT_TIME (1000) /* microseconds */

/* OpenCVA Activate */
#define OPENCVA_FUNC_DISABLE (0)
#define OPENCVA_FUNC_ENABLE (1)
#define OPENCVA_FUNC_NOCHANGE (2)
// #define OPENCVA_ALL_ENABLE

#define STEREO_GAMMA_PATH "stereo_gamma_v1.0.csv"

#define AVG_TIME_CNT (30)

#define APP_SUCCESS (0)
#define APP_ERROR (-1)

#endif
