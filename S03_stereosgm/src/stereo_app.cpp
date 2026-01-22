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
 * File Name    : stereo_app.cpp
 * Version      : v3.20
 * Description  : RZ/V2H AI SDK Sample Application for StereoSGM
 ***********************************************************************************************************************/

/*****************************************
 * Includes
 ******************************************/
/*Definition of Macros & other variables*/
#include "define.h"
#include "capture.h"
#include "image.h"
#include "display.h"
#include "mode.h"

/*****************************************
 * Global Variables
 ******************************************/
/*Multithreading*/
static sem_t terminate_req_sem;
static pthread_t stereo_thread;
static pthread_t kbhit_thread;
static pthread_t capture_thread;
static pthread_t imageL_thread;
static pthread_t imageR_thread;
static pthread_t display_thread;
static std::mutex mtx;

/*Flags*/
static std::atomic<uint8_t> stereo_start(0);
static std::atomic<uint8_t> stereoL_start(0);
static std::atomic<uint8_t> stereoR_start(0);
static std::atomic<uint8_t> imageL_start(0);
static std::atomic<uint8_t> imageR_start(0);
static std::atomic<uint8_t> display_start(0);

static Capture capL;
static Capture capR;
static Image imgL;
static Image imgR;
static Display disp;
static Mode md;

unsigned long OCA_list[OCA_LIST_NUM];

/*GStreamer pipeline for camera capture*/
static std::string gstreamer_pipeline = "";

/*****************************************
 * Function Name : timedifference_msec
 * Description   : compute the time differences in ms between two moments
 * Arguments     : t0 = start time
 *                 t1 = stop time
 * Return value  : the time difference in ms
 ******************************************/
static double timedifference_msec(struct timespec t0, struct timespec t1)
{
    return (t1.tv_sec - t0.tv_sec) * 1000.0 + (t1.tv_nsec - t0.tv_nsec) / 1000000.0;
}

/*****************************************
 * Function Name : wait_join
 * Description   : waits for a fixed amount of time for the thread to exit
 * Arguments     : p_join_thread = thread that the function waits for to Exit
 *                 join_time = the timeout time for the thread for exiting
 * Return value  : 0 if successful
 *                 not 0 otherwise
 ******************************************/
static int8_t wait_join(pthread_t *p_join_thread, uint32_t join_time)
{
    int8_t ret_err;
    struct timespec join_timeout;
    ret_err = clock_gettime(CLOCK_REALTIME, &join_timeout);
    if (0 == ret_err)
    {
        join_timeout.tv_sec += join_time;
        ret_err = pthread_timedjoin_np(*p_join_thread, NULL, &join_timeout);
    }
    return ret_err;
}

/*****************************************
 * Function Name : R_Kbhit_Thread
 * Description   : Executes the Keyboard hit thread (checks if enter key is hit)
 * Arguments     : threadid = thread identification
 * Return value  : -
 ******************************************/
void *R_Kbhit_Thread(void *threadid)
{
    printf("Key Hit Thread Starting\n");

    /*Semaphore Variable*/
    int32_t kh_sem_check = 0;
    /*Variable to store the getchar() value*/
    int32_t c = 0;
    /*Variable for checking return value*/
    int8_t ret = 0;

    printf("************************************************\n");
    printf("* Press ENTER key to quit. *\n");
    printf("************************************************\n");

    /*Set Standard Input to Non Blocking*/
    errno = 0;
    ret = fcntl(0, F_SETFL, O_NONBLOCK);
    if (-1 == ret)
    {
        fprintf(stderr, "[ERROR] Failed to run fctnl(): errno=%d\n", errno);
        goto err;
    }

    while (1)
    {
        /*Gets the Termination request semaphore value. If different then 1 Termination was requested*/
        /*Checks if sem_getvalue is executed wihtout issue*/
        errno = 0;
        ret = sem_getvalue(&terminate_req_sem, &kh_sem_check);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to get Semaphore Value: errno=%d\n", errno);
            goto err;
        }
        /*Checks the semaphore value*/
        if (1 != kh_sem_check)
        {
            goto key_hit_end;
        }

        c = getchar();
        if (EOF != c)
        {
            if(c == 'w')
            {
                imgL.ty = imgL.ty - 0.5;
                std::cout << "up : " << imgL.ty << std::endl;
                c = getchar();
                continue;
            } else if (c == 'x') {
                imgL.ty = imgL.ty + 0.5;
                std::cout << "down" << imgL.ty << std::endl;
                c = getchar();
                continue;
            } else if (c == 'a') {
                imgL.angle = imgL.angle + 0.02;
                std::cout << "left" << imgL.angle << std::endl;
                c = getchar();
                continue;
            } else if (c == 's') {
                std::cout << "right" << imgL.angle << std::endl;
                imgL.angle = imgL.angle - 0.02;
                c = getchar();
                continue;
            }
          
            /* When key is pressed. */
            printf("[INFO] Key Detected.\n");
            goto err;
        }
        else
        {
            /* When nothing is pressed. */
            usleep(WAIT_TIME);
        }
    }

/*Error Processing*/
err:
    /*Set Termination Request Semaphore to 0*/
    sem_trywait(&terminate_req_sem);
    goto key_hit_end;

key_hit_end:
    printf("Key Hit Thread Terminated\n");
    pthread_exit(NULL);
}

/*****************************************
 * Function Name : R_Display_Thread
 * Description   : Executes the HDMI Display with Display thread
 * Arguments     : threadid = thread identification
 * Return value  : -
 ******************************************/
void *R_Display_Thread(void *threadid)
{
    printf("Display Thread Starting\n");

    timespec start_time;
    timespec end_time;

    /*Semaphore Variable*/
    int32_t display_sem_check = 0;
    /*Variable for checking return value*/
    int8_t ret = 0;

    disp.init(md.sz.in_width, md.sz.in_height, md.sz.in_ch,
              md.sz.cal_width, md.sz.cal_height, md.sz.cal_ch, md.pr.disparity, md.pr.heat_map);

    std::string para_siri = "PARA";
    std::string graph_disparity = "disparity_h, " + md.pr.mode
                                + ", rm: " + std::to_string(md.pr.remap)
                                + ", gm: " + std::to_string(md.pr.gamma)
                                + ", sp: " + std::to_string(md.pr.sharp)
                                + ", ps: " + para_siri;

    while (1)
    {
        /*Gets The Termination Request Semaphore Value, If Different Then 1 Termination Is Requested*/
        /*Checks If sem_getvalue Is Executed Without Issue*/
        errno = 0;
        ret = sem_getvalue(&terminate_req_sem, &display_sem_check);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to get Semaphore Value: errno=%d\n", errno);
            goto err;
        }
        /*Checks the semaphore value*/
        if (1 != display_sem_check)
        {
            goto display_end;
        }
        /* Check display_start flag which is set in Capture Thread. */
        if (display_start.load())
        {
            timespec_get(&start_time, TIME_UTC);

            disp.post_process();
            //disp.plot_time();
            disp.plot_time_mat();
            // disp.print_time();

            disp.show_image("src1_gray", "src1_gray");
            // disp.show_image("src2_gray", "src2_gray");
            // disp.show_image("gamma",     "gamma");
            disp.show_image(graph_disparity, "disparity_h");
            disp.show_image("time_mat", "time_mat");

            cv::waitKey(1);

            display_start.store(0);

            timespec_get(&end_time, TIME_UTC);
            disp.set_display_time(timedifference_msec(start_time, end_time));
        }

        usleep(WAIT_TIME); // wait 1 tick timedg
    } /*End Of Loop*/

/*Error Processing*/
err:
    /*Set Termination Request Semaphore To 0*/
    sem_trywait(&terminate_req_sem);
    goto display_end;

display_end:
    /*To terminate the loop in Capture Thread.*/
    display_start.store(0);
    printf("Display Thread Terminated\n");
    pthread_exit(NULL);
}

/*****************************************
 * Function Name : R_Stereo_Thread
 * Description   : Executes the DRP-AI stereo thread
 * Arguments     : threadid = thread identification
 * Return value  : -
 ******************************************/
void *R_Stereo_Thread(void *threadid)
{
    printf("Stereo Thread Starting\n");

    /*Variable for Performance Measurement*/
    timespec start_time;
    timespec end_time;

    /*Semaphore Variable*/
    int32_t stereo_sem_check = 0;

    /*Variable for checking return value*/
    int8_t ret = 0;

    /*Stereo Variables*/
    fd_set rfds;
    struct timespec tv;
    int8_t inf_status = 0;

    int sgbm_mode[5] = {cv::StereoSGBM::MODE_SGBM, cv::StereoSGBM::MODE_HH, cv::StereoSGBM::MODE_SGBM_3WAY,
                        cv::StereoSGBM::MODE_HH4, cv::StereoSGBM::MODE_SGM_DRP};
    int minDisparity = 0;
    int numDisparities = md.pr.disparity;
    int blockSize = md.pr.win_x;
    int P1 = md.pr.p1;
    int P2 = md.pr.p2;
    int disp12MaxDiff = 0;
    int preFilterCap = 0;
    int uniquenessRatio = md.pr.uniquenessRatio;
    int speckleWindowSize = 0;
    int speckleRange = 0;
    int mode = (md.pr.mode == "oca") ? sgbm_mode[4] : sgbm_mode[0];

    cv::Mat src1_gray(cv::Size(md.sz.cal_width, md.sz.cal_height), CV_8UC1);
    cv::Mat src2_gray(cv::Size(md.sz.cal_width, md.sz.cal_height), CV_8UC1);
    cv::Mat stereo_out(cv::Size(md.sz.cal_width, md.sz.cal_height), CV_16SC1); /* disparity */

    cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(minDisparity, numDisparities, blockSize, P1, P2,
                                                            disp12MaxDiff, preFilterCap, uniquenessRatio, speckleWindowSize, speckleRange, mode);

    if (md.pr.mode == "oca")
    {
        OCA_list[OCA_FUNC_STEREOSGM] = OPENCVA_FUNC_ENABLE;
        OCA_Activate(&OCA_list[0]);
    }

    while (1)
    {
        /*Gets the Termination request semaphore value. If different then 1 Termination was requested*/
        /*Checks if sem_getvalue is executed wihtout issue*/
        errno = 0;
        ret = sem_getvalue(&terminate_req_sem, &stereo_sem_check);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to get Semaphore Value: errno=%d\n", errno);
            goto err;
        }
        /*Checks the semaphore value*/
        if (1 != stereo_sem_check)
        {
            goto stereo_end;
        }
        /*Checks if image frame from Capture Thread is start.*/
        if (stereo_start.load())
        {
            timespec_get(&start_time, TIME_UTC);

            src1_gray = imgL.get_calc();
            src2_gray = imgR.get_calc();
            timespec_get(&start_time, TIME_UTC);
            stereo->compute(src1_gray, src2_gray, stereo_out);
            timespec_get(&end_time, TIME_UTC);
            
            disp.set_src1_gray(src1_gray);
            disp.set_stereo_out(stereo_out);

            stereo_start.store(0);

            timespec_get(&end_time, TIME_UTC);
            disp.set_stereo_time(timedifference_msec(start_time, end_time));
        }

        usleep(WAIT_TIME);
    }

    /*End of Stereo Loop*/

/*Error Processing*/
err:
    /*Set Termination Request Semaphore to 0*/
    sem_trywait(&terminate_req_sem);
    goto stereo_end;
/*Stereo Thread Termination*/
stereo_end:
    /*To terminate the loop in Capture Thread.*/
    printf("Stereo Thread Terminated\n");
    pthread_exit(NULL);
}

/*****************************************
 * Function Name : R_ImageL_Thread
 * Description   : Executes img proc with img thread
 * Arguments     : threadid = thread identification
 * Return value  : -
 ******************************************/
void *R_ImageL_Thread(void *threadid)
{
    printf("ImageL Thread Starting\n");

    timespec start_time;
    timespec end_time;

    /*Semaphore Variable*/
    int32_t imageL_sem_check = 0;
    /*Variable for checking return value*/
    int8_t ret = 0;

    imgL.init(md.pr.image, md.pr.format,
              md.sz.in_width, md.sz.in_height, md.sz.in_ch,
              md.sz.cal_width, md.sz.cal_height, md.sz.cal_ch,
              md.sz.trim_x, md.sz.trim_y, md.pr.remap, md.pr.gamma, md.pr.sharp);
    
    if (md.pr.remap == 0)
    {
        // not remap
    } else if (md.pr.remap == 1)
    {
        imgL.init_undistort_map("left");
    } else if (md.pr.remap == 2)
    {
        imgL.init_undistort_map("left");
    } else
    {
        imgL.init_undistort_map("right");
    }

    if (md.pr.gamma != 0)
        imgL.init_gamma();

    while (1)
    {
        /*Gets The Termination Request Semaphore Value, If Different Then 1 Termination Is Requested*/
        /*Checks If sem_getvalue Is Executed Without Issue*/
        errno = 0;
        ret = sem_getvalue(&terminate_req_sem, &imageL_sem_check);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to get Semaphore Value: errno=%d\n", errno);
            goto err;
        }
        /*Checks the semaphore value*/
        if (1 != imageL_sem_check)
        {
            goto imageL_end;
        }
        /* Check img_obj_start flag which is set in Capture Thread. */
        if (imageL_start.load())
        {
            timespec_get(&start_time, TIME_UTC);

            imgL.pre_process();

            imageL_start.store(0);

            timespec_get(&end_time, TIME_UTC);
            disp.set_imageL_time(timedifference_msec(start_time, end_time));
        }

        usleep(WAIT_TIME); // wait 1 tick time
    } /*End Of Loop*/

/*Error Processing*/
err:
    /*Set Termination Request Semaphore To 0*/
    sem_trywait(&terminate_req_sem);
    goto imageL_end;

imageL_end:
    /*To terminate the loop in Capture Thread.*/
    imageL_start.store(0);
    printf("ImageL Thread Terminated\n");
    pthread_exit(NULL);
}

/*****************************************
 * Function Name : R_ImageL_Thread
 * Description   : Executes img proc with img thread
 * Arguments     : threadid = thread identification
 * Return value  : -
 ******************************************/
void *R_ImageR_Thread(void *threadid)
{
    printf("ImageR Thread Starting\n");

    timespec start_time;
    timespec end_time;

    /*Semaphore Variable*/
    int32_t imageR_sem_check = 0;
    /*Variable for checking return value*/
    int8_t ret = 0;

    imgR.init(md.pr.image, md.pr.format,
              md.sz.in_width, md.sz.in_height, md.sz.in_ch,
              md.sz.cal_width, md.sz.cal_height, md.sz.cal_ch,
              md.sz.trim_x, md.sz.trim_y, md.pr.remap, md.pr.gamma, md.pr.sharp);

    if (md.pr.remap == 0)
    {
        // not remap
    } else if (md.pr.remap == 1)
    {
        imgR.init_undistort_map("right");
    } else if (md.pr.remap == 2)
    {
        imgR.init_undistort_map("left");
    } else
    {
        imgR.init_undistort_map("right");
    }

    if (md.pr.gamma != 0)
        imgR.init_gamma();

    while (1)
    {
        /*Gets The Termination Request Semaphore Value, If Different Then 1 Termination Is Requested*/
        /*Checks If sem_getvalue Is Executed Without Issue*/
        errno = 0;
        ret = sem_getvalue(&terminate_req_sem, &imageR_sem_check);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to get Semaphore Value: errno=%d\n", errno);
            goto err;
        }
        /*Checks the semaphore value*/
        if (1 != imageR_sem_check)
        {
            goto imageR_end;
        }
        /* Check img_obj_start flag which is set in Capture Thread. */
        if (imageR_start.load())
        {
            timespec_get(&start_time, TIME_UTC);

            imgR.pre_process();

            imageR_start.store(0);

            timespec_get(&end_time, TIME_UTC);
            disp.set_imageR_time(timedifference_msec(start_time, end_time));
        }

        usleep(WAIT_TIME); // wait 1 tick time
    } /*End Of Loop*/

/*Error Processing*/
err:
    /*Set Termination Request Semaphore To 0*/
    sem_trywait(&terminate_req_sem);
    goto imageR_end;

imageR_end:
    /*To terminate the loop in Capture Thread.*/
    imageR_start.store(0);
    printf("ImageR Thread Terminated\n");
    pthread_exit(NULL);
}

/*****************************************
 * Function Name : R_Capture_Thread
 * Description   : Executes the V4L2 capture with Capture thread.
 * Arguments     : threadid = thread identification
 * Return value  : -
 ******************************************/
void *R_Capture_Thread(void *threadid)
{
    printf("Capture Thread Starting\n");

    timespec start_time;
    timespec end_time;

    timespec start_fr_time;
    timespec end_fr_time;

    /*Semaphore Variable*/
    int32_t capture_sem_check = 0;
    int8_t ret = 0;
    /* Counter to wait for the camera to stabilize */
    uint8_t capture_stabe_cnt = CAPTURE_STABLE_COUNT;

    cv::Mat frameL;
    cv::Mat frameR;
    cv::Mat frameS;

    std::ifstream fileL(md.pr.left + ".txt");
    std::ifstream fileR(md.pr.right + ".txt");
    std::string lineL;
    std::string lineR;

    cv::VideoCapture v_capL;
    cv::VideoCapture v_capR;
    std::string gstreamL;
    std::string gstreamR;

    std::string media_ctlL[6];
    std::string media_ctlR[6];

    /*Initialize Image object.*/
    capL.init("left", md.pr.image, md.pr.left, md.pr.format, md.pr.fps, md.sz.in_width, md.sz.in_height, md.sz.in_ch);
    capR.init("right", md.pr.image, md.pr.right, md.pr.format, md.pr.fps, md.sz.in_width, md.sz.in_height, md.sz.in_ch);

    if ((md.pr.image == "ec22") || (md.pr.image == "ec25"))
    {
        for (int i = 0; i < 6; i++)
        {
            media_ctlL[i] = capL.get_media_ctl(i);
            media_ctlR[i] = capR.get_media_ctl(i);
        }

        ret = system(media_ctlL[0].c_str());
        ret = system(media_ctlL[1].c_str());
        ret = system(media_ctlL[2].c_str());
        ret = system(media_ctlL[3].c_str());
        ret = system(media_ctlL[4].c_str());
        ret = system(media_ctlL[5].c_str());

        ret = system(media_ctlR[0].c_str());
        ret = system(media_ctlR[1].c_str());
        ret = system(media_ctlR[2].c_str());
        ret = system(media_ctlR[3].c_str());
        ret = system(media_ctlR[4].c_str());
        ret = system(media_ctlR[5].c_str());

        std::cout << "----- e-CAM Camaera Left -----" << std::endl;
        std::cout << "media_ctlL[0] : " << media_ctlL[0] << std::endl;
        std::cout << "media_ctlL[1] : " << media_ctlL[1] << std::endl;
        std::cout << "media_ctlL[2] : " << media_ctlL[2] << std::endl;
        std::cout << "media_ctlL[3] : " << media_ctlL[3] << std::endl;
        std::cout << "media_ctlL[4] : " << media_ctlL[4] << std::endl;
        std::cout << "media_ctlL[5] : " << media_ctlL[5] << std::endl;
        std::cout << "----- e-CAM Camaera Right -----" << std::endl;
        std::cout << "media_ctlR[0] : " << media_ctlR[0] << std::endl;
        std::cout << "media_ctlR[1] : " << media_ctlR[1] << std::endl;
        std::cout << "media_ctlR[2] : " << media_ctlR[2] << std::endl;
        std::cout << "media_ctlR[3] : " << media_ctlR[3] << std::endl;
        std::cout << "media_ctlR[4] : " << media_ctlR[4] << std::endl;
        std::cout << "media_ctlR[5] : " << media_ctlR[5] << std::endl;
    }

    if ((md.pr.image == "usb") || (md.pr.image == "ec22") || (md.pr.image == "ec25"))
    {
        gstreamL = capL.get_gstreamer();
        gstreamR = capR.get_gstreamer();
        std::cout << "-------------------------------" << std::endl;
        std::cout << "[INFO] GStreamerL pipeline: " << gstreamL.c_str() << std::endl;
        std::cout << "[INFO] GStreamerR pipeline: " << gstreamR.c_str() << std::endl;
        std::cout << "-------------------------------" << std::endl;

        v_capL.open(gstreamL, cv::CAP_GSTREAMER);
        v_capR.open(gstreamR, cv::CAP_GSTREAMER);
        // v_capL.open(0);
        // v_capR.open(2);
        if (!v_capL.isOpened() || !v_capR.isOpened())
        {
            fprintf(stderr, "[ERROR] Failed to open camera.\n");
            goto err;
        }
    }
    else if (md.pr.image == "stereo")
    {
        gstreamL = capL.get_gstreamer();
        std::cout << "[INFO] GStreamerL pipeline: " << gstreamL.c_str() << std::endl;
        v_capL.open(gstreamL, cv::CAP_GSTREAMER);
        if (!v_capL.isOpened())
        {
            fprintf(stderr, "[ERROR] Failed to open camera.\n");
            goto err;
        }
    }
    else if (md.pr.image == "data")
    {
        printf("Input File mode \n");
    }
    else
    {
        fprintf(stderr, "[ERROR] Failed to open camera.\n");
        goto err;
    }

    timespec_get(&start_fr_time, TIME_UTC); // initial
    while (1)
    {
        timespec_get(&start_time, TIME_UTC);

        /*Gets the Termination request semaphore value. If different then 1 Termination was requested*/
        /*Checks if sem_getvalue is executed wihtout issue*/
        errno = 0;
        ret = sem_getvalue(&terminate_req_sem, &capture_sem_check);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to get Semaphore Value: errno=%d\n", errno);
            goto err;
        }
        /*Checks the semaphore value*/
        if (1 != capture_sem_check)
        {
            goto capture_end;
        }

        /* Capture camera image */
        if ((md.pr.image == "usb") || (md.pr.image == "ec22") || (md.pr.image == "ec25"))
        {
            v_capL.read(frameL);
            v_capR.read(frameR);
            /* Breaking the loop if no video frame is detected */
            if (frameL.empty() || frameR.empty())
            {
                fprintf(stderr, "[ERROR] Failed to get capture image.\n");
                goto err;
            }
        }
        else if (md.pr.image == "stereo")
        {
            v_capL.read(frameS);
            /* Breaking the loop if no video frame is detected */
            if (frameS.empty())
            {
                fprintf(stderr, "[ERROR] Failed to get capture image.\n");
                goto err;
            }
            frameL = cv::Mat(frameS, cv::Rect(0, 0, md.sz.in_width, md.sz.in_height));
            frameR = cv::Mat(frameS, cv::Rect(md.sz.in_width, 0, md.sz.in_width, md.sz.in_height));
        }
        else if (md.pr.image == "data")
        {
            // printf("Input File mode \n");
        }
        else
        {
            fprintf(stderr, "[ERROR] Failed to get capture image.\n");
            goto err;
        }

        if (!imageL_start.load() && !imageR_start.load() && !stereo_start.load() && !display_start.load())
        {
            timespec_get(&end_fr_time, TIME_UTC);
            disp.set_frame_time(timedifference_msec(start_fr_time, end_fr_time));
            timespec_get(&start_fr_time, TIME_UTC);

            if (capture_stabe_cnt == 0)
            {
                stereo_start.store(1);
                display_start.store(1);
            }
            else if (capture_stabe_cnt == 1)
            {
                stereo_start.store(1);
                capture_stabe_cnt--;
            }
            else
            {
                capture_stabe_cnt--;
            }

            /* Read dataset image */
            if (md.pr.image == "data")
            {
                if (std::getline(fileL, lineL) && std::getline(fileR, lineR))
                {
                    frameL = cv::imread(md.pr.left + "/" + lineL);
                    frameR = cv::imread(md.pr.right + "/" + lineR);
                }
                else
                {
                    printf("file data finished \n");
                    break;
                }
            }

            /* Copy captured image to Image object. This will be used in Main Thread. */
            imgL.set_mat(frameL);
            imgR.set_mat(frameR);

            imageL_start.store(1);
            imageR_start.store(1);
        }

        timespec_get(&end_time, TIME_UTC);
        disp.set_capture_time(timedifference_msec(start_time, end_time));

    } /*End of Loop*/

/*Error Processing*/
err:
    sem_trywait(&terminate_req_sem);
    goto capture_end;

capture_end:
    v_capL.release();
    v_capR.release();
    /*To terminate the loop in AI Stereo Thread.*/
    stereo_start.store(1);

    printf("Capture Thread Terminated\n");
    pthread_exit(NULL);
}

/*****************************************
 * Function Name : R_Main_Process
 * Description   : Runs the main process loop
 * Arguments     : -
 * Return value  : 0 if succeeded
 *                 not 0 otherwise
 ******************************************/
int8_t R_Main_Process()
{
    printf("Main Loop Starts\n");

    /*Main Process Variables*/
    int8_t main_ret = 0;
    /*Semaphore Related*/
    int32_t sem_check = 0;
    /*Variable for checking return value*/
    int8_t ret = 0;

    while (1)
    {
        /*Gets the Termination request semaphore value. If different then 1 Termination was requested*/
        errno = 0;
        ret = sem_getvalue(&terminate_req_sem, &sem_check);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to get Semaphore Value: errno=%d\n", errno);
            goto err;
        }
        /*Checks the semaphore value*/
        if (1 != sem_check)
        {
            goto main_proc_end;
        }

        /*Wait for 1 TICK.*/
        usleep(WAIT_TIME);
    }

/*Error Processing*/
err:
    sem_trywait(&terminate_req_sem);
    main_ret = 1;
    goto main_proc_end;
/*Main Processing Termination*/
main_proc_end:
    printf("Main Process Terminated\n");
    return main_ret;
}

/*****************************************
 * Function Name : main
 ******************************************/
int32_t main(int32_t argc, char *argv[])
{
    int8_t main_proc = 0;
    int8_t ret = 0;
    int8_t ret_main = 0;
    uint item_num;

    /*Multithreading Variables*/
    int32_t create_thread_capture = -1;
    int32_t create_thread_imageL = -1;
    int32_t create_thread_imageR = -1;
    int32_t create_thread_stereo = -1;
    int32_t create_thread_display = -1;
    int32_t create_thread_key = -1;
    int32_t sem_create = -1;

    /*Disable OpenCV Accelerator due to the use of multithreading */
#ifdef OPENCVA_ALL_ENABLE
    for (int i = 0; i < OCA_LIST_NUM; i++)
        OCA_list[i] = OPENCVA_FUNC_ENABLE;
#else
    for (int i = 0; i < OCA_LIST_NUM; i++)
        OCA_list[i] = OPENCVA_FUNC_DISABLE;
#endif
    OCA_Activate(&OCA_list[0]);

    /*Setting parameter of application */
    std::cout << "************************************************" << std::endl;
    if (argc == 2)
    {
        std::cout << "Default parameter mode" << std::endl;
        item_num = 30;
        md.set_simple(argv[1], item_num);
    }
    else if (argc == 3)
    {
        std::cout << "Simple parameter mode" << std::endl;
        item_num = atoi(argv[2]);
        md.set_simple(argv[1], item_num);
    }
    else if (argc == 18)
    {
        std::cout << "Detail parameter mode" << std::endl;
        md.set_detail(argv);
    }
    else
    {
        std::cout << "!!!Error!!! Check parameter" << std::endl;
    }
    std::cout << "************************************************" << std::endl;

    /*Termination Request Semaphore Initialization*/
    /*Initialized value at 1.*/
    sem_create = sem_init(&terminate_req_sem, 0, 1);
    if (0 != sem_create)
    {
        fprintf(stderr, "[ERROR] Failed to Initialize Termination Request Semaphore.\n");
        ret_main = -1;
        goto end_threads;
    }

    /*Create Key Hit Thread*/
    create_thread_key = pthread_create(&kbhit_thread, NULL, R_Kbhit_Thread, NULL);
    if (0 != create_thread_key)
    {
        fprintf(stderr, "[ERROR] Failed to create Key Hit Thread.\n");
        ret_main = -1;
        goto end_threads;
    }
    /*Create Capture Thread*/
    create_thread_capture = pthread_create(&capture_thread, NULL, R_Capture_Thread, NULL);
    if (0 != create_thread_capture)
    {
        sem_trywait(&terminate_req_sem);
        fprintf(stderr, "[ERROR] Failed to create Capture Thread.\n");
        ret_main = -1;
        goto end_threads;
    }
    /*Create ImageL Thread*/
    create_thread_imageL = pthread_create(&imageL_thread, NULL, R_ImageL_Thread, NULL);
    if (0 != create_thread_imageL)
    {
        sem_trywait(&terminate_req_sem);
        fprintf(stderr, "[ERROR] Failed to create ImageL Thread.\n");
        ret_main = -1;
        goto end_threads;
    }
    /*Create ImageR Thread*/
    create_thread_imageR = pthread_create(&imageR_thread, NULL, R_ImageR_Thread, NULL);
    if (0 != create_thread_imageR)
    {
        sem_trywait(&terminate_req_sem);
        fprintf(stderr, "[ERROR] Failed to create ImageR Thread.\n");
        ret_main = -1;
        goto end_threads;
    }
    /*Create Stereo Thread*/
    create_thread_stereo = pthread_create(&stereo_thread, NULL, R_Stereo_Thread, NULL);
    if (0 != create_thread_stereo)
    {
        sem_trywait(&terminate_req_sem);
        fprintf(stderr, "[ERROR] Failed to create Stereo Thread.\n");
        ret_main = -1;
        goto end_threads;
    }
    /*Create Display Thread*/
    create_thread_display = pthread_create(&display_thread, NULL, R_Display_Thread, NULL);
    if (0 != create_thread_display)
    {
        sem_trywait(&terminate_req_sem);
        fprintf(stderr, "[ERROR] Failed to create Display Thread.\n");
        ret_main = -1;
        goto end_threads;
    }

    /*Main Processing*/
    main_proc = R_Main_Process();
    if (0 != main_proc)
    {
        fprintf(stderr, "[ERROR] Error during Main Process\n");
        ret_main = -1;
    }
    goto end_threads;

end_threads:
    if (0 == create_thread_display)
    {
        ret = wait_join(&display_thread, DISPLAY_THREAD_TIMEOUT);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to exit Display Thread on time.\n");
            ret_main = -1;
        }
    }
    if (0 == create_thread_stereo)
    {
        ret = wait_join(&stereo_thread, STEREO_TIMEOUT);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to exit Stereo Thread on time.\n");
            ret_main = -1;
        }
    }
    if (0 == create_thread_imageR)
    {
        ret = wait_join(&imageR_thread, IMAGE_THREAD_TIMEOUT);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to exit ImageR Thread on time.\n");
            ret_main = -1;
        }
    }
    if (0 == create_thread_imageL)
    {
        ret = wait_join(&imageL_thread, IMAGE_THREAD_TIMEOUT);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to exit ImageL Thread on time.\n");
            ret_main = -1;
        }
    }
    if (0 == create_thread_capture)
    {
        ret = wait_join(&capture_thread, CAPTURE_TIMEOUT);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to exit Capture Thread on time.\n");
            ret_main = -1;
        }
    }
    if (0 == create_thread_key)
    {
        ret = wait_join(&kbhit_thread, KEY_THREAD_TIMEOUT);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to exit Key Hit Thread on time.\n");
            ret_main = -1;
        }
    }

    /*Delete Terminate Request Semaphore.*/
    if (0 == sem_create)
    {
        sem_destroy(&terminate_req_sem);
    }

end_main:
    printf("Application End\n");
    return ret_main;
}
