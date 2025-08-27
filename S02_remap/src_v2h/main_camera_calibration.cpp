/***********************************************************************************************************************
* Copyright (C) 2024-2025 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name    : main.cpp
* Version      : 1.00
* Description  : RZ/V2H DRP-AI Sample Application for MiDaS with MIPI/USB Camera
***********************************************************************************************************************/

/*****************************************
* Includes
******************************************/
/*Definition of Macros & other variables*/
#include "define.h"
/*MIPI camera control*/
#include "camera.h"
/*Image control*/
#include "image_camera_calibration.h"
/*Wayland control*/
#include "wayland.h"
/*Mutual exclusion*/
#include <mutex>
#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <filesystem>
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>

using namespace std;
/*****************************************
* Global Variables
******************************************/
/*Multithreading*/
static sem_t terminate_req_sem;
static pthread_t kbhit_thread;
static pthread_t capture_thread;
static pthread_t img_thread;
static pthread_t hdmi_thread;
static mutex mtx;

/*Flags*/
static atomic<uint8_t> img_obj_ready   (0);
static atomic<uint8_t> hdmi_obj_ready  (0);

/*Global Variables*/
static uint64_t capture_address;
static uint8_t buf_id;
static Image img;

static uint32_t disp_time = 0;
static uint32_t array_drp_time[30] = {1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000};
static uint32_t array_disp_time[30] = {1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000};
static int32_t drp_max_freq;
static int32_t drpai_freq;
#if END_DET_TYPE
static int8_t display_state=0;
#endif
static Wayland wayland;
static Camera* capture = NULL;

static double remap_time = 0;
static int thread_count = 0;

static bool calibration_flag = false;
static int  calibration_count;
static bool ckey_flag;
static int drp_flg;
static int flip_flag;

static std::vector<std::vector<cv::Point3f>> obj_points;
static std::vector<std::vector<cv::Point2f>> img_points;
static std::vector<cv::Point3f> pattern_points;
static cv::Mat camera_matrix;
static cv::Mat dist_coeffs;
static cv::Mat calibration_maps;

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
    if ( 0 == ret_err )
    {
        join_timeout.tv_sec += join_time;
        ret_err = pthread_timedjoin_np(*p_join_thread, NULL, &join_timeout);
    }
    return ret_err;
}

/*****************************************
* Function Name : file_exists
* Description   : check if a file exists.
* Arguments     : filename = target filename
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
bool file_exists(const std::string& filename) {
    std::ifstream file(filename);
    return file.good();
}

/*****************************************
* Function Name : print_result
* Description   : print the result on display.
* Arguments     : -
* Return value  : 0 if succeeded
*               not 0 otherwise
******************************************/
int8_t print_result(Image* img)
{
#ifdef DEBUG_TIME_FLG
    using namespace std;
    chrono::system_clock::time_point start, end;
    start = chrono::system_clock::now();
#endif // DEBUG_TIME_FLG

    mtx.lock();
    std::stringstream stream;
    std::string str = "";

    if(calibration_flag)
    {
        int ret = img->create_only_img(CAM_IMAGE_WIDTH * 2, CAM_IMAGE_HEIGHT * 2, CAM_RESIZED_PADDING, flip_flag);
        if (ret != 0)
        {
            return -1;
        }

        stream.str("");
        stream << "calibrate " << CALIBRATION_COUNT_MAX << " times, shifting the position";
        str = stream.str();
        img->write_string_rgb(str, 2, TEXT_WIDTH_OFFSET,  LINE_HEIGHT_OFFSET + (LINE_HEIGHT * 1), CHAR_SCALE_LARGE, 0xFFF000u);

        stream.str("");
        stream << "so that the entire chessboard is captured.";
        str = stream.str();
        img->write_string_rgb(str, 2, TEXT_WIDTH_OFFSET,  LINE_HEIGHT_OFFSET + (LINE_HEIGHT * 2), CHAR_SCALE_LARGE, 0xFFF000u);

        stream.str("");
        stream << "C Key : Calibration Execute";
        str = stream.str();
        img->write_string_rgb(str, 2, TEXT_WIDTH_OFFSET,  LINE_HEIGHT_OFFSET + (LINE_HEIGHT * 3), CHAR_SCALE_LARGE, 0xFFF000u);

        stream.str("");
        stream << "Enter Key : End";
        str = stream.str();
        img->write_string_rgb(str, 2, TEXT_WIDTH_OFFSET,  LINE_HEIGHT_OFFSET + (LINE_HEIGHT * 4), CHAR_SCALE_LARGE, 0xFFF000u);

        stream.str("");
        stream << "Calibration times : " << calibration_count <<  " / " << CALIBRATION_COUNT_MAX;
        str = stream.str();
        img->write_string_rgb(str, 2, TEXT_WIDTH_OFFSET,  LINE_HEIGHT_OFFSET + (LINE_HEIGHT * 5), CHAR_SCALE_LARGE, 0xFFF000u);

        if(calibration_count == CALIBRATION_COUNT_MAX)
        {
            stream.str("");
            stream << "Calculating Camera Calibration..." ;
            str = stream.str();
            img->write_string_rgb(str, 2, TEXT_WIDTH_OFFSET,  LINE_HEIGHT_OFFSET + (LINE_HEIGHT * 6.5), CHAR_SCALE_LARGE * 1.5, 0xFF0000);
        }

    }
    else
    {
        int ret = img->create_side_by_side(CAM_RESIZED_WIDTH, CAM_RESIZED_HEIGHT, CAM_RESIZED_PADDING, flip_flag);
        if (ret != 0)
        {
            return -1;
        }

        /* Draw Camera Resolution.*/
        stream.str("");
        stream << "Camera Resolution: " << CAM_IMAGE_WIDTH << "x" << CAM_IMAGE_HEIGHT;
        str = stream.str();
        img->write_string_rgb(str, 2, TEXT_WIDTH_OFFSET,  LINE_HEIGHT_OFFSET + (LINE_HEIGHT * 1), CHAR_SCALE_LARGE, 0xFFF000u);

        /* Draw OpenCV exe.*/
        stream.str("");
        if(drp_flg)
        {
            stream << "OpenCV exe: DRP";
        }
        else
        {
            stream << "OpenCV exe: CPU";
        }
        str = stream.str();
        img->write_string_rgb(str, 2, TEXT_WIDTH_OFFSET,  LINE_HEIGHT_OFFSET + (LINE_HEIGHT * 2), CHAR_SCALE_LARGE, 0xFFF000u);

        /* Draw OpenCV Accelerator Time.*/
        stream.str("");
        stream << "OpenCV Time(remap): " << std::setw(3) << std::fixed << std::setprecision(1) << std::round(remap_time * 10) / 10 << "msec";
        str = stream.str();
        img->write_string_rgb(str, 2, TEXT_WIDTH_OFFSET,  LINE_HEIGHT_OFFSET + (LINE_HEIGHT * 3), CHAR_SCALE_LARGE, 0xFFF000u);

        /* Draw Before Remap.*/
        stream.str("");
        stream << "Before Remap";
        str = stream.str();
        img->write_string_rgb(str, 3, TEXT_WIDTH_OFFSET, LINE_HEIGHT_OFFSET, CHAR_SCALE_LARGE, 0xFFF000u);

        /* Draw After Remap.*/
        stream.str("");
        stream << "After Remap";
        str = stream.str();
        img->write_string_rgb(str, 4, TEXT_WIDTH_OFFSET, LINE_HEIGHT_OFFSET, CHAR_SCALE_LARGE, 0xFFF000u);

    };

#ifdef DISP_AI_FRAME_RATE
    /* Draw AI/Camera Frame Rate on RGB image.*/
    stream.str("");
    stream << "AI/Camera Frame Rate: " << std::setw(3) << (uint32_t)ai_fps << "/" << (uint32_t)cap_fps << "fps";
    str = stream.str();
    img->write_string_rgb(str, 2, TEXT_WIDTH_OFFSET, LINE_HEIGHT_OFFSET + (LINE_HEIGHT * 5), CHAR_SCALE_LARGE, 0xFFF000u);
#endif /* DISP_AI_FRAME_RATE */

#ifdef DEBUG_TIME_FLG
    end = chrono::system_clock::now();
    double time = static_cast<double>(chrono::duration_cast<chrono::microseconds>(end - start).count() / 1000.0);
    printf("Draw Overlay Image Time   : %lf[ms]\n", time);
#endif // DEBUG_TIME_FLG
    mtx.unlock();

    return 0;
}

/*****************************************
* Function Name : add_chessboard_information
* Description   : add information of  calculating the chessboard coordinates.
* Arguments     : -
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
int8_t add_chessboard_information(cv::Size BOARD_SIZE)
{
    std::vector<cv::Point2f> corner;

    cv::Mat img_gray = img.convert_to_grayscale(CAM_IMAGE_WIDTH, CAM_IMAGE_HEIGHT);

    /*Determine if the chessboard is in the picture*/ 
    bool found = cv::findChessboardCorners(img_gray, BOARD_SIZE, corner);
    if(!found)
    {
        return -1;
    }

    /*Detecting corners location in subpixels*/
    cv::TermCriteria term(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1);
    cv::cornerSubPix(img_gray, corner, cv::Size(5, 5), cv::Size(-1, -1), term);

    /*Save chessboard information*/
    obj_points.push_back(pattern_points);
    img_points.push_back(corner);

    return 0;
}

/*****************************************
* Function Name : calc_calibration_result
* Description   : calculate calibration result.
* Arguments     : -
* Return value  : -
******************************************/
void calc_calibration_result()
{
    camera_matrix = cv::Mat();
    dist_coeffs   = cv::Mat(); 
    std::vector<cv::Mat> rvecs, tvecs;

    double rms = cv::calibrateCamera(obj_points, img_points, cv::Size(CAM_IMAGE_HEIGHT, CAM_RESIZED_WIDTH), camera_matrix, dist_coeffs, rvecs, tvecs);

    printf("RMS = %f\n", rms);
    printf("K = \n");
    for (int i = 0; i < camera_matrix.rows; i++) {
        for (int j = 0; j < camera_matrix.cols; j++) {
            printf("%f ", camera_matrix.at<double>(i, j));
        }
        printf("\n");
    }
    printf("d = ");
    for (int i = 0; i < dist_coeffs.total(); i++) {
        printf("%f ", dist_coeffs.at<double>(i));
    }
    printf("\n");
}

/*****************************************
* Function Name : save_matrix_to_file
* Description   : save calibarate matrix to file.
* Arguments     : -
* Return value  : -
******************************************/
void save_matrix_to_file(const cv::Mat& matrix, const std::string& filename) 
{
    std::ofstream file(filename);
    if (file.is_open()) {
        file << std::fixed << std::setprecision(14);
        for (int i = 0; i < matrix.rows; ++i) {
            for (int j = 0; j < matrix.cols; ++j) {
                file << matrix.at<double>(i, j);
                if (j < matrix.cols - 1) file << ",";
            }
            file << std::endl;
        }
        file.close();
        printf("save file : %d\n", filename);
    }
}

/*****************************************
* Function Name : load_camera_calibration_mat
* Description   : load camera calibration mat to file.
* Arguments     : -
* Return value  : camera calibration mat.
******************************************/
cv::Mat load_camera_calibration_mat(const std::string& file_path, int width, int height) {
    
    cv::Mat camera_calibration_mat = cv::Mat(height, width, CV_64F);
    try 
    {
        /*calibration file Open*/
        std::ifstream camera_calibration_file(file_path);

        /*faild to open calibration*/
        if(!camera_calibration_file)
        {
            return cv::Mat();
        }

        /*Create array from a CSV file */
        std::vector<vector<double>> camera_calibration_data;
        std::string line;

        /*line */
        while (std::getline(camera_calibration_file, line)) 
        {
            std::stringstream stringstream_line(line);
            std::string value;
            std::vector<double> camera_calibration_line_data;
            
            /*Cell */
            while (std::getline(stringstream_line, value, ',')) 
            {
                camera_calibration_line_data.push_back(std::stod(value));
            }

            camera_calibration_data.push_back(camera_calibration_line_data);
        }
        
        /*Check CSV data rows and cloums*/
        if(camera_calibration_data.size() != height || camera_calibration_data[0].size() != width)
        {
            fprintf(stderr, "[ERROR] %s : The size of the CSV array is incorrect.\n", file_path.c_str());
            return cv::Mat();
        }

        /*Convert vector to a mat */
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) 
            {
                camera_calibration_mat.at<double>(i, j) = camera_calibration_data[i][j];
            }
        }
    } 
    catch (const std::exception& e) 
    {
        fprintf(stderr, "[ERROR] Failed to load %s.\n", file_path.c_str());
        return cv::Mat();
    }
    
    return camera_calibration_mat;
}

/*****************************************
* Function Name : merge_maps
* Description   : Merge x and y coordinate maps
* Arguments     : -
* Return value  : x and y coordinate maps
******************************************/
cv::Mat merge_maps(const cv::Mat& map_x, const cv::Mat& map_y) {
    if (!(map_x.size() == map_y.size() && map_x.type() == CV_32FC1 && map_y.type() == CV_32FC1))
    {
        return cv::Mat();
    }

    cv::Mat calibration_maps(map_x.size(), CV_32FC2);
    cv::convertMaps(map_x, map_y, calibration_maps, cv::noArray(), CV_32FC2);

    return calibration_maps;
}

/*****************************************
* Function Name : calc_calibration_maps
* Description   : calculation calibration maps.
* Arguments     : filename = target filename
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
int8_t calc_calibration_maps()
{
    cv::Mat mapX;
    cv::Mat mapY;
    calibration_maps = cv::Mat();

    cv::Mat camera = load_camera_calibration_mat(CAMERA_FILE_PATH, CAMERA_FILE_MAT_WIDTH, CAMERA_FILE_MAT_HEIGHT);
    cv::Mat dist = load_camera_calibration_mat(DIST_FILE_PATH, DIST_FILE_MAT_WIDTH, DIST_FILE_MAT_HEIGHT);
    if(camera.empty() || dist.empty())
    {
        return -1;
    }

    cv::initUndistortRectifyMap(camera, dist, cv::Mat(), cv::Mat(), cv::Size(CAM_IMAGE_WIDTH, CAM_IMAGE_HEIGHT), CV_32FC1, mapX, mapY);
    calibration_maps = merge_maps(mapX, mapY);
    if(calibration_maps.empty())
    {
        return -1;
    }

    return 0;
}

/*****************************************
* Function Name : R_Capture_Thread
* Description   : Executes the V4L2 capture with Capture thread.
* Arguments     : threadid = thread identification
* Return value  : -
******************************************/
void *R_Capture_Thread(void *threadid)
{
    Camera* capture = (Camera*) threadid;
    /*Semaphore Variable*/
    int32_t capture_sem_check = 0;
    /*First Loop Flag*/
    uint64_t capture_addr = 0;
    int8_t ret = 0;
    uint8_t * img_buffer;
    uint8_t * img_buffer0;
    uint8_t capture_stabe_cnt = 8;  // Counter to wait for the camera to stabilize
#ifdef DISP_AI_FRAME_RATE
    int32_t cap_cnt = -1;
    static struct timespec capture_time;
    static struct timespec capture_time_prev = { .tv_sec = 0, .tv_nsec = 0, };
#endif /* DISP_AI_FRAME_RATE */
    
    printf("Capture Thread Starting\n");

    img_buffer0 = (uint8_t *)capture->drpai_buf->mem;
    capture_address = capture->drpai_buf->phy_addr;
    while(1)
    {
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

        /* Capture USB camera image and stop updating the capture buffer */
        capture_addr = (uint32_t)capture->capture_image();
#ifdef DISP_AI_FRAME_RATE
        cap_cnt++;
        ret = timespec_get(&capture_time, TIME_UTC);
        proc_time_capture = (timedifference_msec(capture_time_prev, capture_time) * TIME_COEF);
        capture_time_prev = capture_time;

        int idx = cap_cnt % SIZE_OF_ARRAY(array_cap_time);
        array_cap_time[idx] = (uint32_t)proc_time_capture;
        int arraySum = std::accumulate(array_cap_time, array_cap_time + SIZE_OF_ARRAY(array_cap_time), 0);
        double arrayAvg = 1.0 * arraySum / SIZE_OF_ARRAY(array_cap_time);
        cap_fps = 1.0 / arrayAvg * 1000.0 + 0.5;
#endif /* DISP_AI_FRAME_RATE */

        if (capture_addr == 0)
        {
            fprintf(stderr, "[ERROR] Failed to capture image from camera.\n");
            goto err;
        }
        else
        {
            /* Do not process until the camera stabilizes, because the image is unreliable until the camera stabilizes. */
            if( capture_stabe_cnt > 0 )
            {
                capture_stabe_cnt--;
            }
            else
            {
                img_buffer = capture->get_img();

                if (!img_obj_ready.load())
                {
                    img.camera_to_image(img_buffer, capture->get_size());
                    ret = capture->video_buffer_flush_dmabuf(capture->wayland_buf->idx, capture->wayland_buf->size);
                    if (0 != ret)
                    {
                        goto err;   
                    }
                    img_obj_ready.store(1); /* Flag for Display Thread. */
                }
            }
        }

        /* IMPORTANT: Place back the image buffer to the capture queue */
        ret = capture->capture_qbuf();
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to enqueue capture buffer.\n");
            goto err;
        }
    } /*End of Loop*/

/*Error Processing*/
err:
    sem_trywait(&terminate_req_sem);
    goto capture_end;

capture_end:

    printf("Capture Thread Terminated\n");
    pthread_exit(NULL);
}

/*****************************************
* Function Name : R_Img_Thread
* Description   : Executes img proc with img thread
* Arguments     : threadid = thread identification
* Return value  : -
******************************************/
void *R_Img_Thread(void *threadid)
{
    /*Semaphore Variable*/
    int32_t hdmi_sem_check = 0;
    /*Variable for checking return value*/
    int8_t ret = 0;
    timespec start_time;
    timespec end_time;

    bool is_calibaration_files = true;
    cv::Size BOARD_SIZE(BOARD_WIDTH, BOARD_HEIGHT);

    printf("Image Thread Starting\n");

    if(calibration_flag)
    {
        /*Calculate 3D coordinates of each point on the chessboard*/
        pattern_points.reserve(BOARD_SIZE.width * BOARD_SIZE.height);
    
        for (int i = 0; i < BOARD_SIZE.height; ++i) 
        {
            for (int j = 0; j < BOARD_SIZE.width; ++j) 
            {
                // x, y coordinates are multiples of SQUARE_SIZE, z coordinate is 0
                pattern_points.push_back(cv::Point3f(j * SQUARE_SIZE, i * SQUARE_SIZE, 0));
            }
        }
    }
    else
    {
        /*Open calibration files and calculate the map*/
        if(!file_exists(CAMERA_FILE_PATH) && !file_exists(DIST_FILE_PATH))
        {
            fprintf(stderr, "[ERROR] Failed to find calibaration files.\n");
            goto err;
        }

        ret = calc_calibration_maps();
        if(0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to calculate calibration maps.\n");
            goto err;
        }
    }

    while(1)
    {
        /*Gets The Termination Request Semaphore Value, If Different Then 1 Termination Is Requested*/
        /*Checks If sem_getvalue Is Executed Without Issue*/
        errno = 0;
        ret = sem_getvalue(&terminate_req_sem, &hdmi_sem_check);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to get Semaphore Value: errno=%d\n", errno);
            goto err;
        }
        /*Checks the semaphore value*/
        if (1 != hdmi_sem_check)
        {
            goto hdmi_end;
        }
        /* Check img_obj_ready flag which is set in Capture Thread. */
        if (img_obj_ready.load())
        {
            ret = timespec_get(&start_time, TIME_UTC);
            if (0 == ret)
            {
                fprintf(stderr, "[ERROR] Failed to get Display Start Time\n");
                goto err;
            }
                        
            /* Convert YUYV image to BGRA format. */
            img.convert_format();

            if(calibration_flag)
            {                
                if(ckey_flag)
                {
                    ckey_flag = false;
                    if(calibration_count >= CALIBRATION_COUNT_MAX)
                    {
                        fprintf(stderr, "[ERROR] Failed to calibration count.\n");
                        goto err;                    
                    }
                    
                    ret = add_chessboard_information(BOARD_SIZE);
                    if (0 != ret)
                    {
                        printf("Not find Chessboard Corners.\n");
                    }
                    else
                    {
                        calibration_count++;
                    }
                }

                /* Convert output image size. */
                img.convert_size(CAM_IMAGE_WIDTH, CAM_IMAGE_WIDTH  * 2, CAM_IMAGE_HEIGHT, CAM_IMAGE_HEIGHT * 2);
            }
            else
            {
                if(is_calibaration_files)
                {
                    thread_count++;
                    if(thread_count == REMAP_TIME_THREAD_COUNT_MAX)
                    {
                        ret = timespec_get(&start_time, TIME_UTC);
                        if (0 == ret)
                        {
                            fprintf(stderr, "[ERROR] Failed to get DRP remap Start Time\n");
                            goto err;
                        }

                    }

                    cv::Mat clib_ret_img = img.calibrate_camera_img(calibration_maps, CAM_IMAGE_WIDTH, CAM_IMAGE_HEIGHT);

                    if(thread_count == REMAP_TIME_THREAD_COUNT_MAX)
                    {
                        /*Gets AI Pre-process End Time*/
                        ret = timespec_get(&end_time, TIME_UTC);
                        if ( 0 == ret)
                        {
                            fprintf(stderr, "[ERROR] Failed to Get DRP remap End Time\n");
                            goto err;
                        }

                        remap_time = (timedifference_msec(start_time, end_time) * TIME_COEF);

                        thread_count = 0;
                    }

                    img.convert_calib_img_size(clib_ret_img, CAM_RESIZED_WIDTH, CAM_RESIZED_HEIGHT);

                    /* Convert output image size. */
                    img.convert_size(CAM_IMAGE_WIDTH, CAM_RESIZED_WIDTH ,CAM_IMAGE_HEIGHT, CAM_RESIZED_HEIGHT);
                }
            }

            /*displays Results on display.*/
            ret = print_result(&img);
            if (0 != ret)
            {
                fprintf(stderr, "[ERROR] Failed to displays Results on display.\n");
                goto err;
            }
            buf_id = img.get_buf_id();
            img_obj_ready.store(0);

            if (!hdmi_obj_ready.load())
            {
                hdmi_obj_ready.store(1); /* Flag for AI Inference Thread. */
            }
            
            ret = timespec_get(&end_time, TIME_UTC);
            if (0 == ret)
            {
                fprintf(stderr, "[ERROR] Failed to Get Display End Time\n");
                goto err;
            }
#ifdef DEBUG_TIME_FLG
            double img_proc_time = (timedifference_msec(start_time, end_time) * TIME_COEF);
            printf("Img Proc Time             : %lf[ms]\n", img_proc_time);
#endif
        }

        if(calibration_flag && calibration_count == CALIBRATION_COUNT_MAX)
        {
            calc_calibration_result();
            save_matrix_to_file(camera_matrix, CAMERA_FILE_PATH);
            save_matrix_to_file(dist_coeffs, DIST_FILE_PATH);

            if(!file_exists(CAMERA_FILE_PATH) && !file_exists(DIST_FILE_PATH))
            {
                fprintf(stderr, "[ERROR] Failed to find calibaration files.\n");
                goto err;
            }

            ret = calc_calibration_maps();
            if(0 != ret)
            {
                fprintf(stderr, "[ERROR] Failed to calculate calibration maps.\n");
                goto err;
            }            
            calibration_flag = false;
        }

        usleep(WAIT_TIME); //wait 1 tick time
    } /*End Of Loop*/
    
/*Error Processing*/
err:
    /*Set Termination Request Semaphore To 0*/
    sem_trywait(&terminate_req_sem);
    goto hdmi_end;

hdmi_end:
    /*To terminate the loop in Capture Thread.*/
    img_obj_ready.store(0);
    printf("Img Thread Terminated\n");
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
    /*Semaphore Variable*/
    int32_t hdmi_sem_check = 0;
    /*Variable for checking return value*/
    int8_t ret = 0;
    int32_t disp_cnt = 0;

    timespec start_time;
    timespec end_time;
    static struct timespec disp_prev_time = { .tv_sec = 0, .tv_nsec = 0, };

    /* Initialize waylad (draw overlay)*/
    ret = wayland.init(capture->wayland_buf->idx, IMAGE_OUTPUT_WIDTH, IMAGE_OUTPUT_HEIGHT, IMAGE_CHANNEL_BGRA, false);
    if(0 != ret)
    {
        fprintf(stderr, "[ERROR] Failed to initialize Image for Wayland\n");
        goto err;
    }

    printf("Display Thread Starting\n");
    while(1)
    {
        /*Gets The Termination Request Semaphore Value, If Different Then 1 Termination Is Requested*/
        /*Checks If sem_getvalue Is Executed Without Issue*/
        errno = 0;
        ret = sem_getvalue(&terminate_req_sem, &hdmi_sem_check);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to get Semaphore Value: errno=%d\n", errno);
            goto err;
        }
        /*Checks the semaphore value*/
        if (1 != hdmi_sem_check)
        {
            goto hdmi_end;
        }
        /* Check hdmi_obj_ready flag which is set in Capture Thread. */
        if (hdmi_obj_ready.load())
        {
            ret = timespec_get(&start_time, TIME_UTC);
            if (0 == ret)
            {
                fprintf(stderr, "[ERROR] Failed to get Display Start Time\n");
                goto err;
            }
            /*Update Wayland*/
            wayland.commit(img.get_img(buf_id), NULL);

            /*Reset Overlay image*/
            img.reset_overlay_img(buf_id);

#if END_DET_TYPE // To display the app_pointer_det in front of this application.
            if (display_state == 0) 
            {
                display_state = 1;
            }
#endif

            hdmi_obj_ready.store(0);
            ret = timespec_get(&end_time, TIME_UTC);
            if (0 == ret)
            {
                fprintf(stderr, "[ERROR] Failed to Get Display End Time\n");
                goto err;
            }
            disp_time = (uint32_t)((timedifference_msec(disp_prev_time, end_time) * TIME_COEF));
            int idx = disp_cnt++ % SIZE_OF_ARRAY(array_disp_time);
            array_disp_time[idx] = disp_time;
            disp_prev_time = end_time;
#ifdef DEBUG_TIME_FLG
            double disp_proc_time = (timedifference_msec(start_time, end_time) * TIME_COEF);
            /* Draw Disp Frame Rate on RGB image.*/
            int arraySum = std::accumulate(array_disp_time, array_disp_time + SIZE_OF_ARRAY(array_disp_time), 0);
            double arrayAvg = 1.0 * arraySum / SIZE_OF_ARRAY(array_disp_time);
            double disp_fps = 1.0 / arrayAvg * 1000.0;

            printf("Disp Proc Time            : %lf[ms]\n", disp_proc_time);
            printf("Disp Frame Rate           : %lf[fps]\n", disp_fps);
            printf("Dipslay ------------------------------ No. %d\n", disp_cnt);
#endif
        }
        usleep(WAIT_TIME); //wait 1 tick time
    } /*End Of Loop*/

/*Error Processing*/
err:
    /*Set Termination Request Semaphore To 0*/
    sem_trywait(&terminate_req_sem);
    goto hdmi_end;

hdmi_end:
    /*To terminate the loop in Capture Thread.*/
    hdmi_obj_ready.store(0);
    printf("Display Thread Terminated\n");
    pthread_exit(NULL);
}

/*****************************************
* Function Name : R_Kbhit_Thread
* Description   : Executes the Keyboard hit thread (checks if enter key is hit)
* Arguments     : threadid = thread identification
* Return value  : -
******************************************/
void *R_Kbhit_Thread(void *threadid)
{
    /*Semaphore Variable*/
    int32_t kh_sem_check = 0;
    /*Variable to store the getchar() value*/
    int32_t c = 0;
    /*Variable for checking return value*/
    int8_t ret = 0;

    printf("Key Hit Thread Starting\n");

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

    while(1)
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

#if END_DET_TYPE 
        // 1. Receive the end command via named pipe /tmp/appdetect from app_pointer_det.
        // 2. Send the end command via named pipe /tmp/gui to app_rzv2h_demo
        int fd;
        char str[BUF_SIZE];
        char str_end[BUF_SIZE] = "end";
        ssize_t size;
        mkfifo("/tmp/appdetect", 0666);
        fd = open("/tmp/appdetect", O_RDWR);
        size = read(fd, str, BUF_SIZE);
        if (size > 0)
        {
            /* When mouse clicked. */
            printf("mouse clicked. : %s\n", str);
            str[size] = '\n';

            if (strcmp(str, str_end) == 0)
            {
                if (system("echo \"end\" > /tmp/gui") == -1)
                {
                    printf("[ERROR] Failed to send command\n");
                }
                goto err;
            }
        }
        close(fd);
#else
        // Get input characters in real-time
        struct termios before, after;
        int key, bef;
        
        tcgetattr(STDIN_FILENO, &before);
        after = before;
        after.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &after);
        bef = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, bef | O_NONBLOCK);

        c = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &before);
        fcntl(STDIN_FILENO, F_SETFL, bef);

        // C key
        if(c == 'C' || c == 'c')
        {
            printf("key Detected.\n");
            ckey_flag = true;
        }
        // enter key
        else if (EOF != c)
        {
            /* When key is pressed. */
            printf("key Detected.\n");
            goto err;
        }
#endif // END_DET_TYPE

        /* When nothing is detected. */
        usleep(WAIT_TIME);
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
* Function Name : R_Main_Process
* Description   : Runs the main process loop
* Arguments     : -
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
int8_t R_Main_Process()
{
    /*Main Process Variables*/
    int8_t main_ret = 0;
    /*Semaphore Related*/
    int32_t sem_check = 0;
    /*Variable for checking return value*/
    int8_t ret = 0;

    printf("Main Loop Starts\n");
    while(1)
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
#if END_DET_TYPE // To launch app_pointer_det.
        if (display_state == 1)
        {
            if (system("./../app_pointer_det & ") == -1)
            {
                printf("Command Error\n");
                goto main_proc_end;
            }
            display_state = 2;
        }
#endif
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

int32_t main(int32_t argc, char * argv[])
{
    /* Log File Setting */
    FILE *OpenCV_BinFp = NULL;
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_INFO);

    auto now = std::chrono::system_clock::now();
    auto tm_time = spdlog::details::os::localtime(std::chrono::system_clock::to_time_t(now));
    char date_buf[64];
    char time_buf[128];
    memset(time_buf,0,sizeof(time_buf));
    std::strftime(date_buf, sizeof(date_buf), "%Y-%m-%d_%H-%M-%S", &tm_time);
    sprintf(time_buf,"logs/%s_app_topformer_cam.log",date_buf);
    auto logger = spdlog::basic_logger_mt("logger", time_buf);
    spdlog::set_default_logger(logger);

    int8_t main_proc = 0;
    int8_t ret = 0;
    int8_t ret_main = 0;
    /*Multithreading Variables*/
    int32_t create_thread_key = -1;
    int32_t create_thread_capture = -1;
    int32_t create_thread_img = -1;
    int32_t create_thread_hdmi = -1;
    int32_t sem_create = -1;

    /*initialize*/
    obj_points = std::vector<std::vector<cv::Point3f>>();
    img_points = std::vector<std::vector<cv::Point2f>>();

    /* Check  command-line argument  */
    calibration_count = 0;
    calibration_flag = false;
    drp_flg = 0;
    flip_flag = 0;
    ckey_flag = false;


    if (argc > 1 && argv[1] != NULL) 
    {
        int first_command_line = -1;
        int second_command_line = -1;
        int third_command_line = -1;
        try
        {
            first_command_line = std::stoi(argv[1]);

            if(argc > 2 && argv[2] != NULL)
            {
                second_command_line = std::stoi(argv[2]);
            }
            else
            {
                second_command_line = 0;
            }

            if(argc > 3 && argv[3] != NULL)
            {
                third_command_line = std::stoi(argv[3]);
            }
            else
            {
                third_command_line = 0;
            }
        }
        catch(const std::exception& e)
        {
            fprintf(stderr, "[ERROR] Please enter 0 or 1 as a command-line argument.\n");
            goto end_main;
        }

        if(first_command_line == 1)
        {
            drp_flg = 1;
        }
        else if(first_command_line != 0)
        {
            fprintf(stderr, "[ERROR] Please enter 0 or 1 as a command-line argument.\n");
            goto end_main;
        }

        if(second_command_line == 1)
        {
            flip_flag = 1;
        }
        else if (second_command_line != 0)
        {
            fprintf(stderr, "[ERROR] Please enter 0 or 1 as a command-line argument.\n");
            goto end_main;
        }

        if(third_command_line == 1)
        {
            calibration_flag = true;
        }
        else if (third_command_line != 0)
        {
            fprintf(stderr, "[ERROR] Please enter 0 or 1 as a command-line argument.\n");
            goto end_main;
        }
    }
    else
    {
        fprintf(stderr, "[ERROR] Please enter 0 or 1 as the first command-line argument.\n");
        goto end_main;
    }

    unsigned long OCA_list[DRP_FUNC_NUM];
    std::fill(std::begin(OCA_list), std::end(OCA_list), drp_flg);
    OCA_Activate( &OCA_list[0]);

    drp_max_freq = DRP_MAX_FREQ;
    drpai_freq = DRPAI_FREQ;

    printf("RZ/V2H DRP-AI Sample Application\n");
    printf("Input : %s\n", INPUT_CAM_NAME);
    spdlog::info("************************************************");
    spdlog::info("  RZ/V2H DRP-AI Sample Application");
    spdlog::info("  Input : {}", INPUT_CAM_NAME);
    spdlog::info("************************************************");
    printf("Argument : <DRP0_max_freq_factor> = %d\n", drp_max_freq);
    printf("Argument : <AI-MAC_freq_factor> = %d\n", drpai_freq);
    
    errno = 0;

    /* Create Camera Instance */
    capture = new Camera();

    /* Init and Start Camera */
    ret = capture->start_camera();
    if (0 != ret)
    {
        fprintf(stderr, "[ERROR] Failed to initialize Camera.\n");
        delete capture;
        ret_main = ret;
        goto end_main;
    }

    /*Initialize Image object.*/
    ret = img.init(CAM_IMAGE_WIDTH, CAM_IMAGE_HEIGHT, CAM_IMAGE_CHANNEL_YUY2, IMAGE_OUTPUT_WIDTH, IMAGE_OUTPUT_HEIGHT, IMAGE_CHANNEL_BGRA, capture->wayland_buf->mem, capture->overlay_buf->mem);
    if (0 != ret)
    {
        fprintf(stderr, "[ERROR] Failed to initialize Image object.\n");
        ret_main = ret;
        goto end_close_camera;
    }

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
    create_thread_capture = pthread_create(&capture_thread, NULL, R_Capture_Thread, (void *) capture);
    if (0 != create_thread_capture)
    {
        sem_trywait(&terminate_req_sem);
        fprintf(stderr, "[ERROR] Failed to create Capture Thread.\n");
        ret_main = -1;
        goto end_threads;
    }

    /*Create Image Thread*/
    create_thread_img = pthread_create(&img_thread, NULL, R_Img_Thread, NULL);
    if(0 != create_thread_img)
    {
        sem_trywait(&terminate_req_sem);
        fprintf(stderr, "[ERROR] Failed to create Image Thread.\n");
        ret_main = -1;
        goto end_threads;
    }

	/*Create Display Thread*/
    create_thread_hdmi = pthread_create(&hdmi_thread, NULL, R_Display_Thread, NULL);
    if(0 != create_thread_hdmi)
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
    if(0 == create_thread_hdmi)
    {
        ret = wait_join(&hdmi_thread, DISPLAY_THREAD_TIMEOUT);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to exit Display Thread on time.\n");
            ret_main = -1;
        }
    }
    if(0 == create_thread_img)
    {
        ret = wait_join(&img_thread, DISPLAY_THREAD_TIMEOUT);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to exit Image Thread on time.\n");
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

    /* Exit waylad */
    wayland.exit();
    goto end_close_camera;

end_close_camera:
    /*Close MIPI Camera.*/
    ret = capture->close_camera();
    if (0 != ret)
    {
        fprintf(stderr, "[ERROR] Failed to close Camera.\n");
        ret_main = -1;
    }
    delete capture;
    goto end_main;

end_main:    
    printf("Application End\n");
    return ret_main;
}
