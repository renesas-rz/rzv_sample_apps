/***********************************************************************************************************************
* Copyright (C) 2023-2026 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name    : input_files.cpp
* Version      : R2026-06
* Description  : for RZ/V2H DRP-AI Sample Application with MIPI/USB Camera
***********************************************************************************************************************/

/*****************************************
* Includes
******************************************/
#include "input_files.h"
#include <errno.h>

Input_Files::Input_Files()
{
    camera_width    = CAM_IMAGE_WIDTH;
    camera_height   = CAM_IMAGE_HEIGHT;
    camera_color    = CAM_IMAGE_CHANNEL_YUY2;
}

Input_Files::~Input_Files()
{
}

/*****************************************
* Function Name : init
* Description   : Function to initialize Input Image Mode
* Arguments     : dir_path
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
int8_t Input_Files::init(const std::string& dir_path)
{
    images_dir = dir_path;
    png_files.clear();

    if (!std::filesystem::exists(images_dir) || !std::filesystem::is_directory(images_dir))
    {
        std::cerr << "[ERROR] Image dir not found: "
                  << images_dir << std::endl;
        return -1;
    }

    for (const auto& e : std::filesystem::directory_iterator(images_dir))
    {
        if (e.is_regular_file() && e.path().extension() == ".png")
        {
            png_files.push_back(e.path());
        }
    }

    std::sort(png_files.begin(), png_files.end());

    if (png_files.empty())
    {
        std::cerr << "[ERROR] No png files in " << images_dir << std::endl;
        return -1;
    }

    return 0;
}

/*****************************************
* Function Name : get_files_size
* Description   : Get Number of files size
* Arguments     : -
* Return value  : Get Number of files size
******************************************/
size_t Input_Files::get_files_size()
{
    return png_files.size();
}

/*****************************************
* Function Name : read_file
* Description   : Function to read image data from files
* Arguments     : idx_in , bgr(output)
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
int8_t Input_Files::read_file(int idx_in, cv::Mat& bgr)
{
    cv::Mat src = cv::imread(png_files[idx_in].string(), cv::IMREAD_COLOR);

    if (src.empty())
    {
        std::cerr << "[ERROR] Failed to read image: "
                  << png_files[idx_in] << std::endl;
        return -1;
    }

    const int src_w = src.cols;
    const int src_h = src.rows;

    const float scale_w = static_cast<float>(CAM_IMAGE_WIDTH)  / src_w;
    const float scale_h = static_cast<float>(CAM_IMAGE_HEIGHT) / src_h;
    const float scale   = std::min(scale_w, scale_h);

    const int resized_w = static_cast<int>(src_w * scale);
    const int resized_h = static_cast<int>(src_h * scale);

    cv::Mat resized;
    cv::resize(src, resized, cv::Size(resized_w, resized_h));

    bgr = cv::Mat::zeros(CAM_IMAGE_HEIGHT, CAM_IMAGE_WIDTH, CV_8UC3);

    const int offset_x = (CAM_IMAGE_WIDTH  - resized_w) / 2;
    const int offset_y = (CAM_IMAGE_HEIGHT - resized_h) / 2;

    resized.copyTo(
        bgr(cv::Rect(offset_x, offset_y, resized_w, resized_h))
    );

    return 0;
}

/*****************************************
* Function Name : allocate_buf_memory
* Description   : Function to initialize USB/MIPI camera capture
* Arguments     : -
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
int8_t Input_Files::allocate_buf_memory()
{
    int8_t ret = 0;

    wayland_buf = (dma_buffer*)malloc(sizeof(dma_buffer));
    ret = buffer_alloc_dmabuf(wayland_buf,WAYLANDBUF);  
    if (-1 == ret)
    {
        fprintf(stderr, "[ERROR] Failed to Allocate DMA buffer for the wayland_buf\n");
        return -1;
    }

    overlay_buf = (dma_buffer*)malloc(sizeof(dma_buffer));
    ret = buffer_alloc_dmabuf(overlay_buf,WAYLANDBUF);
    if (-1 == ret)
    {
        fprintf(stderr, "[ERROR] Failed to Allocate DMA buffer for the overlay_buf\n");
        return -1;
    }
    
    drpai_buf = (dma_buffer*)malloc(sizeof(dma_buffer));
    ret = buffer_alloc_dmabuf(drpai_buf,DRPAIBUF);
    if (-1 == ret)
    {
        fprintf(stderr, "[ERROR] Failed to Allocate DMA buffer for the drpai_buf\n");
        return -1;
    }
    
    dma_buf = (dma_buffer*)malloc(sizeof(dma_buffer));
    ret = buffer_alloc_dmabuf(dma_buf,CAPTUREBUF);
    if (-1 == ret)
    {
        fprintf(stderr, "[ERROR] Failed to Allocate DMA buffer for the dma_buf\n");
        return ret;
    }

    return 0;
}

/*****************************************
* Function Name : close_buf_memory
* Description   : Close camera and free buffer
* Arguments     : -
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
int8_t Input_Files::close_buf_memory()
{

    buffer_free_dmabuf(wayland_buf);
    free(wayland_buf);
    wayland_buf = NULL;

    buffer_free_dmabuf(overlay_buf);
    free(overlay_buf);
    overlay_buf = NULL;

    buffer_free_dmabuf(drpai_buf);
    free(drpai_buf);
    drpai_buf = NULL;

    buffer_free_dmabuf(dma_buf);
    free(dma_buf);
    dma_buf = NULL;

    return 0;
}

/*****************************************
* Function Name : get_img
* Description   : Function to return the camera buffer
* Arguments     : -
* Return value  : camera buffer
******************************************/
uint8_t * Input_Files::get_img()
{
    return (uint8_t *)dma_buf->mem;
}


/*****************************************
* Function Name : get_size
* Description   : Function to return the camera buffer size (W x H x C)
* Arguments     : -
* Return value  : camera buffer size (W x H x C )
******************************************/
int32_t Input_Files::get_size()
{
    return dma_buf->size;
}

/*****************************************
* Function Name : get_w
* Description   : Get camera_width. This function is currently NOT USED.
* Arguments     : -
* Return value  : camera_width = width of camera capture image.
******************************************/
int32_t Input_Files::get_w()
{
    return camera_width;
}

/*****************************************
* Function Name : set_w
* Description   : Set camera_width. This function is currently NOT USED.
* Arguments     : w = new camera capture image width
* Return value  : -
******************************************/
void Input_Files::set_w(int32_t w)
{
    camera_width= w;
    return;
}

/*****************************************
* Function Name : get_h
* Description   : Get camera_height. This function is currently NOT USED.
* Arguments     : -
* Return value  : camera_height = height of camera capture image.
******************************************/
int32_t Input_Files::get_h()
{
    return camera_height;
}

/*****************************************
* Function Name : set_h
* Description   : Set camera_height. This function is currently NOT USED.
* Arguments     : w = new camera capture image height
* Return value  : -
******************************************/
void Input_Files::set_h(int32_t h)
{
    camera_height = h;
    return;
}

/*****************************************
* Function Name : get_c
* Description   : Get camera_color. This function is currently NOT USED.
* Arguments     : -
* Return value  : camera_color = color channel of camera capture image.
******************************************/
int32_t Input_Files::get_c()
{
    return camera_color;
}

/*****************************************
* Function Name : set_c
* Description   : Set camera_color. This function is currently NOT USED.
* Arguments     : c = new camera capture image color channel
* Return value  : -
******************************************/
void Input_Files::set_c(int32_t c)
{
    camera_color= c;
    return;
}
