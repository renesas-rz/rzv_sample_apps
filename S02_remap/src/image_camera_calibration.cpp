/***********************************************************************************************************************
* Copyright (C) 2023-2025 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name    : image_camera_calibration.cpp
* Version      : 1.00
* Description  : RZ/V2N DRP-AI Sample Application for Remap with MIPI Camera
***********************************************************************************************************************/

/*****************************************
* Includes
******************************************/
#include "image_camera_calibration.h"
#include <opencv2/opencv.hpp>

Image::Image()
{

}


Image::~Image()
{

}


/*****************************************
* Function Name : get_H
* Description   : Function to get the image height
*                 This function is NOT used currently.
* Arguments     : -
* Return value  : img_h = current image height
******************************************/
uint32_t Image::get_H()
{
    return img_h;
}


/*****************************************
* Function Name : get_W
* Description   : Function to get the image width
*                 This function is NOT used currently.
* Arguments     : -
* Return value  : img_w = current image width
******************************************/
uint32_t Image::get_W()
{
    return img_w;
}


/*****************************************
* Function Name : get_C
* Description   : Function to set the number of image channel
*                 This function is NOT used currently.
* Arguments     : c = new number of image channel to be set
* Return value  : -
******************************************/
uint32_t Image::get_C()
{
    return img_c;
}

/*****************************************
* Function Name : get_img
* Description   : Function to return the camera buffer
* Arguments     : -
* Return value  : camera buffer
******************************************/
uint8_t* Image::get_img(uint8_t id)
{
    return img_buffer[id];
}

uint8_t* Image::get_overlay_img(uint8_t id)
{
    return overlay_buffer[id];
}


/*****************************************
* Function Name : init
* Description   : Function to initialize img_buffer in Image class
*                 This application uses udmabuf in order to
*                 continuous memory area for DRP-AI input data
* Arguments     : w = input image width in YUYV
*                 h = input image height in YUYV
*                 c = input image channel in YUYV
*                 ow = output image width in BGRA to be displayed via Wayland
*                 oh = output image height in BGRA to be displayed via Wayland
*                 oc = output image channel in BGRA to be displayed via Wayland
*                 mem_wayland = pointer to the memory for the display buffer
*                 mem_overlay = pointer to the memory for the overlay buffer
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
uint8_t Image::init(uint32_t w, uint32_t h, uint32_t c,
                    uint32_t ow, uint32_t oh, uint32_t oc, void *mem_wayland, void *mem_overlay)      
{
    int32_t i;
    
    /*Initialize input image information */
    img_w = w;
    img_h = h;
    img_c = c;
    /*Initialize output image information*/
    out_w = ow;
    out_h = oh;
    out_c = oc;

    uint32_t out_size = out_w * out_h * out_c;
    for (i = 0; i < WL_BUF_NUM; i++)
    {
        img_buffer[i] =(unsigned char*)mem_wayland+(i*out_size);
        overlay_buffer[i] =(unsigned char*)mem_overlay+(i*out_size);
    }

    return 0;
}

/*****************************************
* Function Name : convert_format
* Description   : Convert YUYV image to BGRA format
* Arguments     : -
* Return value  : -
******************************************/
void Image::convert_format()
{
#ifdef DEBUG_TIME_FLG
    using namespace std;
    chrono::system_clock::time_point start, end;
    start = chrono::system_clock::now();
#endif // DEBUG_TIME_FLG

    cv::Mat yuyv_image(img_h, img_w, CV_8UC2, img_buffer[buf_id]);
    cv::Mat bgra_image;
    cv::Mat out_image(img_h, img_w, CV_8UC4, img_buffer[buf_id]);
    cv::cvtColor(yuyv_image, bgra_image, cv::COLOR_YUV2BGRA_YUYV);
    memcpy(out_image.data, bgra_image.data, img_w * img_h * out_c);

#ifdef DEBUG_TIME_FLG
    end = chrono::system_clock::now();
    double time = static_cast<double>(chrono::duration_cast<chrono::microseconds>(end - start).count() / 1000.0);
    printf("Convert YUYV To BGRA Time : %lf[ms]\n", time);
#endif // DEBUG_TIME_FLG
}

uint8_t Image::Clip(int value)
{
    //unsigned char ret = (uint8_t)std::round(value);
    if (value > 255)
    {
        value = 255;
    }
    if (value < 0)
    {
        value = 0;
    }
    return value;
}


/*****************************************
* Function Name : convert_size
* Description   : Scale down the input data (1920x1080) to the output data (1280x720) using OpenCV.
* Arguments     : -
*                 in_w = width of current buffered image, which is mainly camera captured image.
*                 resize_w = width of resized image, which is mainly displayed on HDMI.
*                 in_h = height of current buffered image, which is mainly camera captured image.
*                 resize_h = height of resized image, which is mainly displayed on HDMI.
* Return value  : -
******************************************/
void Image::convert_size(int in_w, int resize_w, int in_h, int resize_h)
{
    // Return if resizing and padding is unnecessary
    if ( in_w == resize_w && in_h == resize_h )
    {
        return;
    }

#ifdef DEBUG_TIME_FLG
    using namespace std;
    chrono::system_clock::time_point start, end;
    start = chrono::system_clock::now();
#endif // DEBUG_TIME_FLG

    cv::Mat org_image(img_h, img_w, CV_8UC4, img_buffer[buf_id]);
    cv::Mat dst_image = org_image;  // shallow copy
    cv::Mat resize_image;
    cv::Mat padding_image;

    /* Resize */
    /* Use INTER_NEAREST which is faster than INTER_BILINEAR as long as OpenCVA is disabled. */
    cv::resize(dst_image, resize_image, cv::Size(resize_w, resize_h), 0, 0, cv::INTER_NEAREST);

    // Update reference (shallow copy)
    dst_image = resize_image;
    
    memcpy(img_buffer[buf_id], dst_image.data, resize_w * resize_h * out_c);	
    
#ifdef DEBUG_TIME_FLG
    end = chrono::system_clock::now();
    double time = static_cast<double>(chrono::duration_cast<chrono::microseconds>(end - start).count() / 1000.0);
    printf("Convert Size Time         : %lf[ms]\n", time);
#endif // DEBUG_TIME_FLG
}

/*****************************************
* Function Name : camera_to_image
* Description   : Function to copy the external image buffer data to img_buffer
*                 This is only place where the buf_id is updated.
* Arguments     : buffer = buffer to copy the image data
*                 size = size of buffer
* Return value  : none
******************************************/
void Image::camera_to_image(const uint8_t* buffer, int32_t size)
{
    /* Update buffer id */
    buf_id = (buf_id + 1) % WL_BUF_NUM;
    memcpy(img_buffer[buf_id], buffer, sizeof(uint8_t)*size);
}


/*****************************************
* Function Name : at
* Description   : Get the value of img_buffer at index a.
*                 This function is NOT used currently.
* Arguments     : a = index of img_buffer
* Return Value  : value of img_buffer at index a
******************************************/
uint8_t Image::at(int32_t a)
{
    return img_buffer[buf_id][a];
}

/*****************************************
* Function Name : set
* Description   : Set the value of img_buffer at index a.
*                 This function is NOT used currently.
* Arguments     : a = index of img_buffer
*                 val = new value to be set
* Return Value  : -
******************************************/
void Image::set(int32_t a, uint8_t val)
{
    img_buffer[buf_id][a] = val;
    return;
}
/*****************************************
* Function Name : get_buf_id
* Description   : Get the value of the buf_id.
* Arguments     : -
* Return Value  : value of buf_id-
******************************************/
uint8_t Image::get_buf_id(void)
{
    return buf_id;
}


/*****************************************
* Function Name : reset_overlay_img
* Description   : -
* Arguments     : -
* Return Value  : -
******************************************/
void Image::reset_overlay_img(uint8_t id)
{
#ifdef DEBUG_TIME_FLG
    using namespace std;
    chrono::system_clock::time_point start, end;
    start = chrono::system_clock::now();
#endif // DEBUG_TIME_FLG

    cv::Mat src_image = cv::Mat::zeros(out_h, out_w, CV_8UC4);
    uint8_t* dst = overlay_buffer[id];
    uint8_t* src = src_image.data;
    memcpy(dst, src, out_w * out_h * out_c);

#ifdef DEBUG_TIME_FLG
    end = chrono::system_clock::now();
    double time = static_cast<double>(chrono::duration_cast<chrono::microseconds>(end - start).count() / 1000.0);
    printf("Reset Overlay Buffer Time : %lf[ms]\n", time);
#endif // DEBUG_TIME_FLG
}

/*****************************************
* Function Name : write_string_rgb
* Description   : OpenCV putText() in RGB
* Arguments     : str = string to be drawn
*                 align_type : 1=TopLeft, 2=TopRight, 
*                              3=BottomMiddleOfRightCameraImage, 4=BottomMiddleOfLeftCameraImage
*                 x = bottom left coordinate X of string to be drawn
*                 y = bottom left coordinate Y of string to be drawn
*                 scale = scale for letter size
*                 color = letter color must be in RGB, e.g. white = 0xFFFFFF
* Return Value  : -
******************************************/
void Image::write_string_rgb(std::string str, uint32_t align_type, uint32_t x, uint32_t y, float scale, uint32_t color)
{
    uint8_t thickness = CHAR_THICKNESS;
    /*Extract RGB information*/
    uint8_t r = (color >> 16) & 0x0000FF;
    uint8_t g = (color >> 8) & 0x0000FF;
    uint8_t b = (color >> 0) & 0x0000FF;
    int ptx = 0;
    int pty = 0;

    /*OpenCV image data is in BGRA */
    cv::Mat bgra_image(out_h, out_w, CV_8UC4, img_buffer[buf_id]);

    int baseline = 0;
    cv::Size size = cv::getTextSize(str.c_str(), cv::FONT_HERSHEY_SIMPLEX, scale, thickness + 2, &baseline);
    if (align_type == 1)
    {
        ptx = x;
        pty = y;
    }
    else if (align_type == 2)
    {
        ptx = out_w - (size.width + x);
        pty = y;
    }
    else if(align_type == 3 || align_type == 4)
    {
        /*middle of camera image width*/
        int img_middle_w  = (CAM_RESIZED_WIDTH - size.width + x) / 2;

        if(align_type == 3)
        {
            /*bottom middle coodeinate X of left camera image*/
            ptx = img_middle_w;
        }
        else if(align_type == 4)
        {
            /*bottom middle coodeinate X of right camera image*/
            ptx = out_w - (img_middle_w + size.width);
        }

        /*padding height*/
        int padding_h = (out_h - CAM_RESIZED_HEIGHT) / 2;
        /*bottom middle coodeinate Y of left and right camera image*/
        pty = out_h - padding_h + size.height + y;
    }

    /*Color must be in BGR order*/
    cv::putText(bgra_image, str.c_str(), cv::Point(ptx, pty), cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0x00, 0x00, 0x00, 0xFF), thickness + 2);
    cv::putText(bgra_image, str.c_str(), cv::Point(ptx, pty), cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(b, g, r, 0xFF), thickness);
}

/*****************************************
* Function Name : convert_to_grayscale
* Description   : Chage gray scale 
* Arguments     : image_w = width of padding target image.
*                 resized_h = height of resized image.
*                 image = mat of padding target image.
* Return value  : 0 if succeeded
*               not 0 otherwise
******************************************/
cv::Mat Image::convert_to_grayscale(int img_w, int img_h)
{
    cv::Mat org_image(img_h, img_w, CV_8UC4, img_buffer[buf_id]);
    cv::Mat resized_img;
    cv::Mat img_gray;

    cv::cvtColor(org_image, img_gray, cv::COLOR_BGR2GRAY);

    return img_gray;
}

/*****************************************
* Function Name : calibrat_camera_img
* Description   : calibrat camera image.
* Arguments     : marge_maps = mat of calibration XY coodinates.
*                 img_w = width of  target camera image.
*                 img_h = height of target camera image.
* Return value  : calibrate image mat.
******************************************/
cv::Mat Image::calibrate_camera_img(cv::Mat marge_maps, int img_w, int img_h)
{
    cv::Mat ret_img;
    cv::Mat org_image(img_h, img_w, CV_8UC4, img_buffer[buf_id]);  
    
    /*calibrate*/
    cv::remap(org_image, ret_img, marge_maps, cv::Mat(), cv::INTER_LINEAR);

    return ret_img;
}

/*****************************************
* Function Name : convert_calib_img_size
* Description   : convert calib img size.
* Arguments     : resize_w = width of resized image.
*                 resize_h = height of resized image.
* Return value  : -
******************************************/
void Image::convert_calib_img_size(cv::Mat calib_ret_img, int resize_w, int resize_h)
{
    cv::Mat dst_image;

    /* Resize */
    /* Use INTER_NEAREST which is faster than INTER_BILINEAR as long as OpenCVA is disabled. */
    cv::resize(calib_ret_img, dst_image, cv::Size(resize_w, resize_h), 0, 0, cv::INTER_NEAREST);
    
    memcpy(overlay_buffer[buf_id], dst_image.data, resize_w * resize_h * out_c);	

}

/*****************************************
* Function Name : padding_img
* Description   : Padding image 
* Arguments     : image_w = width of padding target image.
*                 resized_h = height of resized image.
*                 image = mat of padding target image.
* Return value  : 0 if succeeded
*               not 0 otherwise
******************************************/
void Image::padding_img(int image_w, int resized_h, cv::Mat image)
{
    uint32_t pad_left = (this->out_w - image_w) / 2;
    uint32_t pad_right =  this->out_w - image_w - pad_left;
    uint32_t pad_top = (this->out_h - resized_h) / 2;
    uint32_t pad_bottom = this->out_h - resized_h - pad_top;

    cv::Mat dst_image;
    copyMakeBorder(image, dst_image, pad_top, pad_bottom, pad_left, pad_right, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0, 255));
    memcpy(this->img_buffer[buf_id], dst_image.data, this->out_w * this->out_h * this->out_c);
}

/*****************************************
* Function Name : create_side_by_side
* Description   : place cam image and depth map side by side
* Arguments     : resized_w = width of resized image.
*                 resized_h = height of resized image.
*                 is_padding = whether padding or not between segmentation image resolution and HDMI resolution.
* Return value  : 0 if succeeded
*               not 0 otherwise
******************************************/
int8_t Image::create_side_by_side(int resized_w, int resized_h, bool is_padding, int flip)
{
    cv::Mat cam_image(resized_h, resized_w, CV_8UC4, img_buffer[buf_id]);
    cv::Mat depth_image(resized_h, resized_w, CV_8UC4, overlay_buffer[buf_id]);
    cv::Mat concat_image;

    if(flip == 1)
    {
        cv::flip(cam_image, cam_image, 1);
        cv::flip(depth_image, depth_image, 1);
    }

    cv::hconcat(cam_image, depth_image, concat_image);

    // Embed resized concat image ((resized_w x 2) x resized_h x 4 ) into Display buffer ( out_w * out_h * out_c )
    uint32_t concat_w = resized_w * 2;
    if ( is_padding )
    {
        padding_img(concat_w, resized_h, concat_image);
    }
    else
    {
        if(concat_w != this->out_w || resized_h != this->out_h)
        {
            cv::resize(concat_image, concat_image, cv::Size(this->out_w, this->out_h), 0, 0, cv::INTER_LINEAR);
        }
        // no padding
        memcpy(this->img_buffer[buf_id], concat_image.data, this->out_w * this->out_h * this->out_c);
    }
    return 0;
}

/*****************************************
* Function Name : create_only_img
* Description   : place cam image in center.
* Arguments     : resized_w = width of resized image.
*                 resized_h = height of resized image.
* Return value  : 0 if succeeded
*               not 0 otherwise
******************************************/
int8_t Image::create_only_img(int resized_w, int resized_h, bool is_padding, int flip)
{
    cv::Mat cam_image(resized_h, resized_w, CV_8UC4, img_buffer[buf_id]);
    cv::Mat non_padding_image;

    if (flip == 1)
    {
        cv::flip(cam_image, cam_image, 1);
    }

    if(is_padding)
    {
        padding_img(resized_w, resized_h, cam_image);
    }
    else
    {
        if(resized_w != this->out_w || resized_h != this->out_h)
        {
            cv::resize(non_padding_image, non_padding_image, cv::Size(this->out_w, this->out_h), 0, 0, cv::INTER_LINEAR);
        }
        // no padding
        memcpy(this->img_buffer[buf_id], non_padding_image.data, this->out_w * this->out_h * this->out_c);
    }

    return 0;
}
