/***********************************************************************************************************************
* Copyright (C) 2023-2026 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name    : image_yolov8_seg_stereo.cpp
* Version      : R2026-07
* Description  : RZ/V2H DRP-AI Sample Application for Ultralytics Detection YOLOv8 with MIPI/USB Camera
***********************************************************************************************************************/

/*****************************************
* Includes
******************************************/
#include "image_yolov8_seg_stereo.h"

Image::Image()
{
    cameraMatrix = cv::Mat(3, 3, CV_64FC1, cm);
    distCoeffs = cv::Mat(5, 1, CV_64FC1, dc);
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

/*****************************************
* Function Name : get_overlay_img
* Description   : Function to return the camera buffer
* Arguments     : -
* Return value  : overlay_buffer
******************************************/
uint8_t* Image::get_overlay_img(uint8_t id)
{
    return overlay_buffer[id];
}

/****************************************
* Function Name : get_disparity_u8_cam
* Description   : Function to return the camera buffer
* Arguments     : -
* Return value  : disparity_cam
******************************************/
cv::Mat Image::get_disparity_u8_cam()
{
    return disparity_u8_cam;
}

/****************************************
* Function Name : get_disparity_s16_cam
* Description   : Function to return the camera buffer
* Arguments     : -
* Return value  : disparity_cam
******************************************/
cv::Mat Image::get_disparity_s16_cam()
{
    return disparity_s16_cam;
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
*                 mem = pointer to the memory for the display buffer
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
uint8_t Image::init(uint32_t w, uint32_t h, uint32_t c,
                    uint32_t ow, uint32_t oh, uint32_t oc, void *mem)
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
        img_buffer[i] =(unsigned char*)mem+(i*out_size);
    }

    /* warpAffine */
    aff_ctr.x = img_w / 2.0;
    aff_ctr.y = img_h / 2.0;

    return 0;
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
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
uint8_t Image::init(uint32_t w, uint32_t h, uint32_t c,
                    uint32_t ow, uint32_t oh, uint32_t oc)
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
        img_buffer[i] = new uint8_t[out_size];
    }
    
    /* warpAffine */
    aff_ctr.x = img_w / 2.0;
    aff_ctr.y = img_h / 2.0;
    
    return 0;
}

/*****************************************
* Function Name : write_char
* Description   : Display character in overlap buffer
* Arguments     : code = code to be displayed
*                 x = X coordinate to display character
*                 y = Y coordinate to display character
*                 color = character color
* Return value  : -
******************************************/
void Image::write_char(char code,  uint32_t x,  uint32_t y, uint32_t color, uint32_t backcolor)
{
    uint32_t height;
    uint32_t width;
    char * p_pattern;
    uint8_t mask = 0x80;

    if ((code >= 0x20) && (code <= 0x7e))
    {
        p_pattern = (char *)&g_ascii_table[code - 0x20][0];
    }
    else
    {
        p_pattern = (char *)&g_ascii_table[10][0]; /* '*' */
    }

    /* Drawing */
    for (height = 0; height < font_h; height++)
    {
        for (width = 0; width < font_w; width++)
        {
            if (p_pattern[width] & mask)
            {
                draw_point_yuyv( width + x, height + y , color );
            }
            else
            {
                draw_point_yuyv( width + x, height + y , backcolor );
            }
        }
        mask = (uint8_t)(mask >> 1);
    }
    return;
}

/*****************************************
* Function Name : write_string_rgb
* Description   : OpenCV putText() in RGB
* Arguments     : str = string to be drawn
*                 x = bottom left coordinate X of string to be drawn
*                 y = bottom left coordinate Y of string to be drawn
*                 scale = scale for letter size
*                 color = letter color must be in RGB, e.g. white = 0xFFFFFF
* Return Value  : -
******************************************/
void Image::write_string_rgb(std::string str, uint32_t align_type,  uint32_t x, uint32_t y, float scale, uint32_t color)
{
    uint8_t thickness = CHAR_THICKNESS;
    /*Extract RGB information*/
    uint8_t r = (color >> 16) & 0x0000FF;
    uint8_t g = (color >>  8) & 0x0000FF;
    uint8_t b = color & 0x0000FF;
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
    /*Color must be in BGR order*/
    cv::putText(bgra_image, str.c_str(), cv::Point(ptx, pty), cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0x00, 0x00, 0x00, 0xFF), thickness + 2);
    cv::putText(bgra_image, str.c_str(), cv::Point(ptx, pty), cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(b, g, r, 0xFF), thickness);
}

/*****************************************
* Function Name : write_string_rgb_boundingbox
* Description   : OpenCV putText() in RGB
* Arguments     : str = string to be drawn
*                 x = bottom left coordinate X of string to be drawn
*                 y = bottom left coordinate Y of string to be drawn
*                 scale = scale for letter size
*                 color = letter color must be in RGB, e.g. white = 0xFFFFFF
* Return Value  : -
******************************************/
void Image::write_string_rgb_boundingbox(std::string str, uint32_t align_type,  uint32_t x_min, uint32_t y_min, uint32_t x_max, uint32_t y_max,float scale, uint32_t color)
{
    uint8_t thickness = CHAR_THICKNESS_BOX;
    /*Extract RGB information*/
    uint8_t r = (color >> 16) & 0x0000FF;
    uint8_t g = (color >>  8) & 0x0000FF;
    uint8_t b = color & 0x0000FF;
	
    int ptx = 0;
    int pty = 0;
    /*OpenCV image data is in BGRA */
    cv::Mat bgra_image(img_h, img_w, CV_8UC4, img_buffer[buf_id]);

    int baseline = 0;
    cv::rectangle(bgra_image, cv::Point(x_min,y_min), cv::Point(x_max,y_max), cv::Scalar(b, g, r, 0xFF), BOX_LINE_SIZE);
    
    cv::Size size = cv::getTextSize(str.c_str(), cv::FONT_ITALIC, scale, thickness + 2, &baseline);
    if (align_type == 1)
    {
        ptx = x_min;
        pty = y_min;
    }
    else if (align_type == 2)
    {
        ptx = img_w - (size.width + x_min);
        pty = y_min;
    }
    cv::rectangle(bgra_image, cv::Point(ptx-BOX_LINE_SIZE+1,pty-BOX_HEIGHT_OFFSET), cv::Point(ptx+size.width,pty), cv::Scalar(b, g, r, 0xFF), cv::FILLED);
    /*Color must be in BGR order*/
    cv::putText(bgra_image, str.c_str(), cv::Point(ptx, pty-BOX_TEXT_HEIGHT_OFFSET), cv::FONT_ITALIC, scale, cv::Scalar(0x00, 0x00, 0x00, 0xFF), thickness);
}

/*****************************************
* Function Name : write_string
* Description   : Display character string in overlap buffer
* Arguments     : pcode = A pointer to the character string to be displayed
*                 x = X coordinate to display character string
*                 y = Y coordinate to display character string
*                 color = character string color
* Return Value  : -
******************************************/
void Image::write_string(const char * pcode, uint32_t x, uint32_t y, uint32_t color, uint32_t backcolor)
{
    uint32_t i;
    uint32_t len = strlen(pcode);

    x = x < 0 ? 2 : x;
    x = (x > img_w - (len * font_w)) ? img_w - (len * font_w)-2 : x;
    y = y < 0 ? 2 : y;
    y = (y > img_h - font_h) ? img_h - font_h - 2 : y;

    for (i = 0; i < len; i++)
    {
        write_char(pcode[i], (x + (i * font_w)), y, color, backcolor);
    }

    return;
}

/*****************************************
* Function Name : draw_point_yuyv
* Description   : Draw a single point on YUYV image
* Arguments     : x = X coordinate to draw a point
*                 y = Y coordinate to draw a point
*                 color = point color
* Return Value  : -
******************************************/
void Image::draw_point_yuyv(int32_t x, int32_t y, uint32_t color)
{
    uint32_t x0 = (uint32_t)x & ~0x1;

    uint32_t draw_u = (color >> 8) & 0xFF;
    uint32_t draw_v = (color >> 0) & 0xFF;

    uint32_t target_u = img_buffer[buf_id][(y * img_w + x0) * img_c + 1];
    uint32_t target_v = img_buffer[buf_id][(y * img_w + x0) * img_c + 3];

    img_buffer[buf_id][(y * img_w + x0) * img_c + 1] = (draw_u + target_u) / 2;
    img_buffer[buf_id][(y * img_w + x0) * img_c + 3] = (draw_v + target_v) / 2;

    if ((x & 1) == 0)
    {
        img_buffer[buf_id][(y * img_w + x0) * img_c]     = (color >> 16) & 0xFF;
    }
    else
    {
        img_buffer[buf_id][(y * img_w + x0) * img_c + 2] = (color >> 16) & 0xFF;
    }
    return;
}

/*****************************************
* Function Name : draw_line
* Description   : Draw a single line
* Arguments     : x0 = X coordinate of a starting point
*                 y0 = Y coordinate of a starting point
*                 x1 = X coordinate of a end point
*                 y1 = Y coordinate of a end point
*                 color = line color
* Return Value  : -
******************************************/
void Image::draw_line(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint32_t color)
{
    int32_t dx = x1 - x0;
    int32_t dy = y1 - y0;
    int32_t sx = 1;
    int32_t sy = 1;
    float de;
    int32_t i;

    /* Change direction */
    if (0 > dx)
    {
        dx *= -1;
        sx *= -1;
    }

    if (0 > dy)
    {
        dy *= -1;
        sy *= -1;
    }

    draw_point_yuyv(x0, y0, color);

    if (dx > dy)
    {
        /* Horizontal Line */
        for (i = dx, de = i / 2; i; i--)
        {
            x0 += sx;
            de += dy;
            if(de > dx)
            {
                de -= dx;
                y0 += sy;
            }
            draw_point_yuyv(x0, y0, color);
        }
    }
    else
    {
        /* Vertical Line */
        for (i = dy, de = i / 2; i; i--)
        {
            y0 += sy;
            de += dx;
            if(de > dy)
            {
                de -= dy;
                x0 += sx;
            }
            draw_point_yuyv(x0, y0, color);
        }
    }
    return;
}

/*****************************************
* Function Name : draw_rect
* Description   : Draw a rectangle
* Arguments     : x = X coordinate of the center of rectangle
*                 y = Y coordinate of the center of rectangle
*                 w = width of the rectangle
*                 h = height of the rectangle
*                 str = string to label the rectangle
* Return Value  : -
******************************************/
void Image::draw_rect(int32_t x, int32_t y, int32_t w, int32_t h, const char * str,uint32_t color)
{
    int32_t x_min = x - round(w / 2.);
    int32_t y_min = y - round(h / 2.);
    int32_t x_max = x + round(w / 2.) - 1;
    int32_t y_max = y + round(h / 2.) - 1;
    /* Check the bounding box is in the image range */
    x_min = x_min < 1 ? 1 : x_min;
    x_max = (((int32_t)img_w - 2) < x_max) ? ((int32_t)img_w - 2) : x_max;
    y_min = y_min < 1 ? 1 : y_min;
    y_max = (((int32_t)img_h - 2) < y_max) ? ((int32_t)img_h - 2) : y_max;

    /* Draw the bounding box and class and probability*/
    write_string_rgb_boundingbox(str,1,x_min, y_min,x_max,y_max,CHAR_SCALE_FONT,color);

    return;
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

    uint8_t* pd = img_buffer[buf_id];
    uint8_t buffer[img_w * img_h * out_c];
    int pix_count = 0;
    for (uint32_t i = 0; i < img_h * img_w / 2; i++)
    {
        int y0 = (int)pd[0] - 16;
        int u0 = (int)pd[1] - 128;
        int y1 = (int)pd[2] - 16;
        int v0 = (int)pd[3] - 128;

        pd += 4;
        buffer[pix_count++] = Clip((298 * y0 + 516 * u0 + 128) >> 8); // blue
        buffer[pix_count++] = Clip((298 * y0 - 100 * u0 - 208 * v0 + 128) >> 8); // green
        buffer[pix_count++] = Clip((298 * y0 + 409 * v0 + 128) >> 8); // red
        buffer[pix_count++] = 255;

        buffer[pix_count++] = Clip((298 * y1 + 516 * u0 + 128) >> 8); // blue
        buffer[pix_count++] = Clip((298 * y1 - 100 * u0 - 208 * v0 + 128) >> 8); // green
        buffer[pix_count++] = Clip((298 * y1 + 409 * v0 + 128) >> 8); // red
        buffer[pix_count++] = 255;
    }
    memcpy(img_buffer[buf_id], &buffer, img_w * img_h * out_c);

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
*                 is_padding = whether padding or not between resized image resolution and HDMI resolution.
* Return value  : -
******************************************/
void Image::convert_size(int in_w, int resize_w, int in_h, int resize_h, bool is_padding)
{
    // Return if resizing and padding is unnecessary
    if ( in_w == resize_w && in_h == resize_h && !is_padding )
    {
        return;
    }

#ifdef DEBUG_TIME_FLG
    using namespace std;
    chrono::system_clock::time_point start, end;
    start = chrono::system_clock::now();
#endif // DEBUG_TIME_FLG

    cv::Mat org_image(img_h, img_w, CV_8UC4, img_buffer[buf_id]);
    cv::Mat dst_image(img_h, img_w, CV_8UC4, cv::Scalar(0,0,0,255));

#ifdef DISPLAY_CUT_INVALID_EREA
    int disp = (int)((double)img_w / md.pr.width * md.pr.disparity);
    cv::Rect src_roi(disp, 0, img_w - disp, img_h);
    cv::Rect dst_roi(0,    0, img_w - disp, img_h);
    org_image(src_roi).copyTo(dst_image(dst_roi));
#else
    dst_image = org_image;
#endif  

    cv::Mat resize_image;
    cv::Mat padding_image;

    if ( in_w != resize_w && in_h != resize_h )
    {
        /* Use "INTER_NEAREST" because the output data will be resized to twice the original size in both horizontal and vertical directions */
        cv::resize(dst_image, resize_image, cv::Size(resize_w, resize_h), 0, 0, cv::INTER_NEAREST);

        // Update reference (shallow copy)
        dst_image = resize_image;
    }
	
    if ( is_padding )
    {
        uint32_t pad_top = (out_h - resize_h) / 2;
        uint32_t pad_bottom = out_h - resize_h - pad_top;
        uint32_t pad_left = (out_w - resize_w) / 2 + 164;
        uint32_t pad_right = out_w - resize_w - pad_left;
        
        // Pad with as black border
        copyMakeBorder(dst_image, padding_image, pad_top, pad_bottom, pad_left, pad_right, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0, 255));

        // Update reference (shallow copy)
        dst_image = padding_image;
    }

    memcpy(img_buffer[buf_id], dst_image.data, out_w * out_h * out_c);

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
void Image::reset_overlay_img()
{
#ifdef DEBUG_TIME_FLG
    using namespace std;
    chrono::system_clock::time_point start, end;
    start = chrono::system_clock::now();
#endif // DEBUG_TIME_FLG

    cv::Mat src_image = cv::Mat::zeros(out_h, out_w, CV_8UC4);
    uint8_t* dst = overlay_buffer[buf_id];
    uint8_t* src = src_image.data;
    memcpy(dst, src, out_w * out_h * out_c);

#ifdef DEBUG_TIME_FLG
    end = chrono::system_clock::now();
    double time = static_cast<double>(chrono::duration_cast<chrono::microseconds>(end - start).count() / 1000.0);
    printf("Reset Overlay Buffer Time : %lf[ms]\n", time);
#endif // DEBUG_TIME_FLG
}

/*****************************************
* Function Name : camera_to_s_image
* Description   : Function to copy the external image buffer data to img_buffer
*                 This is only place where the buf_id is updated.
* Arguments     : buffer = buffer to copy the image data
*                 size = size of buffer
* Return value  : none
******************************************/
void Image::camera_to_s_image(const uint8_t* buffer, int32_t size)
{
    memcpy(img_s_yuyv_buf, buffer, sizeof(uint8_t)*size);
}

/*****************************************
* Function Name : init_undistort_map
* Description   : 
* Arguments     : -
* Return value  : -
******************************************/
void Image::init_undistort_map(std::string side)
{
    std::string cam_mat_path;
    std::string cam_dis_path;

    cv::Mat mapX;
    cv::Mat mapY;

    std::string str_buf;
    std::string str_conma_buf;

    cam_mat_path = (side == "left") ? md.df.cam_left_calib : md.df.cam_right_calib;
    cam_dis_path = (side == "left") ? md.df.cam_left_dist  : md.df.cam_right_dist;

    std::cout << cam_mat_path.c_str() << std::endl;
    std::cout << cam_dis_path.c_str() << std::endl;

    std::ifstream ifs_mat_file(cam_mat_path);
    std::ifstream ifs_dis_file(cam_dis_path);

    for (int ii = 0; ii < 3; ii++)
    {
        std::getline(ifs_mat_file, str_buf);
        std::istringstream i_stream(str_buf);
        for (int jj = 0; jj < 3; jj++)
        {
            std::getline(i_stream, str_conma_buf, ',');
            cm[ii * 3 + jj] = std::stod(str_conma_buf);
        }
    }

    for (int ii = 0; ii < 1; ii++)
    {
        std::getline(ifs_dis_file, str_buf);
        std::istringstream i_stream(str_buf);
        for (int jj = 0; jj < 5; jj++)
        {
            std::getline(i_stream, str_conma_buf, ',');
            dc[ii * 3 + jj] = std::stod(str_conma_buf);
        }
    }

    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cv::Mat(), cv::Size(img_w, img_h), CV_32FC1, mapX, mapY);
    cv::convertMaps(mapX, mapY, calibration_maps, cv::noArray(), CV_32FC2);
}

/*****************************************
* Function Name : convert_format_s_image
* Description   : Convert YUYV image to BGRA format
* Arguments     : -
* Return value  : -
******************************************/
void Image::convert_format_s_image(int rem_en, int aff_en)
{
    cv::Mat img_s_tmp(CAM_IMAGE_HEIGHT, CAM_IMAGE_WIDTH, CV_8UC1);
    cv::Mat img_s_aff;
    cv::Mat img_s_aff_rec;
  
    cv::Mat img_s_yuyv(CAM_IMAGE_HEIGHT, CAM_IMAGE_WIDTH, CV_8UC2, img_s_yuyv_buf);
    cv::Mat img_s_gray(md.pr.height, md.pr.width, CV_8UC1, img_s_gray_buf);
    cv::cvtColor(img_s_yuyv, img_s_tmp, cv::COLOR_YUV2GRAY_YUYV);

    if(rem_en == 1) {
        cv::remap(img_s_tmp, img_s_tmp, calibration_maps, cv::Mat(), cv::INTER_LINEAR);
    }
    
    if(aff_en == 1) {
        aff_mat = cv::getRotationMatrix2D(aff_ctr, md.pr.angle, 1.0);
        aff_mat.at<double>(0,2) = md.pr.tx;
        aff_mat.at<double>(1,2) = md.pr.ty;
        cv::warpAffine(img_s_tmp, img_s_aff, aff_mat, img_s_tmp.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);

        img_s_aff_rec = cv::Mat(img_s_aff, cv::Rect(0, 0,  CAM_IMAGE_WIDTH, CAM_IMAGE_HEIGHT));
    } else {
        img_s_aff_rec = img_s_tmp;
    }
    
    cv::resize( img_s_aff_rec, img_s_gray, cv::Size(md.pr.width, md.pr.height));
}
/*****************************************
* Function Name : get_s_image
* Description   : Function to return the camera buffer
* Arguments     : -
* Return value  : img_s_gray_buf
******************************************/
uint8_t* Image::get_s_image()
{
    return img_s_gray_buf;
}
/*****************************************
* Function Name : set_stereo_out
* Description   : Function to return the camera buffer
* Arguments     : -
* Return value  : camera buffer
******************************************/
void Image::set_stereo_out(const cv::Mat &input_mat)
{
    cv::Mat disparity_s16;
    cv::Mat disparity_u8;
    cv::Mat disparity_h_tmp;
    cv::Mat disparity_h_res;

    stereo_out_s16 = input_mat.clone() / md.mode_param;
    
    /* for calculate distance */
    cv::resize(stereo_out_s16, disparity_s16_cam, cv::Size(CAM_IMAGE_WIDTH, CAM_IMAGE_HEIGHT));

    /* for display */
    stereo_out_s16.convertTo(disparity_u8, CV_8UC1, 256.0 / (md.pr.disparity));
    cv::applyColorMap(disparity_u8, disparity_h_tmp, md.pr.heat_map);
    cv::resize(disparity_h_tmp, disparity_h_res, cv::Size(STEREO_OUTPUT_WIDTH, STEREO_OUTPUT_HEIGHT));
    cv::cvtColor(disparity_h_res, disparity_h, cv::COLOR_BGR2BGRA);
}
/*****************************************
* Function Name : draw_stereo_out
* Description   : Function to return the camera buffer
* Arguments     : -
* Return value  : camera buffer
 ******************************************/
void Image::draw_stereo_out()
{
    /*OpenCV image data is in BGRA */
    cv::Mat bgra_image(out_h, out_w, CV_8UC4, img_buffer[buf_id]);
    int x = 0;
    int y = out_h - STEREO_OUTPUT_HEIGHT;
    int width = STEREO_OUTPUT_WIDTH;
    int height = STEREO_OUTPUT_HEIGHT;

    cv::Mat roi = bgra_image(cv::Rect(x, y, width, height));
    disparity_h.copyTo(roi);
    
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%+5.1f", md.pr.ty);

    std::string str_cnt;
    str_cnt = (md.pr.focus_en == 0) ? "Manual" : (md.focus_cnt >= md.pr.focus_cnt_max) ? "Done" : std::to_string(md.focus_cnt);

    std::string str =
    "W:" + std::to_string(md.pr.width) +
    ", H:" + std::to_string(md.pr.height) +
    ", D:" + std::to_string(md.pr.disparity) +
    ", y:" + std::string(buf) +
    ", F:" + str_cnt;

    uint8_t thickness = CHAR_THICKNESS;
    cv::putText(bgra_image, str.c_str(), cv::Point(x + 20, y - 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0xFF, 0xFF, 0xFF, 0xFF), thickness + 2);    
}

/*****************************************
* Function Name : adjust_focus
* Description   : Function to return the camera buffer
* Arguments     : -
* Return value  : camera buffer
******************************************/
void Image::adjust_focus()
{
    cv::Mat lap;
    cv::Mat src_8u;
    cv::Scalar mean, stddev;

    if(md.focus_st == 0)
    {
        val_min = 99999;
        ty_min = 0;
        angle_min = 0;
    } else if (md.focus_st == 1) {
        cv::Rect roi(md.pr.disparity, 0, md.pr.width - md.pr.disparity, md.pr.height);
        cv::Mat src_rect = stereo_out_s16(roi);
        //cv::normalize(src_rect, src_8u, 0, 255, cv::NORM_MINMAX, CV_8U);
        //cv::Laplacian(src_8u, lap, CV_64F);
        cv::Laplacian(src_rect, lap, CV_64F);
        cv::meanStdDev(lap, mean, stddev);
        //if(stddev_max < stddev.val[0]) {
        if(val_min > stddev.val[0]) {
            val_min = stddev.val[0];
            ty_min = md.pr.ty;
        }
        md.pr.ty = (double)(md.focus_cnt) * 0.5 - (double)(md.pr.focus_cnt_max) / 4.0;
    } else {
        md.pr.ty = ty_min;
    }

    //std::cout << "[INFO] focus_en : " << focus_en << " tx : " << md.pr.tx << " ty : " << md.pr.ty << " angle : " << md.pr.angle
    //          << ", stddev : " << stddev.val[0] << ", ty_min : " << ty_min << ", angle_min : " << angle_min << ", val_min : " << val_min;
}
