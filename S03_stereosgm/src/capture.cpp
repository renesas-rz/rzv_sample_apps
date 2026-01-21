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
 * File Name    : capture.cpp
 * Version      : v3.10
 * Description  : RZ/V2H AI SDK Sample Application for StereoSGM
 ***********************************************************************************************************************/

/*****************************************
 * Includes
 ******************************************/
#include "capture.h"
Capture::Capture()
{
}

Capture::~Capture()
{
}

/*****************************************
 * Function Name : get_gstreamer
 ******************************************/
std::string Capture::get_gstreamer()
{
    return v4l2src_str;
}

/*****************************************
 * Function Name : get_mat
 ******************************************/
std::string Capture::get_media_ctl(int index)
{
    return media_ctl[index];
}

/*****************************************
 * Function Name : init
 ******************************************/
void Capture::init(std::string side, std::string image, std::string drv, std::string form, uint fps, uint w, uint h, uint c)
{
    /*Initialize input image information */
    img_w = (image == "stereo") ? w * 2 : w;
    img_h = h;
    img_c = c;
    /*Initialize camera information*/
    cam_side = side;
    cam_image = image;
    cam_drv = drv;
    cam_type = (form == "YUYV") ? "video/x-raw" : "image/jpeg";
    cam_form = (form == "YUYV") ? "YUY2" : form;
    cam_jpeg = (form == "YUYV") ? "" : "jpegdec ! ";
    cam_fps = fps;

    ss << "v4l2src device=/dev/" << cam_drv << " ! "
       << cam_type << ", "
       << "format=" << cam_form << ", "
       << "width=" << img_w << ", "
       << "height=" << img_h << ", "
       << "framerate=" << cam_fps << "/1 ! "
       << cam_jpeg
       << "videoconvert ! "
       << "appsink -v";

    v4l2src_str = ss.str();

    // v4l2src_str ="v4l2src device=/dev/video0 ! video/x-raw, format=YUY2, width=640, height=480, framerate=30/1 ! videoconvert ! appsink -v";
    // v4l2src_str ="v4l2src device=/dev/video2 ! video/x-raw, format=YUY2, width=640, height=480, framerate=30/1 ! videoconvert ! appsink -v";

    // Camera test command
    // gst-launch-1.0 -v -e v4l2src device=/dev/video0 ! video/x-raw, format=YUY2, width=640,  height=480, framerate=30/1 !           videoconvert ! waylandsink
    // gst-launch-1.0 -v -e v4l2src device=/dev/video2 ! video/x-raw, format=YUY2, width=640,  height=480, framerate=30/1 !           videoconvert ! waylandsink
    // gst-launch-1.0 -v -e v4l2src device=/dev/video0 ! image/jpeg,  format=MJPG, width=1280, height=720, framerate=30/1 ! jpegdec ! videoconvert ! waylandsink
    // gst-launch-1.0 -v -e v4l2src device=/dev/video2 ! image/jpeg,  format=MJPG, width=1280, height=720, framerate=30/1 ! jpegdec ! videoconvert ! waylandsink
    //
    // gst-launch-1.0 -v -e v4l2src device=/dev/video0 ! video/x-raw, format=YUY2, width=640,  height=480, framerate=30/1 !           videoconvert ! autovideosink
    // gst-launch-1.0 -v -e v4l2src device=/dev/video2 ! video/x-raw, format=YUY2, width=640,  height=480, framerate=30/1 !           videoconvert ! autovideosink
    // gst-launch-1.0 -v -e v4l2src device=/dev/video0 ! image/jpeg,  format=MJPG, width=1280, height=720, framerate=30/1 ! jpegdec ! videoconvert ! autovideosink
    // gst-launch-1.0 -v -e v4l2src device=/dev/video2 ! image/jpeg,  format=MJPG, width=1280, height=720, framerate=30/1 ! jpegdec ! videoconvert ! autovideosink

    if ((img_w == 640) && (cam_image == "ec22"))
    {
        if (cam_side == "left")
        {
            media_ctl[0] = "media-ctl -d /dev/media0 -r";
            media_ctl[1] = "media-ctl -d /dev/media0 -l \"\'csi-16000400.csi20\':1 -> \'cru-ip-16000000.video0\':0 [1]\"";
            media_ctl[2] = "media-ctl -d /dev/media0 -V \"\'csi-16000400.csi20\':1 [fmt:UYVY8_2X8/640x480 field:none]\"";
            media_ctl[3] = "media-ctl -d /dev/media0 -V \"\'imx462 0-001f\':0 [fmt:UYVY8_2X8/640x480 field:none]\"";
            media_ctl[4] = "media-ctl -d /dev/media0 -V \"\'cru-ip-16000000.video0\':0 [fmt:UYVY8_2X8/640x480 field:none]\"";
            media_ctl[5] = "media-ctl -d /dev/media0 -V \"\'cru-ip-16000000.video0\':1 [fmt:UYVY8_2X8/640x480 field:none]\"";
        }
        else
        {
            media_ctl[0] = "media-ctl -d /dev/media1 -r";
            media_ctl[1] = "media-ctl -d /dev/media1 -l \"\'csi-16010400.csi21\':1 -> \'cru-ip-16010000.video1\':0 [1]\"";
            media_ctl[2] = "media-ctl -d /dev/media1 -V \"\'csi-16010400.csi21\':1 [fmt:UYVY8_2X8/640x480 field:none]\"";
            media_ctl[3] = "media-ctl -d /dev/media1 -V \"\'imx462 1-001f\':0 [fmt:UYVY8_2X8/640x480 field:none]\"";
            media_ctl[4] = "media-ctl -d /dev/media1 -V \"\'cru-ip-16010000.video1\':0 [fmt:UYVY8_2X8/640x480 field:none]\"";
            media_ctl[5] = "media-ctl -d /dev/media1 -V \"\'cru-ip-16010000.video1\':1 [fmt:UYVY8_2X8/640x480 field:none]\"";
        }
    }
    else if ((img_w == 1280) && (cam_image == "ec22"))
    {
        if (cam_side == "left")
        {
            media_ctl[0] = "media-ctl -d /dev/media0 -r";
            media_ctl[1] = "media-ctl -d /dev/media0 -l \"\'csi-16000400.csi20\':1 -> \'cru-ip-16000000.video0\':0 [1]\"";
            media_ctl[2] = "media-ctl -d /dev/media0 -V \"\'csi-16000400.csi20\':1 [fmt:UYVY8_2X8/1280x720 field:none]\"";
            media_ctl[3] = "media-ctl -d /dev/media0 -V \"\'imx462 0-001f\':0 [fmt:UYVY8_2X8/1280x720 field:none]\"";
            media_ctl[4] = "media-ctl -d /dev/media0 -V \"\'cru-ip-16000000.video0\':0 [fmt:UYVY8_2X8/1280x720 field:none]\"";
            media_ctl[5] = "media-ctl -d /dev/media0 -V \"\'cru-ip-16000000.video0\':1 [fmt:UYVY8_2X8/1280x720 field:none]\"";
        }
        else
        {
            media_ctl[0] = "media-ctl -d /dev/media1 -r";
            media_ctl[1] = "media-ctl -d /dev/media1 -l \"\'csi-16010400.csi21\':1 -> \'cru-ip-16010000.video1\':0 [1]\"";
            media_ctl[2] = "media-ctl -d /dev/media1 -V \"\'csi-16010400.csi21\':1 [fmt:UYVY8_2X8/1280x720 field:none]\"";
            media_ctl[3] = "media-ctl -d /dev/media1 -V \"\'imx462 1-001f\':0 [fmt:UYVY8_2X8/1280x720 field:none]\"";
            media_ctl[4] = "media-ctl -d /dev/media1 -V \"\'cru-ip-16010000.video1\':0 [fmt:UYVY8_2X8/1280x720 field:none]\"";
            media_ctl[5] = "media-ctl -d /dev/media1 -V \"\'cru-ip-16010000.video1\':1 [fmt:UYVY8_2X8/1280x720 field:none]\"";
        }
    }
    else if ((img_w == 1920) && (cam_image == "ec22"))
    {
        if (cam_side == "left")
        {
            media_ctl[0] = "media-ctl -d /dev/media0 -r";
            media_ctl[1] = "media-ctl -d /dev/media0 -l \"\'csi-16000400.csi20\':1 -> \'cru-ip-16000000.video0\':0 [1]\"";
            media_ctl[2] = "media-ctl -d /dev/media0 -V \"\'csi-16000400.csi20\':1 [fmt:UYVY8_2X8/1920x1080 field:none]\"";
            media_ctl[3] = "media-ctl -d /dev/media0 -V \"\'imx462 0-001f\':0 [fmt:UYVY8_2X8/1920x1080 field:none]\"";
            media_ctl[4] = "media-ctl -d /dev/media0 -V \"\'cru-ip-16000000.video0\':0 [fmt:UYVY8_2X8/1920x1080 field:none]\"";
            media_ctl[5] = "media-ctl -d /dev/media0 -V \"\'cru-ip-16000000.video0\':1 [fmt:UYVY8_2X8/1920x1080 field:none]\"";
        }
        else
        {
            media_ctl[0] = "media-ctl -d /dev/media1 -r";
            media_ctl[1] = "media-ctl -d /dev/media1 -l \"\'csi-16010400.csi21\':1 -> \'cru-ip-16010000.video1\':0 [1]\"";
            media_ctl[2] = "media-ctl -d /dev/media1 -V \"\'csi-16010400.csi21\':1 [fmt:UYVY8_2X8/1920x1080 field:none]\"";
            media_ctl[3] = "media-ctl -d /dev/media1 -V \"\'imx462 1-001f\':0 [fmt:UYVY8_2X8/1920x1080 field:none]\"";
            media_ctl[4] = "media-ctl -d /dev/media1 -V \"\'cru-ip-16010000.video1\':0 [fmt:UYVY8_2X8/1920x1080 field:none]\"";
            media_ctl[5] = "media-ctl -d /dev/media1 -V \"\'cru-ip-16010000.video1\':1 [fmt:UYVY8_2X8/1920x1080 field:none]\"";
        }
    }
    else if ((img_w == 640) && (cam_image == "ec25"))
    {
        if (cam_side == "left")
        {
            media_ctl[0] = "media-ctl -d /dev/media0 -r";
            media_ctl[1] = "media-ctl -d /dev/media0 -l \"\'csi-16000400.csi20\':1 -> \'cru-ip-16000000.video0\':0 [1]\"";
            media_ctl[2] = "media-ctl -d /dev/media0 -V \"\'csi-16000400.csi20\':1 [fmt:UYVY8_2X8/640x480 field:none]\"";
            media_ctl[3] = "media-ctl -d /dev/media0 -V \"\'ar0234 0-0042\':0 [fmt:UYVY8_2X8/640x480 field:none]\"";
            media_ctl[4] = "media-ctl -d /dev/media0 -V \"\'cru-ip-16000000.video0\':0 [fmt:UYVY8_2X8/640x480 field:none]\"";
            media_ctl[5] = "media-ctl -d /dev/media0 -V \"\'cru-ip-16000000.video0\':1 [fmt:UYVY8_2X8/640x480 field:none]\"";
        }
        else
        {
            media_ctl[0] = "media-ctl -d /dev/media1 -r";
            media_ctl[1] = "media-ctl -d /dev/media1 -l \"\'csi-16010400.csi21\':1 -> \'cru-ip-16000000.video1\':0 [1]\"";
            media_ctl[2] = "media-ctl -d /dev/media1 -V \"\'csi-16010400.csi21\':1 [fmt:UYVY8_2X8/640x480 field:none]\"";
            media_ctl[3] = "media-ctl -d /dev/media1 -V \"\'ar0234 1-0042\':0 [fmt:UYVY8_2X8/640x480 field:none]\"";
            media_ctl[4] = "media-ctl -d /dev/media1 -V \"\'cru-ip-16010000.video1\':0 [fmt:UYVY8_2X8/640x480 field:none]\"";
            media_ctl[5] = "media-ctl -d /dev/media1 -V \"\'cru-ip-16010000.video1\':1 [fmt:UYVY8_2X8/640x480 field:none]\"";
        }
    }
    else if ((img_w == 1280) && (cam_image == "ec25"))
    {
        if (cam_side == "left")
        {
            media_ctl[0] = "media-ctl -d /dev/media0 -r";
            media_ctl[1] = "media-ctl -d /dev/media0 -l \"\'csi-16000400.csi20\':1 -> \'cru-ip-16000000.video0\':0 [1]\"";
            media_ctl[2] = "media-ctl -d /dev/media0 -V \"\'csi-16000400.csi20\':1 [fmt:UYVY8_2X8/1280x720 field:none]\"";
            media_ctl[3] = "media-ctl -d /dev/media0 -V \"\'ar0234 0-0042\':0 [fmt:UYVY8_2X8/1280x720 field:none]\"";
            media_ctl[4] = "media-ctl -d /dev/media0 -V \"\'cru-ip-16000000.video0\':0 [fmt:UYVY8_2X8/1280x720 field:none]\"";
            media_ctl[5] = "media-ctl -d /dev/media0 -V \"\'cru-ip-16000000.video0\':1 [fmt:UYVY8_2X8/1280x720 field:none]\"";
        }
        else
        {
            media_ctl[0] = "media-ctl -d /dev/media1 -r";
            media_ctl[1] = "media-ctl -d /dev/media1 -l \"\'csi-16010400.csi20\':1 -> \'cru-ip-16010000.video1\':0 [1]\"";
            media_ctl[2] = "media-ctl -d /dev/media1 -V \"\'csi-16010400.csi20\':1 [fmt:UYVY8_2X8/1280x720 field:none]\"";
            media_ctl[3] = "media-ctl -d /dev/media1 -V \"\'ar0234 1-0042\':0 [fmt:UYVY8_2X8/1280x720 field:none]\"";
            media_ctl[4] = "media-ctl -d /dev/media1 -V \"\'cru-ip-16010000.video1\':0 [fmt:UYVY8_2X8/1280x720 field:none]\"";
            media_ctl[5] = "media-ctl -d /dev/media1 -V \"\'cru-ip-16010000.video1\':1 [fmt:UYVY8_2X8/1280x720 field:none]\"";
        }
    }
    else if ((img_w == 1920) && (cam_image == "ec25"))
    {
        if (cam_side == "left")
        {
            media_ctl[0] = "media-ctl -d /dev/media0 -r";
            media_ctl[1] = "media-ctl -d /dev/media0 -l \"\'csi-16000400.csi20':1 -> \'cru-ip-16000000.video0\':0 [1]\"";
            media_ctl[2] = "media-ctl -d /dev/media0 -V \"\'csi-16000400.csi20':1 [fmt:UYVY8_2X8/1920x1080 field:none]\"";
            media_ctl[3] = "media-ctl -d /dev/media0 -V \"\'ar0234 0-0042\':0 [fmt:UYVY8_2X8/1920x1080 field:none]\"";
            media_ctl[4] = "media-ctl -d /dev/media0 -V \"\'cru-ip-16000000.video0\':0 [fmt:UYVY8_2X8/1920x1080 field:none]\"";
            media_ctl[5] = "media-ctl -d /dev/media0 -V \"\'cru-ip-16000000.video0\':1 [fmt:UYVY8_2X8/1920x1080 field:none]\"";
        }
        else
        {
            media_ctl[0] = "media-ctl -d /dev/media1 -r";
            media_ctl[1] = "media-ctl -d /dev/media1 -l \"\'csi-16010400.csi21':1 -> \'cru-ip-16010000.video1\':0 [1]\"";
            media_ctl[2] = "media-ctl -d /dev/media1 -V \"\'csi-16010400.csi21':1 [fmt:UYVY8_2X8/1920x1080 field:none]\"";
            media_ctl[3] = "media-ctl -d /dev/media1 -V \"\'ar0234 1-0042\':0 [fmt:UYVY8_2X8/1920x1080 field:none]\"";
            media_ctl[4] = "media-ctl -d /dev/media1 -V \"\'cru-ip-16010000.video1\':0 [fmt:UYVY8_2X8/1920x1080 field:none]\"";
            media_ctl[5] = "media-ctl -d /dev/media1 -V \"\'cru-ip-16010000.video1\':1 [fmt:UYVY8_2X8/1920x1080 field:none]\"";
        }
    }
    else
    {
        media_ctl[0] = "";
        media_ctl[1] = "";
        media_ctl[2] = "";
        media_ctl[3] = "";
        media_ctl[4] = "";
        media_ctl[5] = "";
    }
}
