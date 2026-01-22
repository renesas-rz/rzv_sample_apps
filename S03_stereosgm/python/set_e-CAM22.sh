#!/bin/bash
#
#<<< GStreamer command line >>>
cam_side="${1}"
cam_width="${2}"
cam_height="${3}"

echo "e-CAM22" $cam_side $cam_width $cam_height

#v4l2-ctl -d /dev/video0 --set-ctrl horizontal_flip=0
#v4l2-ctl -d /dev/video0 --set-ctrl vertical_flip=1
#v4l2-ctl -d /dev/video1 --set-ctrl horizontal_flip=0
#v4l2-ctl -d /dev/video1 --set-ctrl vertical_flip=1

if [ $cam_width = '640' ]; then
#media-ctl -d /dev/media0 -r
#media-ctl -d /dev/media0 -l "'rzg2l_csi2 16000400.csi20':1 -> 'CRU output':0 [1]"
#media-ctl -d /dev/media0 -V "'rzg2l_csi2 16000400.csi20':1 [fmt:UYVY8_2X8/640x480 field:none]"
#media-ctl -d /dev/media0 -V "'imx462 0-001f':0 [fmt:UYVY8_2X8/640x480 field:none]"
media-ctl -d /dev/media0 -r
media-ctl -d /dev/media0 -l "'csi-16000400.csi20':1 -> 'cru-ip-16000000.video0':0 [1]"
media-ctl -d /dev/media0 -V "'csi-16000400.csi20':1 [fmt:UYVY8_2X8/640x480 field:none]"
media-ctl -d /dev/media0 -V "'imx462 0-001f':0 [fmt:UYVY8_2X8/640x480 field:none]"
media-ctl -d /dev/media0 -V "'cru-ip-16000000.video0':0 [fmt:UYVY8_2X8/640x480 field:none]"
media-ctl -d /dev/media0 -V "'cru-ip-16000000.video0':1 [fmt:UYVY8_2X8/640x480 field:none]"
#gst-launch-1.0 -v -e v4l2src device=/dev/video0 ! video/x-raw, format=YUY2, width=640, height=480, framerate=30/1 ! videoconvert ! waylandsink
#-----------------------------------------------------------------------------------------------------------------------------------
#media-ctl -d /dev/media1 -r
#media-ctl -d /dev/media1 -l "'rzg2l_csi2 16010400.csi21':1 -> 'CRU output':0 [1]"
#media-ctl -d /dev/media1 -V "'rzg2l_csi2 16010400.csi21':1 [fmt:UYVY8_2X8/640x480 field:none]"
#media-ctl -d /dev/media1 -V "'imx462 1-001f':0 [fmt:UYVY8_2X8/640x480 field:none]"
media-ctl -d /dev/media1 -r
media-ctl -d /dev/media1 -l "'csi-16010400.csi21':1 -> 'cru-ip-16010000.video1':0 [1]"
media-ctl -d /dev/media1 -V "'csi-16010400.csi21':1 [fmt:UYVY8_2X8/640x480 field:none]"
media-ctl -d /dev/media1 -V "'imx462 1-001f':0 [fmt:UYVY8_2X8/640x480 field:none]"
media-ctl -d /dev/media1 -V "'cru-ip-16010000.video1':0 [fmt:UYVY8_2X8/640x480 field:none]"
media-ctl -d /dev/media1 -V "'cru-ip-16010000.video1':1 [fmt:UYVY8_2X8/640x480 field:none]"
#gst-launch-1.0 -v -e v4l2src device=/dev/video1 ! video/x-raw, format=YUY2, width=640, height=480, framerate=30/1 ! videoconvert ! waylandsink
#-----------------------------------------------------------------------------------------------------------------------------------
elif [ $cam_width = '1280' ]; then
#media-ctl -d /dev/media0 -r
#media-ctl -d /dev/media0 -l "'rzg2l_csi2 16000400.csi20':1 -> 'CRU output':0 [1]"
#media-ctl -d /dev/media0 -V "'rzg2l_csi2 16000400.csi20':1 [fmt:UYVY8_2X8/1280x720 field:none]"
#media-ctl -d /dev/media0 -V "'imx462 0-001f':0 [fmt:UYVY8_2X8/1280x720 field:none]"
media-ctl -d /dev/media0 -r
media-ctl -d /dev/media0 -l "'csi-16000400.csi20':1 -> 'cru-ip-16000000.video0':0 [1]"
media-ctl -d /dev/media0 -V "'csi-16000400.csi20':1 [fmt:UYVY8_2X8/1280x720 field:none]"
media-ctl -d /dev/media0 -V "'imx462 0-001f':0 [fmt:UYVY8_2X8/1280x720 field:none]"
media-ctl -d /dev/media0 -V "'cru-ip-16000000.video0':0 [fmt:UYVY8_2X8/1280x720 field:none]"
media-ctl -d /dev/media0 -V "'cru-ip-16000000.video0':1 [fmt:UYVY8_2X8/1280x720 field:none]"
#gst-launch-1.0 -v -e v4l2src device=/dev/video0 ! video/x-raw, format=YUY2, width=1280, height=720, framerate=30/1 ! videoconvert ! waylandsink
#-----------------------------------------------------------------------------------------------------------------------------------
#media-ctl -d /dev/media1 -r
#media-ctl -d /dev/media1 -l "'rzg2l_csi2 16010400.csi21':1 -> 'CRU output':0 [1]"
#media-ctl -d /dev/media1 -V "'rzg2l_csi2 16010400.csi21':1 [fmt:UYVY8_2X8/1280x720 field:none]"
#media-ctl -d /dev/media1 -V "'imx462 1-001f':0 [fmt:UYVY8_2X8/1280x720 field:none]"
media-ctl -d /dev/media1 -r
media-ctl -d /dev/media1 -l "'csi-16010400.csi21':1 -> 'cru-ip-16010000.video1':0 [1]"
media-ctl -d /dev/media1 -V "'csi-16010400.csi21':1 [fmt:UYVY8_2X8/1280x720 field:none]"
media-ctl -d /dev/media1 -V "'imx462 1-001f':0 [fmt:UYVY8_2X8/1280x720 field:none]"
media-ctl -d /dev/media1 -V "'cru-ip-16010000.video1':0 [fmt:UYVY8_2X8/1280x720 field:none]"
media-ctl -d /dev/media1 -V "'cru-ip-16010000.video1':1 [fmt:UYVY8_2X8/1280x720 field:none]"
#gst-launch-1.0 -v -e v4l2src device=/dev/video1 ! video/x-raw, format=YUY2, width=1280, height=720, framerate=30/1 ! videoconvert ! waylandsink
#-----------------------------------------------------------------------------------------------------------------------------------
elif [ $cam_width = '1920' ]; then
#media-ctl -d /dev/media0 -r
#media-ctl -d /dev/media0 -l "'rzg2l_csi2 16000400.csi20':1 -> 'CRU output':0 [1]"
#media-ctl -d /dev/media0 -V "'rzg2l_csi2 16000400.csi20':1 [fmt:UYVY8_2X8/1920x1080 field:none]"
#media-ctl -d /dev/media0 -V "'imx462 0-001f':0 [fmt:UYVY8_2X8/1920x1080 field:none]"
media-ctl -d /dev/media0 -r
media-ctl -d /dev/media0 -l "'csi-16000400.csi20':1 -> 'cru-ip-16000000.video0':0 [1]"
media-ctl -d /dev/media0 -V "'csi-16000400.csi20':1 [fmt:UYVY8_2X8/1920x1080 field:none]"
media-ctl -d /dev/media0 -V "'imx462 0-001f':0 [fmt:UYVY8_2X8/1920x1080 field:none]"
media-ctl -d /dev/media0 -V "'cru-ip-16000000.video0':0 [fmt:UYVY8_2X8/1920x1080 field:none]"
media-ctl -d /dev/media0 -V "'cru-ip-16000000.video0':1 [fmt:UYVY8_2X8/1920x1080 field:none]"
#gst-launch-1.0 -v -e v4l2src device=/dev/video0 ! video/x-raw, format=YUY2, width=1920, height=1080, framerate=30/1 ! videoconvert ! waylandsink
#-----------------------------------------------------------------------------------------------------------------------------------
#media-ctl -d /dev/media1 -r
#media-ctl -d /dev/media1 -l "'rzg2l_csi2 16010400.csi21':1 -> 'CRU output':0 [1]"
#media-ctl -d /dev/media1 -V "'rzg2l_csi2 16010400.csi21':1 [fmt:UYVY8_2X8/1920x1080 field:none]"
#media-ctl -d /dev/media1 -V "'imx462 1-001f':0 [fmt:UYVY8_2X8/1920x1080 field:none]"
media-ctl -d /dev/media1 -r
media-ctl -d /dev/media1 -l "'csi-16010400.csi21':1 -> 'cru-ip-16010000.video1':0 [1]"
media-ctl -d /dev/media1 -V "'csi-16010400.csi21':1 [fmt:UYVY8_2X8/1920x1080 field:none]"
media-ctl -d /dev/media1 -V "'imx462 1-001f':0 [fmt:UYVY8_2X8/1920x1080 field:none]"
media-ctl -d /dev/media1 -V "'cru-ip-16010000.video1':0 [fmt:UYVY8_2X8/1920x1080 field:none]"
media-ctl -d /dev/media1 -V "'cru-ip-16010000.video1':1 [fmt:UYVY8_2X8/1920x1080 field:none]"
#gst-launch-1.0 -v -e v4l2src device=/dev/video1 ! video/x-raw, format=YUY2, width=1920, height=1080, framerate=30/1 ! videoconvert ! waylandsink
#-----------------------------------------------------------------------------------------------------------------------------------
else
echo "error size"    
fi
