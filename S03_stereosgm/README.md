# StereoSGM

## Table of Contents

[Application: Overview](#application-overview)  
[Application: specification](#application-specification)  
[Application: Process Flow and video](#application-process-flow-and-video)  
[Application: Requirements](#application-requirements)  
[Application: Build Stage](#application-build-stage)  
[Application: Deploy Stage](#application-deploy-stage)  
[Application: Run Stage](#application-run-stage)  
[Application: Tuning the image position](#application-tuning-the-image-position)  
[Application: Execution Mode](#application-execution-mode)  
[Application: Comparison of parameters between DRP Library and CV::StereoSGBM](#application-comparison-of-parameters-between-drp-library-and-cvstereosgbm)  
[Implementation using DRP for StereoSGM](#implementation-using-drp-for-stereosgm)  
[Explanation of Sample software](#explanation-of-sample-software)  
[Example for setting parameters and camera](#example-for-setting-parameters-and-camera)  
[Calibraiton Method](#calibraiton-method)  
[License](#license)  

## Application: Overview

This is a sample stereo vision application of RZ/V2H accelerating with **OpenCV Accelerator using DRP lib**. The algorithm of stereo vision processing is based on StereoSGM(the modified H. Hirschmuller algorithm). The application calculates disparity(D) from two images and display the result as heat map. User can estimate distance(Z) from cameras with disparity(D), Baseline(B) and Focal length(F)

### The principle of stereo camera

<img src=./img/principle.png width=600 alt="The principle of stereo camera" style="border: 3px solid grey;">

## Application: specification

|Item|Details|
|:---|:---|
|HW Platform|RZ/V2H Evaluation Board Kit|
|AI SDK version|RZ/V2H AI SDK v6.00|
|Stereo algorithm|StereoSGM (the modified H. Hirschmuller algorithm [[129]](https://docs.opencv.org/4.x/d0/de3/citelist.html#CITEREF_hh08))<br> The algorithm uses census transforms and 4 directions Dynamic Programming.|
|Target distance from camera <br> (For example)|Baseline(B), Distance(Z) <br> B:163mm, D:1m\~10m <br> B:40mm, D:0.2m\~2m|
|Input|Two camara(USB or MIPI) or sample data|
|Output|HDMI|
|Calculation image size|Resolution up to FHD(selectable)|
|Output data|Disparity(D) and heatmap image|
|Calibration|Calibrate camera and output parameters for remap|

## Application: Process Flow and video

### Process Flow

<img src=./img/sample_soft_flow.png width=800 alt="The principle of stereo camera" style="border: 3px solid grey;">

```text
The sample software is composed of the following six threads.
    (1) capture thread : Capture the image from camera.
    (2) imageL thread : Preprocess the left image.
    (3) imageR thread : Preprocess the right image.
    (4) stereo thread : Stereo processing selectable OCA or CPU. 
    (5) display thread : Postprocess and display the disparity.
    (6) kbhit thread : Wait for keyboard input, then either exit the application or adjust the image.
Stereo processing can be selected from OCA or CPU.
Show the Disparity and the execution time on the display.
```

### In case of OCA running stereo process (without remap, filter2D, LUT)

Because DRP is used for stereo processing, the load on the four CPUs remains low.

<!--
<video src=./img_mp4/stereo_oca_crop_resize.mp4 width=800 alt="stereo oca" controls="true"></video>
-->

<video src="https://github.com/user-attachments/assets/113899c69-b42b-4d93-aabb-b723d9ee5223" width="800" alt="stereo oca" controls="true"></video>

### In case of CPU running stereo process (without remap, filter2D, LUT)

Because CPU is used for stereo processing, the load on the four CPUs remains high.

<!--
<video src=./img_mp4/stereo_cpu_crop_resize.mp4 width=800 alt="stereo cpu" controls="true" style="border: 3px solid grey;"></video>
-->

<video src="https://github.com/user-attachments/assets/17a1f3b57-9a0c-45d4-9f6c-4dd2d3b522a0" width="800" alt="stereo cpu" controls="true" style="border: 3px solid grey;"></video>

## Application: Requirements

### Hardware Requirements

|Equipment|Details|
|:---|:---|
|RZ/V2H EVK|Evaluation Board Kit for RZ/V2H.|
|AC Adapter|USB Power Delivery adapter for the board power supply. <br> 100W is required.|
|HDMI Cable|Used to connect the HDMI Monitor and the board. <br> RZ/V2H EVK has HDMI port.|
|USB or MIPI Camera|Used as two cameras input source. (MIPI:e-CAM22 or e-CAM25)|
|USB Cable Type-C|Connect AC adapter and the board.|
|HDMI Monitor|Used to display the graphics of the board.|
|microSD card|Used as the filesystem. <br> Must have over 16GB capacity of blank space. <br> Operating Environment: Transcend UHS-I microSD 300S 16GB|
|Linux PC|Used to build application and setup microSD card. <br> Operating Environment: Ubuntu 20.04|
|SD card reader|Used for setting up microSD card.|
|USB Hub|sed to connect USB Keyboard and USB Mouse to the board.|
|USB Keyboard|Used to type strings on the terminal of board.|
|USB Mouse|Used to operate the mouse on the screen of board.|

>**Note:** All external devices will be attached to the board and does not require any driver installation (Plug n Play Type)

### Connection of the hardware

### RZ/V2H EVK

<img src=./img/evk_board.png width=600>

>**Note 1:** When using the keyboard connected to RZ/V Evaluation Board, the keyboard layout and language are fixed to English.  
**Note 2:** For RZ/V2H EVK, there are USB 2.0 and USB 3.0 ports.  
USB camera needs to be connected to appropriate port based on its requirement.

## Application: Build Stage

>**Note:** User can skip to the [next stage (deploy)](#application-deploy-stage) if they do not want to build the application.  
All pre-built binaries are provided.

### Prerequisites

This section expects the user to have completed Step 5 of [Getting Started Guide](https://renesas-rz.github.io/rzv_ai_sdk/latest/getting_started.html#step5) provided by Renesas.

After completion of the guide, the user is expected of following things.

- AI SDK setup is done.
- Following docker container is running on the host machine.

    |Board|Docker container|
    |:---|:---|
    |RZ/V2H EVK|`rzv2h_ai_sdk_container`|

    >**Note 1:** Docker environment is required for building the sample application.  

|When using USB, e-CAM22 camera|When using e-CAM22, e-CAM25 camera|
|:---:|:---:|
|<img src=./img/cross_compile_env.png width=470 alt="The principle of stereo camera" style="border: 3px solid grey;">|<img src=./img/cross_compile_env_ecam.png width=500 alt="The principle of stereo camera" style="border: 3px solid grey;">|

### Application File Generation

1. On your host machine, copy the repository from the GitHub to the desired location.
It is recommended to copy/clone the repository on the `data` folder, which is mounted on the Docker container.

    ```sh
    cd <path_to_data_folder_on_host>/data
    git clone https://github.com/renesas-rz/rzv_sample_apps.git
    ```

    >Note: This command will download the whole repository, which include all other applications.  
    If you have already downloaded the repository of the same version, you may not need to run this command.  

2. Run (or start) the docker container and open the bash terminal on the container.  
E.g., for RZ/V2H, use the `rzv2h_ai_sdk_container` as the name of container created from  `rzv2h_ai_sdk_image` docker image.  
    > Note that all the build steps/commands listed below are executed on the docker container bash terminal.  

3. Set your clone directory to the environment variable.  

    ```sh
    export PROJECT_PATH=/drp-ai_tvm/data/rzv_sample_apps
    ```

4. Go to the application source code directory.

    ```sh
    cd ${PROJECT_PATH}/S03_stereosgm/src_v2h
    ```

5. Create and move to the `build` directory.

    ```sh
    mkdir -p build && cd build
    ```

6. Build the application by following the commands below.
  
    ```sh
    cmake -DCMAKE_TOOLCHAIN_FILE=./toolchain/runtime.cmake ..
    make -j$(nproc)
    ```

7. The following application file would be generated in the `${PROJECT_PATH}/S03_stereosgm/src/build` directory
    ```sh
    stereo_app
    ```

## Application: Deploy Stage

### Prerequisites

This section expects the user to have completed Step 7-1 of [Getting Started Guide](https://renesas-rz.github.io/rzv_ai_sdk/latest/getting_started.html#step7) provided by Renesas.

After completion of the guide, the user is expected of following things.

- microSD card setup is done.

### File Configuration

For the ease of deployment all the deployable files and folders are provided in following folders.  

|Board | `EXE_DIR` |
|:---|:---|
|RZ/V2H EVK|[exe_v2h](./exe_v2h)  |
  
The folder contains following items.

|File/Folder|Details|
|:---|:---|
|licenses|License information. <br> Not necessary for running application.|
|calib_data|camera calibration data|
|stereo_gamma_v1.0.csv|gamma data|
|stereo_app|application file.|

### Instruction

1. Register the working directory path to an environment variable. <br>
The environment variable WORK is the working directory path that you set in Step 4-2 of [Getting Started Guide](https://renesas-rz.github.io/rzv_ai_sdk/latest/getting_started.html#step4).

    ```sh
    export WORK=<path to the working directory>/ai_sdk_work
    export APPS_PATH=${WORK}/ai_sdk_setup/data/rzv_sample_apps
    ```

2. Run following commands to download the necessary file.  
Replace each variable according to your board.  

    ```sh
    cd ${APPS_PATH}/S03_stereosgm/<EXE_DIR>
    sudo wget <URL>/<SO_FILE>
    ```

    |Board|`EXE_DIR`|`URL`|`Dataset`|File Location|
    |:---|:---|:---|:---|:---|
    |RZ/V2H EVK|[exe_v2h](./exe_v2h)|<span style="font-size:small">`https://github.com/renesas-rz/rzv_sample_apps/releases/download/v1.20/`</span>|<span style="font-size: small">`drivingstereo_small_ver1.00.tar.gz`</span> <br> <span style="font-size:small">`images_stereo_small_ver1.00.tar.gz`</span>|[Release v1.20](https://github.com/renesas-rz/rzv_sample_apps/releases/tag/v1.20/)|

    - E.g., use following commands.

    ```sh
    cd ${APPS_PATH}/S03_stereosgm/exe_v2h/
    sudo wget https://github.com/renesas-rz/rzv_sample_apps/releases/download/v1.20/drivingstereo_small_ver1.00.tar.gz
    sudo wget https://github.com/renesas-rz/rzv_sample_apps/releases/download/v1.20/images_stereo_small_ver1.00.tar.gz
    ```

3. Extract Dataset `drivingstereo_small_ver1.00.tar.gz` and `images_stereo_small_ver1.00.tar.gz`.

    ```sh
    tar -zxvf drivingstereo_small_ver1.00.tar.gz
    tar -zxvf images_stereo_small_ver1.00.tar.gz
    ```

4. Copy the following files to the `/home/weston/exe` directory of the rootfs (SD Card) for the board.

    |File| Details|
    |:---|:---|
    |`stereo_app` application file | Generated the file according to [Application File Generation](#application-file-generation)|

5. Folder structure in the rootfs (SD Card) is shown below.

    ```
    |-- home/
        `-- weston/
            `-- exe_v2h/ 
                |-- calib_data
                |-- stereo_gamma_v1.0.csv
                |-- stereo_app
                |-- driving_stereo
                `-- images_stereo
    ```

>**Note:** The directory name could be anything instead of `exe`. If you copy the whole `EXE_DIR` folder on the board, you are not required to rename it `exe`.

## Application: Run Stage

### Prerequisites

This section expects the user to have completed Step 7-3 of [Getting Started Guide](https://renesas-rz.github.io/rzv_ai_sdk/latest/getting_started.html#step7) provided by Renesas.

After completion of the guide, the user is expected of following things.  

- The board setup is done.  
- The board is booted with microSD card, which contains the application file.  

### Instruction

<img src=./img/terminal_1.png width=288 alt="The principle of stereo camera" style="border: 3px solid grey;">
<img src=./img/terminal_2.png width=300 alt="The principle of stereo camera" style="border: 3px solid grey;">

1. On Board terminal, go to the `weston` directory of the rootfs.

    ```sh
    cd /home/weston
    su
    ```

   >**Note :** For AI SDK v6.00 and later, you need to switch to the root user with the 'su' command when running an application.<br>
    This is because when you run an application from a weston-terminal, you are switched to the "weston" user, which does not have permission to run the /dev/xxx device used in the application.

2. Run the application.

    #### (1) Default mode

    ```sh
    ./stereo_app <mode>

        For example
        ./stereo_app oca
        ./stereo_app cpu
    ```

    #### (2) Simple parameter mode

    ```sh
    ./stereo_app <mode> <Item_Num>

        For example
        ./stereo_app oca 30
        ./stereo_app cpu 30
    ```

    #### (3) Detail parameter mode

    ```sh
    ./stereo_app <mode> <image> <format> <fps> <left> <right> <size> <p1> <p2> <dep> <win_x> <win_y> <remap> <uniquenessRatio> <gamma> <sharp> <heatmap> 

        For example
        ./stereo_app oca data file 0 drivingstereo/2018-07-11-14-48-52-L-811 drivingstereo/2018-07-11-14-48-52-R-811 9 10 30 64 9 9 0 0 0 0 20
        ./stereo_app oca usb YUYV 30 video0 video2 0 10 30 64 9 9 0 0 0 0 20
    ```

3. Following window shows up on HDMI screen.  

    ```sh
    ./stereo_app oca
    ```

    <img src=./img/terminal_3.png width=195 alt="The principle of stereo camera" style="border: 3px solid grey;">

    Move the window by mouse.

    <img src=./img/terminal_4.png width=500 alt="The principle of stereo camera" style="border: 3px solid grey;">


    On application window, following information is displayed.  

    |ITEM|UNIT|COMMENT|
    |:---:|:---:|:---|
    |CAPTURE|ms|capture thread time (camera frame time)|
    |IMAGE_L|ms|imageL thread time|
    |IMAGE_R|ms|imageR thread time|
    |STEREO|ms|stereo thread time|
    |DISPLAY|ms|display thread time|
    |FRAME|ms|Frame time (1cycle time of process)|
    |TEMP|°C|temparature|

    <img src=./img/sample_soft_time.png width=450 alt="The principle of stereo camera" style="border: 3px solid grey;">

## Application: Tuning the image position

During software operation, keyboard input enables vertical translation and rotation of the left image, allowing fine adjustments. 

```text
Press ENTER key to quit.

Press "w" and ENTER key to up the image.<br>
Press "x" and ENTER key to down the image.<br>
Press "s" and ENTER key to rotate the image clockwise.<br>
Press "a" and ENTER key to rotate the image counterclockwise.<br>
```

|KEY|Translation and Rotation|
|:---:|:---:|
|<img src=./img/keyboard.png width=224 alt="The principle of stereo camera" style="border: 3px solid grey;">|<img src=./img/trans_rot_key.png width=240 alt="The principle of stereo camera" style="border: 3px solid grey;">|

<!--
<video src=./img_mp4/trans_rot_image_short_resize.mp4 width=500 alt="stereo cpu" controls="true" style="border: 3px solid grey;"></video>
-->

<video src="https://github.com/user-attachments/assets/db01a695-5e75-4603-b793-1828e5410746" width="500" alt="stereo cpu" controls="true" style="border: 3px solid grey;"></video>

## Application: Execution Mode

The sample software starts in one of modes (1) to (3) depending on the arguments entered at startup.

```text
(1) Default parameter mode
(2) Simple parameter mode
(3) Detail parameter mode
```

### (1) Default parameter mode

When starting the sample software, enter only \<mode\> as the argument. In this case, the parameter table is set to `Item_Num = 30`, and stereo computation is executed by either OCA or CPU depending on the value of \<mode\>.<br>
With regard to Meaning of parameters refer to Simple and Detail parameter mode.


```sh
./stereo_app <mode>

    For example
    ./stereo_app oca
    ./stereo_app cpu
```

| mode |
| :-: |
| oca |
| cpu |

Parameter Table for “Item\_Num = 30”

| Setting | INPUT/OUTPUT | | | | Prameters | | | | | | | | | | | | | | | |
| :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: |
| Item_<br />Num  | INPUT METHOD | INPUT<br />SIZE | OUTPUT<br />SIZE | | image | format | fps | Left | Right | Image<br />Size Num | P1 | P2 | Depth | Win<br />-X | Win<br />-Y | Remap | unique<br />ness<br />Ratio | Gamma | Sharp | Heat<br />_Map |
| 30 | file | 881/400 | QVGA* | | data | file | 0 | drivingstereo/2018-07-11-14-48-52-L-811 | drivingstereo/2018-07-11-14-48-52-R-811 | 9 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 20 |

### (2) Simple parameter mode 

Select "mode" and "Item_Num" from a combination of execute mode, input method, input size, and output size.
Check the camera format using the command on the right terminal figure and select the appropriate “INPUT METHOD” and “INPUT SIZE”.
With regard to Meaning of parameters refer to Detail parameter mode. 

```sh
./stereo_app <mode> <Item_Num>

    For example
    ./stereo_app oca 30
    ./stereo_app cpu 30
    ./stereo_app oca 0
    ./stereo_app oca 10
```

| mode |
| :-: |
| oca |
| cpu |

| Setting | INPUT/OUTPUT | | |   | Prameters | | | | | | | | | | | | | | | |
| :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: |
| Item_<br />Num  | INPUT METHOD | INPUT<br />SIZE | OUTPUT<br />SIZE | | image | format | fps | Left | Right | Image<br />Size Num | P1 | P2 | Depth | Win<br />-X | Win<br />-Y | Remap | unique<br />ness<br />Ratio | Gamma | Sharp | Heat<br />_Map |
| 0 | camera/YUYV | VGA | QVGA* |   | usb | YUYV | 30 | video0 | video2 | 0　 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 20 |
| 1 | camera/YUYV | VGA | VGA |  | usb | YUYV | 30 | video0 | video2 | 1 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 2 | camera/YUYV | HD | QVGA* |  | usb | YUYV | 30 | video0 | video2 | 2 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 20 |
| 3 | camera/YUYV | HD | VGA |  | usb | YUYV | 30 | video0 | video2 | 3 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 4 | camera/YUYV | HD | HD |  | usb | YUYV | 30 | video0 | video2 | 4 | 10 | 30 | 96 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 5 | camera/YUYV | FHD | QVGA* |  | usb | YUYV | 30 | video0 | video2 | 5 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 20 |
| 6 | camera/YUYV | FHD | VGA |  | usb | YUYV | 30 | video0 | video2 | 6 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 7 | camera/YUYV | FHD | HD |  | usb | YUYV | 30 | video0 | video2 | 7 | 10 | 30 | 96 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 8 | camera/YUYV | FHD | FHD |  | usb | YUYV | 30 | video0 | video2 | 8 | 10 | 30 | 128 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 9 | --- | --- | --- |  | --- | | | | | 0 | | | | | | | | | | |
| 10 | camera/MJPG | VGA | QVGA* |  | usb | MJPG | 30 | video0 | video2 | 0 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 20 |
| 11 | camera/MJPG | VGA | VGA |  | usb | MJPG | 30 | video0 | video2 | 1 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 12 | camera/MJPG | HD | QVGA* |  | usb | MJPG | 30 | video0 | video2 | 2 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 20 |
| 13 | camera/MJPG | HD | VGA |  | usb | MJPG | 30 | video0 | video2 | 3 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 14 | camera/MJPG | HD | HD |  | usb | MJPG | 30 | video0 | video2 | 4 | 10 | 30 | 96 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 15 | camera/MJPG | FHD | QVGA* |  | usb | MJPG | 30 | video0 | video2 | 5 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 20 |
| 16 | camera/MJPG | FHD | VGA |  | usb | MJPG | 30 | video0 | video2 | 6 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 17 | camera/MJPG | FHD | HD |  | usb | MJPG | 30 | video0 | video2 | 7 | 10 | 30 | 96 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 18 | camera/MJPG | FHD | FHD |  | usb | MJPG | 30 | video0 | video2 | 8 | 10 | 30 | 128 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 19 | --- | --- | --- |  | --- | | | | | | | | | | | | | | | |
| 20 | file | VGA | QVGA* |  | data | file | 0 | images_stereo/take3-L-640 | images_stereo/take3-R-640 | 0 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 20 |
| 21 | file | VGA | VGA |  | data | file | 0 | images_stereo/take3-L-640 | images_stereo/take3-R-640 | 1 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 22 | file | HD | QVGA* |  | data | file | 0 | images_stereo/take3-L-1280 | images_stereo/take3-R-1280 | 2 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 20 |
| 23 | file | HD | VGA |  | data | file | 0 | images_stereo/take3-L-1280 | images_stereo/take3-R-1280 | 3 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 24 | file | HD | HD |  | data | file | 0 | images_stereo/take3-L-1280 | images_stereo/take3-R-1280 | 4 | 10 | 30 | 96 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 25 | file | FHD | QVGA* |  | data | file | 0 | images_stereo/take3-L-1920 | images_stereo/take3-R-1920 | 5 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 20 |
| 26 | file | FHD | VGA |  | data | file | 0 | images_stereo/take3-L-1920 | images_stereo/take3-R-1920 | 6 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 27 | file | FHD | HD |  | data | file | 0 | images_stereo/take3-L-1920 | images_stereo/take3-R-1920 | 7 | 10 | 30 | 96 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 28 | file | FHD | FHD |  | data | file | 0 | images_stereo/take3-L-1920 | images_stereo/take3-R-1920 | 8 | 10 | 30 | 128 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 29 | --- | --- | --- |  | --- | | | | | | | | | | | | | | | |
| 30 | file | 881/400 | QVGA* |  | data | file | 0 | drivingstereo/2018-07-11-14-48-52-L-811 | drivingstereo/2018-07-11-14-48-52-R-811 | 9 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 20 |
| 31 | file | 881/400 | 800/300 |  | data | file | 0 | drivingstereo/2018-07-11-14-48-52-L-811 | drivingstereo/2018-07-11-14-48-52-R-811 | 10 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 40 | camera/EC22 | VGA | QVGA* |  | ec22 | YUYV | 30 | video0 | video1 | 0 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 20 |
| 41 | camera/EC22 | VGA | VGA |  | ec22 | YUYV | 30 | video0 | video1 | 1 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 42 | camera/EC22 | HD | QVGA* |  | ec22 | YUYV | 30 | video0 | video1 | 2 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 20 |
| 43 | camera/EC22 | HD | VGA |  | ec22 | YUYV | 30 | video0 | video1 | 3 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 44 | camera/EC22 | HD | HD |  | ec22 | YUYV | 30 | video0 | video1 | 4 | 10 | 30 | 96 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 45 | camera/EC22 | FHD | QVGA* |  | ec22 | YUYV | 30 | video0 | video1 | 5 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 20 |
| 46 | camera/EC22 | FHD | VGA |  | ec22 | YUYV | 30 | video0 | video1 | 6 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 47 | camera/EC22 | FHD | HD |  | ec22 | YUYV | 30 | video0 | video1 | 7 | 10 | 30 | 96 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 48 | camera/EC22 | FHD | FHD |  | ec22 | YUYV | 30 | video0 | video1 | 8 | 10 | 30 | 128 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 49 | --- | --- | --- |  | --- | | | | | | | | | | | | | | | |
| 50 | --- | --- | --- |  | --- | | | | | | | | | | | | | | | |
| 51 | --- | --- | --- |  | --- | | | | | | | | | | | | | | | |
| 52 | camera/EC25 | HD | QVGA* |  | ec25 | YUYV | 30 | video0 | video1 | 2 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 20 |
| 53 | camera/EC25 | HD | VGA |  | ec25 | YUYV | 30 | video0 | video1 | 3 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 54 | camera/EC25 | HD | HD |  | ec25 | YUYV | 30 | video0 | video1 | 4 | 10 | 30 | 96 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 55 | camera/EC25 | FHD | QVGA* |  | ec25 | YUYV | 30 | video0 | video1 | 5 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 20 |
| 56 | camera/EC25 | FHD | VGA |  | ec25 | YUYV | 30 | video0 | video1 | 6 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 57 | camera/EC25 | FHD | HD |  | ec25 | YUYV | 30 | video0 | video1 | 7 | 10 | 30 | 96 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 58 | camera/EC25 | FHD | FHD |  | ec25 | YUYV | 30 | video0 | video1 | 8 | 10 | 30 | 128 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 59 | --- | --- | --- |  | --- | | | | | | | | | | | | | | | |
| 60 | stereo/YUYV | VGAx2 | QVGA* |  | stereo | YUYV | 120 | video0 | tmp | 0 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 20 |
| 61 | stereo/YUYV | VGAx2 | VGA |  | stereo | YUYV | 120 | video0 | tmp | 1 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 62 | stereo/YUYV | HDx2 | QVGA* |  | stereo | YUYV | 30 | video0 | tmp | 2 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 20 |
| 63 | stereo/YUYV | HDx2 | VGA |  | stereo | YUYV | 30 | video0 | tmp | 3 | 10 | 30 | 64 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 64 | stereo/YUYV | HDx2 | HD |  | stereo | YUYV | 30 | video0 | tmp | 4 | 10 | 30 | 96 | 9 | 9 | 0 | 0 | 0 | 0 | 1 |
| 65 | --- | --- | --- |  | --- | | | | | | | | | | | | | | | |
| 66 | --- | --- | --- |  | --- | | | | | | | | | | | | | | | |
| 67 | --- | --- | --- |  | --- | | | | | | | | | | | | | | | |
| 68 | --- | --- | --- |  | --- | | | | | | | | | | | | | | | |
| 69 | --- | --- | --- |  | --- | | | | | | | | | | | | | | | |

### (3) Detail parameter mode

Run the following the command and the parameter.

```sh
./stereo_app <mode> <image> <format> <fps> <left> <right> <size> <p1> <p2> <dep> <win_x> <win_y> <remap> <uniquenessRatio> <gamma> <sharp> <heatmap> 

    For example
    ./stereo_app oca data file 0 drivingstereo/2018-07-11-14-48-52-L-811 drivingstereo/2018-07-11-14-48-52-R-811 9 10 30 64 9 9 0 0 0 0 20
    ./stereo_app cpu data file 0 drivingstereo/2018-07-11-14-48-52-L-811 drivingstereo/2018-07-11-14-48-52-R-811 9 10 30 64 9 9 0 0 0 0 20
    ./stereo_app oca usb YUYV 30 video0 video2 0 10 30 64 9 9 0 0 0 0 20
    ./stereo_app cpu usb YUYV 30 video0 video2 0 10 30 64 9 9 0 0 0 0 20
    ./stereo_app oca usb MJPG 30 video0 video2 0 10 30 64 9 9 0 0 0 0 20
    ./stereo_app cpu usb MJPG 30 video0 video2 0 10 30 64 9 9 0 0 0 0 20
```

| mode |
| :-: |
| oca |
| cpu |

| image | format | fps | left_image | right_image |
| :-: | :-: | :-: | :-: | :-: |
| usb | YUYV | 30 | video0 | video2 |
| usb | MJPG | 30 | video0 | video2 |
| data | file | 0 | left folder name | right folder name |
| ec22 | YUYV | 30 | video0 | video1 |
| ec25 | YUYV | 30 | video0 | video1 |
| stereo | YUYV | 30/120 | video0 | tmp |

In case of \<image\> = data, <br>
the name of the folder saved image and the name of the image list file must be the same.

```sh
# ls drivingstereo/
2018-07-11-14-48-52-L-811      <--- the folder saved image
2018-07-11-14-48-52-L-811.txt  <--- the image list file
2018-07-11-14-48-52-R-811
2018-07-11-14-48-52-R-811.txt
```

In case of \<image\> = usb, <br>

```sh
# v4l2-ctl --list-devices
Anker PowerConf C200: Anker Pow (usb-15850000.usb-1):
        /dev/video0
....
Anker PowerConf C200: Anker Pow (usb-15860000.usb-1):
        /dev/video2
....
```

In case of \<image\> = ec22 or ec25, <br>

```sh
# v4l2-ctl --list-devices
RZG2L_CRU (platform:16000000.cru0):
        /dev/video0
....
RZG2L_CRU (platform:16010000.cru1):
        /dev/video1
....
```

When inputting the image file, select the same "Input image size" as the width and height of the image file.<br>
W_cal need to multiple of 4.

| |  | input image | size |  | calculate image | size |
| :-: | :-: | :-: | :-: | :-: | :-: | :-: |
| Image Size Num |  | W_in | H_in |  | W_cal | H_cal |
| 0 |  | 640 | 480 |  | 384 | 288 |
| 1 |  | 640 | 480 |  | 640 | 480 |
| 2 |  | 1280 | 720 |  | 384 | 288 |
| 3 |  | 1280 | 720 |  | 640 | 480 |
| 4 |  | 1280 | 720 |  | 1280 | 720 |
| 5 |  | 1920 | 1080 |  | 384 | 288 |
| 6 |  | 1920 | 1080 |  | 640 | 480 |
| 7 |  | 1920 | 1080 |  | 1280 | 720 |
| 8 |  | 1920 | 1080 |  | 1920 | 1080 |
| 9 |  | 881 | 400 |  | 384 | 288 |
| 10 |  | 881 | 400 |  | 700 | 320 |

p1, p2 parameter of the disparity smoothness
p1 < p2

| p1 | p2 | |
| :-: | :-: | :-: |
| 2 \~ 80 | 5 \~ 80 | W_cal <= 640 |
| 2 \~ 50 | 5 \~ 50 | W_cal <= 1280 |
| 2 \~ 30 | 5 \~ 30 | W_cal <= 1920 |

Calculation range to search for the pattern of the left image from the right image.
Disparity need to multiple of 4.

| disparity | |
| :-: | :-: |
| 16 \~ 256 | W_cal <= 1280 |
| 16 \~ 128 | W_cal <= 1920 |

Census window size
win_x = win_y

| win_x |
| :-: |
| 3,5,7,9 |

| win_y |
| :-: |
| 3,5,7,9 |

<img src=./img/census_flow.png width=300 alt="The principle of stereo camera" style="border: 3px solid grey;">

Calibration data : Camera matrix and Distortion coefficients.

| remap | |
| :-: | :- |
| 0 | Not apply the calibration data. |
| 1 | Apply the left and right calibration data to each camera. |
| 2 | Apply left calibration data to each camera. |
| 3 | Apply right calibration data to each camera. |

The parameter used to determine the reliability of matching between the left and right images. <br>
It sets the threshold ratio between the best (minimum) computed cost value and the second-best cost value.<br>
If the difference is greater than the threshold, the disparity value is returned; otherwise, 0 is returned.

| uniquenessRatio |
| :-: |
| 0\~100 |

| No. | Gamma |  | No. | Gamma |  | No. | Gamma |  | No. | Gamma |  | No. | Gamma |
| :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: |
| 0 | --- |  | 10 | 1.0 |  | 20 | 2.0 |  | 30 | 3.0 |  | 40 | ST1 |
| 1 | 0.1 |  | 11 | 1.1 |  | 21 | 2.1 |  | 31 | 3.1 |  | 41 | SB1 |
| 2 | 0.2 |  | 12 | 1.2 |  | 22 | 2.2 |  | 32 | 3.2 |  | 42 | SB2 |
| 3 | 0.3 |  | 13 | 1.3 |  | 23 | 2.3 |  | 33 | 3.3 |  | 43 | SB3 |
| 4 | 0.4 |  | 14 | 1.4 |  | 24 | 2.4 |  | 34 | 3.4 |  | 44 | SB4 |
| 5 | 0.5 |  | 15 | 1.5 |  | 25 | 2.5 |  | 35 | 3.5 |  | 45 | SB5 |
| 6 | 0.6 |  | 16 | 1.6 |  | 26 | 2.6 |  | 36 | 3.6 |  | 46 | SB6 |
| 7 | 0.7 |  | 17 | 1.7 |  | 27 | 2.7 |  | 37 | 3.7 |  | 47 | SB7 |
| 8 | 0.8 |  | 18 | 1.8 |  | 28 | 2.8 |  | 38 | 3.8 |  | 48 | --- |
| 9 | 0.9 |  | 19 | 1.9 |  | 29 | 2.9 |  | 39 | 3.9 |  | 49 | --- |

<img src=./img/gamma_1.png width=300 alt="The principle of stereo camera" style="border: 3px solid grey;">
<img src=./img/gamma_2.png width=300 alt="The principle of stereo camera" style="border: 3px solid grey;">

| sharp | k |
| :-: | :-: |
| 0 | OFF |
| 1 | ON |

\[\-1\, \-1\, \-1\]\, <br>
\[\-1\,  9\, \-1\]\, <br>
\[\-1\, \-1\, \-1\]\,

| heat map |
| :-: |
| 0 \~ 21 |

<img src=./img/heat_map_color.png width=250 alt="The principle of stereo camera" style="border: 3px solid grey;">

## Application: Comparison of parameters between DRP Library and CV::StereoSGBM

For information about cv::StereoSGBM\, please refer to the following URL.<br>
[cv::StereoSGBM](https://docs.opencv.org/4.x/d2/d85/classcv_1_1StereoSGBM.html)

| | OpenCV(Ver4.9.0)  cv::StereoSGBM | | | | OCA StereoSGM | |
| :-: | :-: | :- | :-: | :-: | :-: | :-: |
| No. | Parameters | Description | Initial Value | Range | Parameters | Range |
| 1 | minDisparity | Minimum possible disparity value. Normally, it is zero | 0 | 0 (Normally) | 0 | 0 |
| 2 | numDisparities | Maximum disparity minus minimum disparity.<br> In the current implementation, this parameter must be divisible by 16. | 16 | Multiple of 16 | Disparity | Multiple of 16<br>16 \~ 256 (width ≦ 1280)<br>16 \~ 128 (width ≦ 1920) |
| 3 | blockSize | Matched block size. | 3 | 3,5,7,9,11 | win_x<br>win_y | 3,5,7,9 |
| 4 | P1 | The first parameter controlling the disparity smoothness.<br>See below. | 2 | P2 > P1 > 0 | p1 | P2 > P1<br>1\~80 (Width ≦ 640)<br>1\~50 (Width ≦ 1280)<br>1\~30 (Width ≦ 1920) |
| 5 | P2 | The second parameter controlling the disparity smoothness. <br>The larger the values are, the smoother the disparity is.<br>P1 is the penalty on the disparity change by plus or minus 1 between neighbor pixels.<br>P2 is the penalty on the disparity change by more than 1 between neighbor pixels. | 5 | P2 > P1 > 0 | p2 | P2 > P1 <br>2\~80 (Width ≦ 640)<br>2\~50 (Width ≦ 1280)<br>2\~30 (Width ≦ 1920) |
| 6 | disp12MaxDiff | Maximum allowed difference (in integer pixel units) in the left-right disparity check.  | 0 | --- | Not implemented | --- |
| 7 | preFilterCap | Truncation value for the prefiltered image pixels.  | 0 | --- | Not implemented | --- |
| 8 | uniquenessRatio | Margin in percentage by which the best (minimum) computed cost function value should "win" the second best value to consider the found match correct. | 0 | 0\~100 | uniquenessRatio | 0\~100 |
| 9 | speckleWindowSize | Maximum size of smooth disparity regions to consider their noise speckles and invalidate. | 0 | --- | Not implemented | --- |
| 10 | speckleRange | Maximum disparity variation within each connected component. | 0 | --- | Not implemented | --- |
| 11 | mode | The algorithm<br>directions | MODE_SGBM<br>(5 directions) | MODE_SGBM (5)<br>MODE_HH (8)<br>MODE_SGBM_3WAY (3)<br>MODE_HH4 (4) | fixed | MODE_SGM_DRP<br>(4 directions) |
| 12 | input data format | Input image format | --- | RGB<br>GRAY SCALE | fixed | GRAY SCALE |
| 13 | input data width | Input image width | --- | --- | --- | Multiple of 4<br>32 ≦ width ≦ 1920 |

## Implementation using DRP for StereoSGM

```text
The flow of stereoSGM is divided into the following two blocks.
    (1) Census transforms and Block Matching
    (2) Dynamic Programming(DP)
These two blocks are implemented together in DRP.
```

<img src=./img/stereosgm_flow_1.png width=800 alt="The principle of stereo camera" style="border: 3px solid grey;">

### (1) Census transforms and Block Matching

<img src=./img/stereosgm_flow_2.png width=800 alt="The principle of stereo camera" style="border: 3px solid grey;">

### (2) Dynamic Programming(DP)

<img src=./img/stereosgm_flow_3.png width=800 alt="The principle of stereo camera" style="border: 3px solid grey;">

## Explanation of Sample software

### Sample Program Configuration

```text
Image input can be selected from Dataset or Camera. Stereo processing can be selected from OCA or CPU.
Show the Disparity and the execution time on the display. The sample software is composed of the following six threads.
    (1) capture thread : Capture the image from camera.
    (2) imageL thread : Preprocess the left image.
    (3) imageR thread : Preprocess the right image.
    (4) stereo thread : Stereo processing selectable OCA or CPU. 
    (5) display thread : Postprocess and display the disparity.
    (6) kbhit thread : Wait for keyboard input, then either exit the application or adjust the image.
```

<img src=./img/sample_soft_1.png width=800 alt="The principle of stereo camera" style="border: 3px solid grey;">

### Flowchart

<img src=./img/sample_soft_2.png width=800 alt="The principle of stereo camera" style="border: 3px solid grey;">

### Timingchart

Each thread runs in parallel and operates within the pipeline.

<img src=./img/sample_soft_3.png width=800 alt="The principle of stereo camera" style="border: 3px solid grey;">

|||
|:-:|:-:|
|camera frame rate |30fps|
|camera image size| 640 x 480|
|OCA input size |384 x 288|
|Disparity image |size 384 x 288|

```sh
For Example
    ./stereo_app oca 10
```

### Software configuration to launch DRP

```text
This sample software outputs disparity images using the OpenCV API cv::stereoSGBM. 
By setting the argument mode to MODE_SGM_DRP, "/dev/drp1" driver is opened and the DRP library is executed.
```

<img src=./img/sample_soft_4.png width=800 alt="The principle of stereo camera" style="border: 3px solid grey;">

```c++
/* [CPU]Opencv start */
OCA_list[OCA_FUNC_STEREOSGM] = OPENCVA_FUNC_ DISABLE;
OCA_Activate(&OCA_list[0]);
mode = cv::StereoSGBM::MODE_SGBM;
cv::Ptr<cv::StereoSGBM> stereo_cpu = cv::StereoSGBM::create(minDisparity, numDisparities, blockSize, P1, P2,
                                disp12MaxDiff, preFilterCap, uniquenessRatio,speckleWindowSize, speckleRange, mode);

stereo_cpu->compute(src1_gray, src2_gray, stereo_out_cpu);
```

```c++
/* [OCA]Opencv start */
OCA_list[OCA_FUNC_STEREOSGM] = OPENCVA_FUNC_ENABLE;
OCA_Activate(&OCA_list[0]);
mode = cv::StereoSGBM::MODE_SGM_DRP;
cv::Ptr<cv::StereoSGBM> stereo_oca = cv::StereoSGBM::create(minDisparity, numDisparities, blockSize, P1, P2,
                                disp12MaxDiff, preFilterCap, uniquenessRatio,speckleWindowSize, speckleRange, mode);

stereo_oca->compute(src1_gray, src2_gray, stereo_out_oca);
```

## Example for setting parameters and camera

### Casio EX-XR3100

<img src=./img/camera_casio.png width=500 alt="The principle of stereo camera" style="border: 3px solid grey;">

```text
sensor : 7.7mm x 5.6mm (1/1.7 CMOS)
Equivalent size of 1pix when outputting at 1920 : 7.7/1920=4.01[um/pix]
F = 5.4mm : Convert "mm" to "pix" 5.4[mm] / 4.01[um/pix] = 1346[pix]
B = 163mm
α = 1920 / 384 
Z = ( B x F ) / D
   =163[mm] x 1346[pix] / (d[pix] x α)
Disparity(max)=64

command :
./stereo_app  oca data file 0 images_stereo/take3-L-640 images_stereo/take3-R-640 0 10 30 64 9 9 0 0 0 0 20
```

<!--
<video src=./img_mp4/stereo_oca_park_crop_resize.mp4 width=800 alt="stereo cpu" controls="true" style="border: 3px solid grey;"></video>
-->

<video src="https://github.com/user-attachments/assets/6f43c141-e913-4645-b9a0-e3cc959fa611" width="800" alt="stereo cpu" controls="true" style="border: 3px solid grey;"></video>

<img src=./img/camera_casio_disparity.png width=400 alt="The principle of stereo camera" style="border: 3px solid grey;">

### Anker PowerConf C200

<img src=./img/camera_anker.png width=500 alt="The principle of stereo camera" style="border: 3px solid grey;">
<img src=./img/camera_matrix.png width=200 alt="The principle of stereo camera" style="border: 3px solid grey;">

```text
F=503[pix] (Using the camera matrix[fx] obtained by camera calibration)
B=40mm
α=640 / 384 
Z = ( B x F ) / D
   =40[mm] x 503[pix] / (d[pix] x α)
Disparity(max)=80

command :
./stereo_app  oca usb YUYV 30 video0 video2 0 30 60 80 9 9 0 0 0 0 20
```

<!--
<video src=./img_mp4/stereo_oca_desk_crop_resize.mp4 width=800 alt="stereo cpu" controls="true" style="border: 3px solid grey;"></video>
-->

<video src="https://github.com/user-attachments/assets/4b84b094-7607-401a-844b-4417e53b2946" width="800" alt="stereo cpu" controls="true" style="border: 3px solid grey;"></video>

<img src=./img/camera_anker_disparity.png width=400 alt="The principle of stereo camera" style="border: 3px solid grey;">

### Example of changing the parameters

Show the results of calculating disparity after changing the following parameters.<br>

```text
(1) p1,p2,
(2) disparity,
(3) win_x,win_y,
(4) uniquenessRatio
```

|Base command|p1|p2|disparity|win_x|win_y||uniqueness<br>Ratio||||
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
|./stereo_app oca data file 0 images_stereo/take3-L-640 images_stereo/take3-R-640 0|10|30|64|9|9|0|0|0|0|20|

<img src=./img/param.png width=1000 alt="The principle of stereo camera" style="border: 3px solid grey;">

### Example of a jig fixing cameras

Fix the two cameras in parallel using the jig.<br>
If the up/down angles or left/right directions differ between the cameras, the disparity image cannot be calculated exactly.

e-CAM22 and e-CAM25 need to mount upside down. It is also possible to flip using the camera driver.

|USB camara|e-CAM22|e-CAM25|
|:-:|:-:|:-:|
|<img src=./img/usb_camera_jig.png width=205 alt="The principle of stereo camera" style="border: 3px solid grey;">|<img src=./img/ecam22_jig.png width=250 alt="The principle of stereo camera" style="border: 3px solid grey;">|<img src=./img/ecam25_jig.png width=255 alt="The principle of stereo camera" style="border: 3px solid grey;">

## Calibraiton Method

### Calibraiton Flow

|e-CAM22|e-CAM25|
|:-:|:-:|
|<img src=./img/camera_calib_usb.png width=400 alt="The principle of stereo camera" style="border: 3px solid grey;">|<img src=./img/camera_calib_mipi.png width=378 alt="The principle of stereo camera" style="border: 3px solid grey;">|

(1) Copy and Extract rzv2h_calib_camera_ver1.00.tar.gz on EVK.

```sh
cp rzv2h_calib_camera_ver1.00.tar.gz  /home/weston
cd /home/weston
tar -zxvf rzv2h_calib_camera_ver1.00.tar.gz
```

(2) Download the Chesspattern and print it.

This file is Chesspattern. Use it when calibrating.

|Chesspattern image|Download file|
|:-:|:-:|
|<img src=./img/chesspattern_7x10.png width=100 alt="The principle of stereo camera" style="border: 3px solid grey;">|[chesspattern_7x10](./img/chesspattern_7x10.pdf)|

(3) Start Calibration

Change directory and run python script.

```sh
cd rzv2h_calib_camera_ver1.00
python3 calib_cam.py 2 1 1 2
```

Displayed next message.

```sh
./set_e-CAM25.sh left 1280 720
e-CAM25 left 1280 720
[ 1091.153687] ar0234 0-0042: mcu_stream_config: width = 1280, height = 720
[ 1091.628166] ar0234 1-0042: mcu_stream_config: width = 1280, height = 720
Capture 25 images. Press c key to capture a image. Press q key to end.
```

The meaning of the Calibration command arguments is as follows.

```sh
python3  calib_cam.py  1st:Type  2nd:Side  3rd:Format  4th:Size
```

| 1st argument | Camera Type |
| :-: | :-: |
| 1 | e-CAM22 |
| 2 | e-CAM25 |
| 3 | USB |

| 2nd argument | Camera Side |
| :-: | :-: |
| 1 | left |
| 2 | rigth |

| 3rd argument | Camera Format |
| :-: | :-: |
| 1 | YUYV |
| 2 | MJPG |

| 4th argument | Camera Size |
| :-: | :-: |
| 1 | 640x480 |
| 2 | 1280x720 |
| 3 | 1920x1080 |


(4) Take pictures of the chesspattern from different angles and distances.<br>
Capture 25 images. Press c key to capture a image. Press q key to end.

```sh
Capture 25 images. Press c key to capture a image. Press q key to end.
Detect chessboard = 1/25
Detect chessboard = 2/25
.....
Detect chessboard = 25/25
RMS =  1.7269159917128716
K =
.....
Output Camera file camera_EC25_left_YUYV_1280_720.csv
Output Distortion file dist_EC25_left_YUYV_1280_720.csv
```

(5) Copy output 2 files(camera_xxx_....csv, dist_xxx_....csv)<br>
to /home/weston/rzv2h_stereosgm_ver3.10/src_rzv2h/calib_data

```sh
cp camera_EC25_left_YUYV_1280_720.csv /home/weston/rzv2h_stereosgm_ver3.10/src_rzv2h
cp dist_EC25_left_YUYV_1280_720.csv /home/weston/rzv2h_stereosgm_ver3.10/src_rzv2h
```

(6) Run the following the command and Remap = 1.

```sh
cd /home/weston/rzv2h_stereosgm_ver3.10/src_rzv2h
./stereo_app  oca ec25 YUYV 30 video0 video1 3 10 30 64 9 9 1 0 0 0 1
```

|Calibration flow|Caprure image|
|:-:|:-:|
|<img src=./img/camera_calib_flow.png width=300 alt="The principle of stereo camera" style="border: 3px solid grey;">|<img src=./img/camera_calib_img_1.png width=150 alt="The principle of stereo camera" style="border: 3px solid grey;"><br><img src=./img/camera_calib_img_2.png width=150 alt="The principle of stereo camera" style="border: 3px solid grey;"><br><img src=./img/camera_calib_img_3.png width=150 alt="The principle of stereo camera" style="border: 3px solid grey;">|

### Effects of distortion correction

e-CAM22 and e-CAM25 have distortion so it is recommended to calibrate them.

|Before calibration|After calibration|
|:-:|:-:|
|left image / right image|left image / rignt image|
|<img src=./img/camera_calib_before.png width=300 alt="The principle of stereo camera" style="border: 3px solid grey;">|<img src=./img/camera_calib_after.png width=300 alt="The principle of stereo camera" style="border: 3px solid grey;">|

## License

- Apache 2.0

The license is included in the folder for each OSS.<br>
For this application, see following directory.

|Software|License|
|:-:|:-:|
|OpenCV Accelerator|[License](https://github.com/renesas-rz/rzv2h_opencv_accelerator?tab=readme-ov-file#license)|

|DRP Driver|[License](https://github.com/renesas-rz/rzv2h_opencv_accelerator?tab=readme-ov-file#license)|






