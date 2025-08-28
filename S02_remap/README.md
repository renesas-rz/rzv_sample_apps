# Remap

## Application: Overview
This application is designed to correct distortion of the lens by reading calibration files and display the results on an HDMI screen.<br>
The application calibrates the lens distortion and outputs the results to calibration files, so it can handle distortions caused by various lenses.

### Supported Product  
<table>
    <tr>
      <th>Product</th>
      <th>Supported AI SDK version</th>
    </tr>
    <tr>
      <td>RZ/V2H Evaluation Board Kit (RZ/V2H EVK)</td>
      <td>RZ/V2H AI SDK v5.20</td>
    </tr>
    <tr>
      <td>RZ/V2N Evaluation Board Kit (RZ/V2N EVK)</td>
      <td>RZ/V2N AI SDK v6.00</td>
    </tr>
    
</table>

### Input/Output
<table>
    <tr>
      <th>Board</th>
      <th>Input</th>
      <th>Output</th>
    </tr>
    <tr>
      <td>RZ/V2H EVK</td>
      <td rowspan="2">MIPI camera</td>
      <td rowspan="2">HDMI</td>
    </tr>
    <tr>
      <td >RZ/V2N EVK</td>
    </tr>
</table>


## Application: Requirements

### Hardware Requirements
<table>
    <tr>
      <th>For</th>
      <th>Equipment</th>
      <th>Details</th>
    </tr>
    <tr>
      <td rowspan="4">RZ/V2H</td>
      <td>RZ/V2H EVK</td>
      <td>Evaluation Board Kit for RZ/V2H.</td>
    </tr>
    <tr>
      <td>AC Adapter</td>
      <td>USB Power Delivery adapter for the board power supply.<br>
      100W is required.</td>
    </tr>
    <tr>
      <td>HDMI Cable</td>
      <td>Used to connect the HDMI Monitor and the board.<br>
      RZ/V2H EVK has HDMI port.</td>
    </tr>
    <tr>
      <td>MIPI Camera</td>
      <td>Used as a camera input source.</td>
    </tr>
    <tr>
      <td rowspan="4">RZ/V2N</td>
      <td>RZ/V2N EVK</td>
      <td>Evaluation Board Kit for RZ/V2N.</td>
    </tr>
    <tr>
      <td>AC Adapter</td>
      <td>USB Power Delivery adapter for the board power supply.<br>
      100W is required.</td>
    </tr>
    <tr>
      <td>HDMI Cable</td>
      <td>Used to connect the HDMI Monitor and the board.<br>
      RZ/V2N EVK has HDMI port.</td>
    </tr>
    <tr>
      <td>MIPI Camera</td>
      <td>Used as a camera input source.</td>
    </tr>
    <tr>
      <td rowspan="8">Common</td>
      <td>USB Cable Type-C</td>
      <td>Connect AC adapter and the board.</td>
    </tr>
    <tr>
      <td>HDMI Monitor</td>
      <td>Used to display the graphics of the board.</td>
    </tr>
    <tr>
      <td>microSD card</td>
      <td>Used as the filesystem.<br>
      Must have over 16GB capacity of blank space.<br>
      Operating Environment: Transcend UHS-I microSD 300S 16GB</td>
    </tr>
    <tr>
      <td>Linux PC</td>
      <td>Used to build application and setup microSD card.<br>
      Operating Environment: Ubuntu 20.04</td>
    </tr>
    <tr>
      <td>SD card reader</td>
      <td>Used for setting up microSD card.<br></td>
    </tr>
    <tr>
      <td>USB Hub</td>
      <td>Used to connect USB Keyboard and USB Mouse to the board.</td>
    </tr>
    <tr>
      <td>USB Keyboard</td>
      <td>Used to type strings on the terminal of board.</td>
    </tr>
    <tr>
      <td>USB Mouse</td>
      <td>Used to operate the mouse on the screen of board.</td>
    </tr>
  </table>

>**Note:** All external devices will be attached to the board and does not require any driver installation (Plug n Play Type)

Connect the hardware as shown below.  

|RZ/V2H EVK | RZ/V2N EVK |
|:---|:---|
|<img src=./img/remap_hw_conf_v2h.png width=600> |<img src=./img/remap_hw_conf_v2n.png width=600> |

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
    |Board | Docker container |
    |:---|:---|
    |RZ/V2H EVK|`rzv2h_ai_sdk_container`  |
    |RZ/V2N EVK|`rzv2n_ai_sdk_container`  |

    >**Note 1:** Docker environment is required for building the sample application.  
<!--    >**Note 2:** Since RZ/V2N is a brother chip of RZ/V2H, the same environment can be used. -->


### Application File Generation
1. On your host machine, copy the repository from the GitHub to the desired location. 
    1. It is recommended to copy/clone the repository on the `data` folder, which is mounted on the Docker container. 
    ```sh
    cd <path_to_data_folder_on_host>/data
    git clone https://github.com/renesas-rz/rzv_sample_apps/tree/apps/S02_remap
    ```
    >Note: This command will download the whole repository, which include all other applications.  
    If you have already downloaded the repository of the same version, you may not need to run this command.  

2. Run (or start) the docker container and open the bash terminal on the container.  
E.g., for RZ/V2H, use the `rzv2h_ai_sdk_container` as the name of container created from  `rzv2h_ai_sdk_image` docker image.  
    > Note that all the build steps/commands listed below are executed on the docker container bash terminal.  

3. Set your clone directory to the environment variable.  
    ```sh
    export PROJECT_PATH=/drp-ai_tvm/data
    ```
4. Go to the application source code directory.
    - For RZ/V2H,  
      ```sh
      cd ${PROJECT_PATH}/S02_remap/src_v2h
      ```
    - For RZ/V2N,  
      ```sh
      cd ${PROJECT_PATH}/S02_remap/src_v2n
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
7. The following application file would be generated in the `${PROJECT_PATH}/S02_remap/src/build` directory
    - app_camera_calibration


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
|RZ/V2N EVK|[exe_v2n](./exe_v2n)  |
<!-- > Note: Since RZ/V2N is a brother chip of RZ/V2H, the same execution environment can be used.   -->
  
Each folder contains following items.
|File | Details |
|:---|:---|
|licenses | License information. <br>Not necessary for running application. |
|camera.csv | Correction file1. |
|dist.csv | Correction file2. |
|app_camera_calibration | application file. |

### Instruction
1. Register the working directory path to an environment variable.<br>
The environment variable WORK is the working directory path that you set in Step 4-2 of [Getting Started Guide](https://renesas-rz.github.io/rzv_ai_sdk/latest/getting_started.html#step4).

    ```sh
    export WORK=<path to the working directory>/ai_sdk_work
    export APPS_PATH=${WORK}/ai_sdk_setup/data
    ```
2. Copy the following files to the `/home/root/exe` directory (RZ/V2H) or `/home/weston/exe` directory (RZ/V2N) of the rootfs (SD Card) for the board.
    |File | Details |
    |:---|:---|
    |`app_camera_calibration` application file | Generated the file according to [Application File Generation](#application-file-generation) |

3. Folder structure in the rootfs (SD Card) is shown below.<br>
   
   - For RZ/V2H
    ```
    |-- home/
        `-- root/
            `-- exe/ 
                |-- camera.csv
                |-- dist.csv
                `-- app_camera_calibration
    ```
   - For RZ/V2N
    ```
    |-- home/
        `-- weston/
            `-- exe/ 
                |-- camera.csv
                |-- dist.csv
                `-- app_camera_calibration
    ```

>**Note:** The directory name could be anything instead of `exe`. If you copy the whole `EXE_DIR` folder on the board, you are not required to rename it `exe`.

## Application: Run Stage

### Prerequisites
This section expects the user to have completed Step 7-3 of [Getting Started Guide](https://renesas-rz.github.io/rzv_ai_sdk/latest/getting_started.html#step7) provided by Renesas. 

After completion of the guide, the user is expected of following things.  
- The board setup is done.  
- The board is booted with microSD card, which contains the application file.  

### Instruction
1. On Board terminal, go to the `tvm` directory of the rootfs.
   - For RZ/V2H
    ```sh
    cd /home/root/tvm
    ```
   - For RZ/V2N
    ```sh
    cd /home/weston/tvm
    ```

2. Run the application.
   - For RZ/V2H
    ```sh
    ./app_camera_calibration <OpenCV>
    ```
   - For RZ/V2N
    ```sh
    su
    ./app_camera_calibration <OpenCV>
    exit    # After pressing ENTER key to terminate the application.
    ```
    >**Note :** For RZ/V2N AI SDK v6.00 and later, you need to switch to the root user with the 'su' command when running an application.<br>
    This is because when you run an application from a weston-terminal, you are switched to the "weston" user, which does not have permission to run the /dev/xxx device used in the application.<br>
    
    - OpenCV options
        |Value  |Description              |
        |-------|-------------------------|
        |0      | Run OpenCV on the CPU   |
        |1      | Run OpenCV on the DRP   |
    >**Note :** The application will not work with values other than 0 or 1.

3. Following window shows up on HDMI screen.  

    |RZ/V2H EVK (OpenCV:CPU) | RZ/V2N EVK (OpenCV:CPU) |
    |:---|:---|
    |<img src=./img/remap_v2h_cpu_normal.png width=350>  | <img src=./img/remap_v2n_cpu_normal.png width=350>  |

    On application window, following information is displayed.  
    - Camera capture  
    - Remap result
    - Camera capture size
    - OpenCV mode
    - Remap time 
       
4. Following window shows up on HDMI screen in Flip mode by executing the command below.
    ```sh
    ./app_camera_calibration <OpenCV> <flip>
    ```
    - flip options
        |Value  |Description                       |
        |-------|----------------------------------|
        |0      | The displayed image is not flipped.  |
        |1      | The displayed image is flipped.<br>The image is displayed with the left and right sides reversed.         |
   >**Note :** The application will not work with values other than 0 or 1.

   The flip function is useful when you want to face the camera and move an object while looking at the display. When you move the real-world object, the object on the display will move in the same direction.

5. Following window shows up on HDMI screen in Calibration  mode by ececuting the command below.

    ```sh
    ./app_camera_calibration <OpenCV> <flip> <calibration>
    ```
    - calibration options
        |Value  |Description                  |
        |-------|-----------------------------|
        |0      | Do not perform calibration  |
        |1      | Perform calibration         |
   >**Note :** The application will not work with values other than 0 or 1.
   
    |RZ/V2H EVK | RZ/V2N EVK |
    |:---|:---|
    |<img src=./img/remap_v2h_calib.png width=350>  |<img src=./img/remap_v2n_calib.png width=350>  |

    On application window, following information is displayed.  
    - Camera capture  
    - Number of shots required for calibration
    - Input key for calibration
    - Input key to end the operation
    - Calibration times
    
    ### calibraiton Method
    - Perform calibration using the following method and create a calibration file.

    5-1. A [chess board](http://opencv.jp/sample/pics/chesspattern_7x10.pdf) is displayed in front of the camera.

    5-2. While photographing the chessboard, press the C key.

    5-3. Once the application recognizes the chessboard, the "Calibration times" in the upper right corner of the display will count up.

    5-4. Repeat steps 5-1 to 5-3 the number of times indicated on the display.
    >**Note:** Please take photos of the chessboard so that the entire board is visible in the camera at various angles.<br>

    5-5. After repeating the specified number of times, "Calculating Camera Calibration..." will be displayed in red and the calibration calculation will be performed.

    |Display during calibration calculation|
    |:---|
    |<img src=./img/remap_v2h_calib_end.png width=350>  |
    >**Note:** The same string is displayed for both V2H and V2N.<br>

    5-6. The calibration file will be output and read, and the screen will switch to the calibration execution screen.

6. To terminate the application, switch the application window to the terminal by pressing ENTER key on the terminal of the board.

7. Here is a table summarizing the possible command combinations:
   |No| OpenCV | Flip | Calibration |Command pattern                |
   |:---|:---|:---|:---|:---|
   |1 | CPU    | OFF  | OFF         |./app_camera_calibration 0     |
   |2 | CPU    | ON   | OFF         |./app_camera_calibration 0 1   |
   |3 | CPU    | OFF  | OFF         |./app_camera_calibration 0 0 0 |
   |4 | CPU    | OFF  | ON          |./app_camera_calibration 0 0 1 |
   |5 | CPU    | ON   | OFF         |./app_camera_calibration 0 1 0 |
   |6 | CPU    | ON   | ON          |./app_camera_calibration 0 1 1 |
   |7 | DRP    | OFF  | OFF         |./app_camera_calibration 1     |
   |8 | DRP    | ON   | OFF         |./app_camera_calibration 1 1   |
   |9 | DRP    | OFF  | OFF         |./app_camera_calibration 1 0 0 |
   |10| DRP    | OFF  | ON          |./app_camera_calibration 1 0 1 |
   |11| DRP    | ON   | OFF         |./app_camera_calibration 1 1 0 |
   |12| DRP    | ON   | ON          |./app_camera_calibration 1 1 1 |
>**Note1:** No.1 and 3 have the same content.<br>
>**Note2:** No.2 and 5 have the same content.<br>
>**Note3:** No.7 and 9 have the same content.<br>
>**Note4:** No.8 and 11 have the same content.<br>

## Application: Configuration  
### Image buffer size

|Board | Camera capture buffer size|HDMI output buffer size|
|:---|:---|:---|
|RZ/V2H EVK| VGA (640x480) in YUYV format  | FHD (1920x1080) in BGRA format  |
|RZ/V2N EVK | VGA (640x480) in YUYV format  | FHD (1920x1080) in BGRA format  |


<!-- > **Note:** This application allocates the DRP-AI input buffer with **640x640** resolution in order to maintain the same aspect ratio with **416x416** square size of YOLOv3 input shape after the resize pre-processing. --> 
  
Following is the buffer flow.  

<img src=./img/remap_buffer_flow.png width="800">

## License
- 3-clause BSD

The license is included in the folder for each OSS.<br>
For this application, see following directory.
| Software | License |
| ------------ | --------- |
| OpenCV Accelerator | [3-clause BSD](licenses/opencva/LICENSE) |
| DRP Driver | [GPL-2.0](licenses/drp-driver/LICENSE)  |
