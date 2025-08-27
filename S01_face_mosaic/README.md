# Face Mosaic

## Application: Overview
This application is designed to detect and mosaic heads in the capture image of Camera and displays the result on HDMI screen.

This software could be useful in a variety of settings, such as retail stores, museums, and events.  
The AI model used for the sample application is [YOLOV3](https://arxiv.org/pdf/1804.02767.pdf).

### Supported Product  
<table>
    <tr>
      <th>Product</th>
      <th>Supported AI SDK version</th>
    </tr>
    <tr>
      <td>RZ/V2L Evaluation Board Kit (RZ/V2L EVK)</td>
      <td>RZ/V2L AI SDK v5.00</td>
    </tr>
    <tr>
      <td>RZ/V2H Evaluation Board Kit (RZ/V2H EVK)</td>
      <td>RZ/V2H AI SDK v5.20</td>
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
      <td>RZ/V2L EVK</td>
      <td rowspan="2">USB camera</td>
      <td rowspan="2">HDMI</td>
    </tr>
    <tr>
      <td >RZ/V2H EVK</td>
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
      <td rowspan="4">RZ/V2L</td>
      <td>RZ/V2L EVK</td>
      <td>Evaluation Board Kit for RZ/V2L.<br>Includes followings.
        <ul class="mb-1">
          <li>MicroUSB to Serial Cable for serial communication.</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>AC Adapter</td>
      <td>USB Power Delivery adapter for the board power supply.</td>
    </tr>
    <tr>
      <td>MicroHDMI Cable</td>
      <td>Used to connect the HDMI Monitor and the board.<br>
      RZ/V2L EVK has microHDMI port.</td>
    </tr>
    <tr>
      <td>USB Camera</td>
      <td>Used as a camera input source.</td>
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
      <td>USB Camera</td>
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

|RZ/V2L EVK | RZ/V2H EVK |
|:---|:---|
|<img src=./img/facemosaic_hw_conf_v2l.png width=600>|<img src=./img/facemosaic_hw_conf_v2h.png width=600> |

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
    |RZ/V2L EVK|`rzv2l_ai_sdk_container`  |
    |RZ/V2H EVK|`rzv2h_ai_sdk_container`  |

    >**Note:** Docker environment is required for building the sample application.
<!--    >**Note 2:** Since RZ/V2N is a brother chip of RZ/V2H, the same environment can be used. -->


### Application File Generation
1. On your host machine, copy the repository from the GitHub to the desired location. 
    1. It is recommended to copy/clone the repository on the `data` folder, which is mounted on the Docker container. 
    ```sh
    cd <path_to_data_folder_on_host>/data
    git clone https://github.com/renesas-rz/rzv_sample_apps/tree/apps/S01_face_mosaic
    ```
    >Note: This command will download the whole repository, which include all other applications.  
    If you have already downloaded the repository of the same version, you may not need to run this command.  

2. Run (or start) the docker container and open the bash terminal on the container.  
E.g., for RZ/V2L, use the `rzv2l_ai_sdk_container` as the name of container created from  `rzv2l_ai_sdk_image` docker image.  
    > Note that all the build steps/commands listed below are executed on the docker container bash terminal.  

3. Set your clone directory to the environment variable.  
    ```sh
    export PROJECT_PATH=/drp-ai_tvm/data
    ```
4. Go to the application source code directory.  
    ```sh
    cd ${PROJECT_PATH}/S01_face_mosaic/src
    ```

5. Create and move to the `build` directory.
    ```sh
    mkdir -p build && cd build
    ```
6. Build the application by following the commands below. 
  
    - For RZ/V2L,
      ```sh
      cmake -DCMAKE_TOOLCHAIN_FILE=./toolchain/runtime.cmake ..
      make -j$(nproc)
      ```
    - For RZ/V2H,
      ```sh
      cmake -DCMAKE_TOOLCHAIN_FILE=./toolchain/runtime.cmake -DV2H=ON ..
      make -j$(nproc)
      ```
      
7. The following application file would be generated in the `${PROJECT_PATH}/S01_face_mosaic/src/build` directory
    - face_mosaic_app


## Application: Deploy Stage
### Prerequisites
This section expects the user to have completed Step 7-1 of [Getting Started Guide](https://renesas-rz.github.io/rzv_ai_sdk/latest/getting_started.html#step7) provided by Renesas. 

After completion of the guide, the user is expected of following things.
- microSD card setup is done.

### File Configuration
For the ease of deployment all the deployable files and folders are provided in following folders.  
|Board | `EXE_DIR` |
|:---|:---|
|RZ/V2L EVK|[exe_v2l](./exe_v2l)  |
|RZ/V2H EVK|[exe_v2h](./exe_v2h)  |
<!-- > Note: Since RZ/V2N is a brother chip of RZ/V2H, the same execution environment can be used.   -->
  
Each folder contains following items.
|File | Details |
|:---|:---|
|licenses | License information. <br>Not necessary for running application. |
|face_mosaic_yolov3 | Model object files for deployment.<br>Pre-processing Runtime Object files included. |
|labels.txt | Label list for Head Detection. |
|face_mosaic_app | application file. |

### Instruction
1. Register the working directory path to an environment variable.<br>
The environment variable WORK is the working directory path that you set in Step 4-2 of [Getting Started Guide](https://renesas-rz.github.io/rzv_ai_sdk/latest/getting_started.html#step4).

    ```sh
    export WORK=<path to the working directory>/ai_sdk_work
    export APPS_PATH=${WORK}/ai_sdk_setup/data
    ```

2. Run following commands to download the necessary file.  
Replace each variable according to your board.  
    ```sh
    cd ${APPS_PATH}/S02_face_mosaic/<EXE_DIR>/face_mosaic_yolov3
    wget <URL>/<SO_FILE>
    ```
    |Board | `EXE_DIR` |`URL` |`SO_FILE` |File Location |
    |:---|:---|:---|:---|:---|
    |RZ/V2L EVK|[exe_v2l](./exe_v2l)  |<span style="font-size: small">`https://github.com/renesas-rz/rzv_sample_apps/releases/download/v1.00/`</span>  |<span style="font-size: small">`S02_face_mosaic_deploy_tvm_v2l-v250.so`</span> |[Release v1.00](https://github.com/renesas-rz/rzv_sample_apps/releases/tag/v1.00/)  |
    |RZ/V2H EVK|[exe_v2h](./exe_v2h)  |<span style="font-size: small">`https://github.com/renesas-rz/rzv_sample_apps/releases/download/v1.00/`</span>  |<span style="font-size: small">`S02_face_mosaic_deploy_tvm_v2h-v230.so`</span> |[Release v1.00](https://github.com/renesas-rz/rzv_sample_apps/releases/tag/v1.00/)  |
<!--    > Note: Since RZ/V2N is a brother chip of RZ/V2H, the same execution environment can be used.  -->

    - E.g., for RZ/V2L EVK, use following commands.
        ```sh
        cd ${APPS_PATH}/S02_face_mosaic/exe_v2l/face_mosaic_yolov3
        wget https://github.com/renesas-rz/rzv_sample_apps/releases/download/v1.00/S02_face_mosaic_deploy_tvm_v2l_v250.so
        ```
3. Rename the `S02_face_mosaic_deploy_*.so` to `deploy.so`.
    ```sh
    mv <SO_FILE> deploy.so
    ```
4. Copy the following files to the `/home/root/tvm` directory (RZ/V2L, RZ/V2H) of the rootfs (SD Card) for the board.
    |File | Details |
    |:---|:---|
    |All files in `EXE_DIR` directory | Including `deploy.so` file. |
    |`face_mosaic_app` application file | Generated the file according to [Application File Generation](#application-file-generation) |

5. Folder structure in the rootfs (SD Card) is shown below.<br>
   Check if `libtvm_runtime.so` exists in the rootfs directory (SD card) on the board.
   
   - For RZ/V2L and RZ/V2H
    ```
    |-- usr/
    |   `-- lib64/
    |       `-- libtvm_runtime.so
    |
    `-- home/
        `-- root/
            `-- tvm/ 
                |-- face_mosaic_yolov3/
                |   |-- preprocess
                |   |-- deploy.json
                |   |-- deploy.params
                |   `-- deploy.so
                |-- labels.txt
                `-- face_mosaic_app
    ```

>**Note:** The directory name could be anything instead of `tvm`. If you copy the whole `EXE_DIR` folder on the board, you are not required to rename it `tvm`.

## Application: Run Stage

### Prerequisites
This section expects the user to have completed Step 7-3 of [Getting Started Guide](https://renesas-rz.github.io/rzv_ai_sdk/latest/getting_started.html#step7) provided by Renesas. 

After completion of the guide, the user is expected of following things.  
- The board setup is done.  
- The board is booted with microSD card, which contains the application file.  

### Instruction
1. On Board terminal, go to the `tvm` directory of the rootfs.
   - For RZ/V2L and RZ/V2H
    ```sh
    cd /home/root/tvm
    ```

2. Run the application.
   - For RZ/V2L and RZ/V2H
    ```sh
    ./face_mosaic_app USB
    ```
3. Following window shows up on HDMI screen.  

    |RZ/V2L EVK | RZ/V2H EVK |
    |:---|:---|
    |<img src=./img/facemosaic_v2l_normal.png width=350>| <img src=./img/facemosaic_v2h_normal.png width=350>  |

    On application window, following information is displayed.  
    - Camera capture  
    - Head Detection result (with mosaic)
    - Number of detected heads  
    - Processing time  
        - AI Total Time: Sum of all processing time below.  
        - Pre-Proc: Processing time taken for AI pre-processing. 
        - Inference: Processing time taken for AI inference.   
        - Post-Proc: Processing time taken for AI post-processing.<br>(excluding the time for drawing on HDMI screen).
        - Image Proc: Processing time taken for image processing.  

4. Following window shows up on HDMI screen in Bounding Box mode by ececuting the command below.
    ```sh
    ./face_mosaic_app USB BOX
    ```

    |RZ/V2L EVK |RZ/V2H EVK |
    |:---|:---|
    |<img src=./img/facemosaic_v2l_box.png width=350>  |<img src=./img/facemosaic_v2h_box.png width=350>  |

5. Following window shows up on HDMI screen in Flip mode by ececuting the command below.
    ```sh
    ./face_mosaic_app USB FLIP
    ```
    The image is displayed with the left and right sides reversed.<br>
    The flip function is useful when you want to face the camera and move an object while looking at the display.<br>
    When you move the real-world object, the object on the display will move in the same direction.
    
   >**Note:** Bounding Box mode and Flip mode can be used together.<br>
   E.g.) ./face_mosaic_app USB BOX FLIP<br>
   It is okay if "BOX" and "FLIP" are swapped.<br>

6. To terminate the application, switch the application window to the terminal by pressing ENTER key on the terminal of the board.


## Application: Configuration 
### AI Model
- YOLOv3: [Darknet](https://pjreddie.com/darknet/yolo/)  
Dataset: *[HollywoodHeads](https://www.di.ens.fr/willow/research/headdetection/) *[Head_data](https://www.kaggle.com/datasets/houssad/head-data) *[RGBD_Indoor_Dataset](https://drive.google.com/file/d/1fOub9LcNqfDlr-mEcdnenAJWw-rqWGmG/view)  
Input size: 1x3x416x416  
Output1 size: 1x13x13x18  
Output2 size: 1x26x26x18  
Output3 size: 1x52x52x18   

### AI inference time
|Board | AI inference time|
|:---|:---|
|RZ/V2L EVK| Approximately 335ms  |
|RZ/V2H EVK | Approximately 25ms  |

### Processing

|Processing | Details |
|:---|:---|
|Pre-processing | Processed by DRP-AI. <br> |
|Inference | Processed by DRP-AI and CPU. |
|Post-processing | Processed by CPU. |


### Image buffer size

|Board | Camera capture buffer size|HDMI output buffer size|
|:---|:---|:---|
|RZ/V2L EVK| VGA (640x480) in YUYV format  | FHD (1920x1080) in BGRA format  |
|RZ/V2H EVK | VGA (640x480) in YUYV format  | FHD (1920x1080) in BGRA format  |


> **Note:** This application allocates the DRP-AI input buffer with **640x640** resolution in order to maintain the same aspect ratio with **416x416** square size of YOLOv3 input shape after the resize pre-processing.  
  
Following is the buffer flow.  

<img src=./img/facemosaic_buffer_flow.png width="800">


## Appendix
- The memory map of DRP-AI for the V2L and V2H application is shown below.
    |RZ/V2L DRP-AI mempry map |RZ/V2H DRP-AI mempry map |
    |:---|:---|
    |<img src=./img/facemosaic_drpai_memorymap_v2l.png width=350>  |<img src=./img/facemosaic_drpai_memorymap_v2h.png width=350>  |

- #### The memory area ①
  The memory is used for preprocessing of DRP-AI and is automatically defined by the preruntime.Load function in [face_mosaic.cpp](./src/face_mosaic.cpp).<br>
  Please check the files in the table below for the contents of the function.<br>

  |Board |Function |File |
  |:---|:---|:---|
  |RZ/V2L EVK|PreRuntime::Load( )|[PreRuntime.cpp](./src/PreRuntime.cpp)       |
  |RZ/V2H EVK|PreRuntime::Load( )|[PreRuntimeV2H.cpp](./src/PreRuntimeV2H.cpp) |
  
- #### The memory area ② 
  The memory is used for the main processing of DRP-AI and is automatically defined by the runtime.LoadModel function in [face_mosaic.cpp](./src/face_mosaic.cpp).<br>
  Please check the files in the table below for the contents of the function.<br>

  |Function |File |
  |:---|:---|
  |MeraDrpRuntimeWrapper::LoadModel( )|[MeraDrpRuntimeWrapper.cpp](./src/MeraDrpRuntimeWrapper.cpp) |

If the main processing (AI model) or preprocessing changes, the total size of area ① and area ② must not exceed the allocated DRP-AI memory size (0x20000000).<br>
If it does, extend the value of DRP-AI memory size allocated in the device tree.<br>

### V2L has the following features:<br>
(1) The memory area ① is reserved by an offset address.<br>
The offset address value is defined by "DRPAI_MEM_OFFSET" in [define.h](./src/define.h)<br>
(2) "DRPAI_MEM_OFFSET" is used as the second argument of runtime.LoadModel( ) function in [face_mosaic.cpp](./src/face_mosaic.cpp).<br>
The memory area ② is assigned from the address set by the second argument.<br>
(3) If the total size of memory area ① is changed, change the offset address as necessary.<br>
> **Note:** The offset address must be aligned with 64 bytes.

The following files are set in the memory map:<br>
|Area | `File` |
|:---|:---|
|①|[preprocess](./exe_v2l/face_mosaic_yolov3/preprocess)|
|②|[deploy.json, deploy.param](./exe_v2l/face_mosaic_yolov3/), [S02_face_mosaic_deploy_tvm_v2l_v250.so](https://github.com/renesas-rz/rzv_sample_apps/releases/tag/v1.00/) |

Please refer to the file below for the memory size of Area ①.<br>
|Board | `File` |
|:---|:---|
|RZ/V2L EVK|[pp_addrmap_intm.txt](./exe_v2l/face_mosaic_yolov3/preprocess/pp_addrmap_intm.txt)  |
> **Note1 :** The total size of memory area ① is the sum of the hexadecimal address and size on the last line.<br>
> **Note2 :** In this application, the size of area ① is "0x6ba250" (6ba100 + 150). (Image below)<br>
Therefore, the offset value is set to "0x700000".<br>
<img src=./img/facemosaic_drpai_memory1_size_v2l.PNG width=120>

Please refer to the file below for the memory size of area ②.<br>
**[How to read the Compile log](https://github.com/renesas-rz/rzv_drp-ai_tvm/blob/main/how-to/tips/how-to-read-log.md)**
> **Note1 :** The size of area ② corresponds to "DRP-AI memory area" in the article.<br>
> **Note2 :** In this application, the size of area ② is 224 Mbyte. (Image below)<br>
<img src=./img/facemosaic_drpai_memory2_size_v2l.PNG width=700>

### V2H has the following features:<br>
(1) The memory area ① is set from the bottom of the memory map.<br>
(2) The memory area ② is set from the beginning of the memory map.<br>

The following files are set in the memory map:<br>
|Area | `File` |
|:---|:---|
|①|[preprocess](./exe_v2h/face_mosaic_yolov3/preprocess)|
|②|[deploy.json, deploy.param](./exe_v2h/face_mosaic_yolov3/), [S02_face_mosaic_deploy_tvm_v2h_v230.so](https://github.com/renesas-rz/rzv_sample_apps/releases/tag/v1.00/)  |

Please refer to the file below for the memory size of Area ①.<br>
|Board | `File` |
|:---|:---|
|RZ/V2H EVK|[addr_map.txt](./exe_v2h/face_mosaic_yolov3/preprocess/addr_map.txt)  |
> **Note1 :** The total size of memory area ① is the sum of the hexadecimal address and size on the last line.<br>
> **Note2 :** In this application, the size of area ① is "0x691010" (690f00 + 110). (Image below)<br>
<img src=./img/facemosaic_drpai_memory1_size_v2h.PNG width=120><br>

> **Note3 :** In V2H, the starting address of area ① is calculated by reading the starting address and size of DRP-AI in the device tree and subtracting the size of area ①.<br>
The start address is calculated automatically using the following formula( PreRuntime::Load( ) in [PreRuntimeV2H.cpp](./src/PreRuntimeV2H.cpp)):<br>
(1) area ①.size = drp_desc.start_address + drp_desc.size;<br>
(2) Occupied size = (area ①.size + 0xffffff) & 0xff000000;<br>
(3) area ①.start_address = drpai.start_address + drpai.size - Occupied size;<br>
E.g. ) In this application, the values are as follows:<br>
(1) area ①.size = 0x690f00 + 0x110 = 0x691010<br>
(2) Occupied size = (0x691010 + 0xffffff) & 0xff000000 = 0x1000000<br>
(3) area ①.start_address = 0x240000000 + 0x20000000 - 0x1000000 = 0x25f000000<br>

Please refer to the file below for the memory size of area ②.<br>
**[How to read the Compile log](https://github.com/renesas-rz/rzv_drp-ai_tvm/blob/main/how-to/tips/how-to-read-log.md)**
> **Note1 :** The size of area ② corresponds to "DRP-AI memory area" in the article.<br>
> **Note2 :** In this application, the size of area ② is 101 Mbyte. (Image below)<br>
<img src=./img/facemosaic_drpai_memory2_size_v2h.PNG width=700>

## License
- 3-clause BSD

For this application, see following directory.. 
| Software | License |
| ------------ | --------- |
| OpenCV Accelerator | [3-clause BSD](licenses/opencva/LICENSE) |
| DRP Driver | [GPL-2.0](licenses/drp-driver/LICENSE)  |

For AI model, see following directory..  
|Board | AI Model | License directory|
|:---|:---|:---|
|RZ/V2L EVK| YOLOv3  | `exe_v2l/licenses`  |
|RZ/V2H EVK| YOLOv3  | `exe_v2h/licenses`  |
