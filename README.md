English| [简体中文](./README_cn.md)

# Function Introduction

The elevation_net package is an elevation network detection algorithm example developed using the hobot_dnn package. It utilizes elevation network models and indoor data on the Horizon X3 development board for model inference using the BPU processor.
This package can subscribe to topics of type sensors/msg/Image (encoding must be "nv12"). It detects the pixel-based depth and height information of the image, processes the depth and height information, and publishes it as a 3D point cloud topic of type PointCloud2. By default, this package reads local images for detection and publishes the data obtained from AI inference.

The supported indoor environment detection information by the algorithm is as follows:

```
1. Height
2. Depth
```

The package externally publishes the 3D positioning information as a PointCloud2 type topic. Users can subscribe to the published PointCloud2 data for application development.
PointCloud2 is a built-in data structure in ROS, located in sensor_msg, where each point cloud in cloud contains x, y, z, and height, representing the three-dimensional coordinates and height information of the point cloud relative to the camera coordinate system.
The description is as follows:

```
header: // Header information of the point cloud
  seq: 963 //
  stamp: // Timestamp
    secs: 1653692850
    nsecs: 453970527
  frame_id: "camera"
height: 1 // If the cloud is unordered, height is 1
width: 518400 // The length of the point cloud
fields: // sensor_msgs/PointField[] fields 
  - name: "x" // x-axis coordinate
    offset: 0
    datatype: 7 // uint8 INT8 = 1
                // uint8 UINT8 = 2
                // uint8 INT16 = 3
                // uint8 UINT16 = 4
                // uint8 INT32 = 5
                // uint8 UINT32 = 6
                // uint8 FLOAT32 = 7
                // uint8 FLOAT64 = 8
    count: 1
  - name: "y" // y-axis coordinate
    offset: 4
    datatype: 7
    count: 1
  - name: "z" // z-axis coordinate
    offset: 8
    datatype: 7
    count: 1
  - name: height // Height
    offset: 12
    datatype: 7
    count: 1

is_bigendian: False
point_step: 16 // Length of a point in bytes
row_step: 8294400 // Length of a row in bytes
data: [ .......................................................... ] // Actual point data, size is (row_step*height)
is_dense: True // No invalid points
```

Support compiling on X3 Ubuntu system and using docker cross-compilation on PC.

#### Compilation Options

1. CV_BRIDGE_PKG

- Enable switch for cv_bridge pkg dependency, default is off, use `-DCV_BRIDGE_PKG=ON` command during compilation to enable.
- If enabled, compilation and execution will depend on cv_bridge pkg, supporting model inference with subscribed RGB8 and NV12 format images.
- If disabled, compilation and execution will not depend on cv_bridge pkg, only supporting model inference with subscribed NV12 format images.

### Compilation of X3 Version on Ubuntu Board

1. Compilation Environment Verification

- ROS environment variables have been set in the current compilation terminal: `source /opt/ros/foxy/setup.bash`.
- ROS2 compilation tool colcon has been installed. If the installed ROS does not include colcon, it needs to be manually installed. Installation command for colcon: `apt update; apt install python3-colcon-common-extensions`
- dnn node package has been installed
- cv_bridge package has been installed (installation method in Dependency section)
- ai_msgs has been installed

2. Compilation

- Compilation command: `colcon build --packages-select elevation_net`

### Docker Cross-compilation of X3 Version

1. Compilation Environment Verification

   - Compilation in docker, with TogetherROS already installed in docker. For docker installation, cross-compilation instructions, TogetherROS compilation and deployment instructions, refer to the README.md in the robot development platform robot_dev_config repo.

2. Compilation

  - Compilation command:

  ```
  export TARGET_ARCH=aarch64
  export TARGET_TRIPLE=aarch64-linux-gnu
  export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

  colcon build --packages-select elevation_net \
    --merge-install \
    --cmake-force-configure \
    --cmake-args \
    --no-warn-unused-cli \
    -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
  ```

### Compilation of X86 Version on X86 Ubuntu System1. Compilation environment confirmation

   - x86 Ubuntu version: Ubuntu 20.04

2. Compilation

   - Compilation command:

   ```
   colcon build --packages-select elevation_net  \
      --merge-install \
      --cmake-args \
      -DPLATFORM_X86=ON \
      -DTHIRD_PARTY=`pwd`/../sysroot_docker \
   ```

## Notes

# Usage Guide

## Dependencies

- elevation_net package: Publishes PointCloud2 data

## Parameters

| Parameter Name        | Type        | Explanation                                | Required | Supported Configurations | Default Value                |
| ---------------------- | ----------- | --------------------------------------------| -------- | ------------------------ | ---------------------------- |
| config_file_path      | std::string | Path to configuration file used for inference | No      | Configure based on actual file path | ./config          |
| feed_image            | std::string | Image used for inference | No | Configure based on actual path | ./config/images/charging_base.png |

## Execution

After successful compilation, copy the generated install directory to the Horizon X3 development board (if compiling on X3, ignore the copy step), and run the following command:

### **X3 Ubuntu**

To run, using ros2 run:

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# The path in the config is an example model. Copy based on the actual installation path
# For onboard compilation (without --merge-install compilation option), copy with command cp -r install/PKG_NAME/lib/PKG_NAME/config/ ., where PKG_NAME is the specific package name.
cp -r install/lib/elevation_net/config/ .

# Start the elevation_net package
ros2 run elevation_net elevation_net --ros-args -p config_file_path:=./config -p feed_image:=./config/images/charging_base.png --ros-args --log-level info
```

### Running Method 2, Starting with Launch Files:

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# The model used in the config is for demonstration purposes, please copy according to the actual installation path
# If compiling on the board (without the --merge-install option), the copy command should be cp -r install/PKG_NAME/lib/PKG_NAME/config/. where PKG_NAME is the specific package name.
cp -r install/lib/elevation_net/config/ .

# Start the launch file
ros2 launch install/share/elevation_net/launch/elevation_net.launch.py

```

### **X3 Linux**

```
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# The model used in the config is for demonstration purposes, please copy according to the actual installation path
cp -r install/lib/elevation_net/config/ .

# Start the elevation network detection package
./install/lib/elevation_net/elevation_net

```

### **X86 Ubuntu**

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash

# The model used in the config is for demonstration purposes, please copy according to the actual installation path
cp -r install/lib/elevation_net/config/ .

# Start the elevation network detection package
./install/lib/elevation_net/elevation_net

```

# Analysis of Results

## Display of Results

```
[16:15:17:520]root@ubuntu:/userdata# ros2 run elevation_net elevation_net
[16:15:18:976][WARN] [1655108119.406738772] [example]: This is dnn node example!
[16:15:19:056][WARN] [1655108119.475098438] [elevation_dection]: Parameter:
[16:15:19:056]config_file_path_:./config
[16:15:19:056] model_file_name_: ./config/elevation.hbm
[16:15:19:058]feed_image:./config/images/charging_base.png
[16:15:19:058][INFO] [1655108119.475257138] [dnn]: Node init.
[16:15:19:058][INFO] [1655108119.475309553] [elevation_dection]: Set node para.
[16:15:19:058][INFO] [1655108119.475370258] [dnn]: Model init.
[16:15:19:058][BPU_PLAT]BPU Platform Version(1.3.1)!
[16:15:19:095][HBRT] set log level as 0. version = 3.13.27
[16:15:19:095][DNN] Runtime version = 1.8.4_(3.13.27 HBRT)
[16:15:19:133][000:000] (model.cpp:244): Empty desc, model name: elevation, input branch:0, input name:inputquanti-_output
[16:15:19:133][000:000] (model.cpp:244): Empty desc, model name: elevation, input branch:1, input name:inputquanti2-_output
[16:15:19:134][000:000] (model.cpp:313): Empty desc, model name: elevation, output branch:0, output name:output_block1quanticonvolution0_conv_output
[16:15:19:134][INFO] [1655108119.528437276] [dnn]: The model input 0 width is 960 and height is 512
[16:15:19:134][INFO] [1655108119.528535271] [dnn]: The model input 1 width is 960 and height is 512
[16:15:19:134][INFO] [1655108119.528598393] [dnn]: Task init.
[16:15:19:135][INFO] [1655108119.530435806] [dnn]: Set task_num [2]
[16:15:19:135][INFO] [1655108119.530549051] [elevation_dection]: The model input width is 960 and height is 512
[16:15:19:158][INFO] [1655108119.559583836] [elevation_dection]: read image: ./config/images/charging_base.png to detect
[16:15:19:299][WARN] [1655108119.731084555] [elevation_dection]: start success!!!
[16:15:19:351][INFO] [1655108119.779924566] [elevation_net_parser]: fx_inv_: 0.000605
[16:15:19:383][INFO] [1655108119.780357879] [elevation_net_parser]: fy_inv_: 0.000604
[16:15:19:383][INFO] [1655108119.780576493] [elevation_net_parser]: cx_inv_: -0.604389
[16:15:19:383][INFO] [1655108119.780654031] [elevation_net_parser]: cy_inv_: -0.318132
[16:15:19:384][INFO] [1655108119.780751527] [elevation_net_parser]: nx_: 0.000000
[16:15:19:384][INFO] [1655108119.780858063] [elevation_net_parser]: ny_: 0.000000
[16:15:19:384][INFO] [1655108119.780962558] [elevation_net_parser]: nz_: 1.000000
[16:15:19:384][INFO] [1655108119.781067928] [elevation_net_parser]: camera_height: 1.000000
[16:15:19:385][INFO] [1655108119.781833267] [elevation_net_parser]: model out width: 480, height: 256
[16:15:19:416][INFO] [1655108119.808395254] [elevation_net_parser]: depth: 998.000000
[16:15:19:416][INFO] [1655108119.808593786] [elevation_net_parser]: height: -42.699909
[16:15:19:416][INFO] [1655108119.808644533] [elevation_net_parser]: depth: 998.000000
[16:15:19:416][INFO] [1655108119.808692531] [elevation_net_parser]: height: -25.339746
[16:15:19:416][INFO] [1655108119.808739279] [elevation_net_parser]: depth: 998.000000
[16:15:19:416][INFO] [1655108119.808785527] [elevation_net_parser]: height: -22.111366
[16:15:19:416][INFO] [1655108119.808832774] [elevation_net_parser]: depth: 998.000000
[16:15:19:416][INFO] [1655108119.808878606] [elevation_net_parser]: height: -25.339746
[16:15:19:416][INFO] [1655108119.808925645] [elevation_net_parser]: depth: 998.000000
[16:15:19:416][INFO] [1655108119.808971809] [elevation_net_parser]: height: -21.989540
[16:15:19:416][INFO] [1655108119.809017516] [elevation_net_parser]: depth: 998.000000
[16:15:19:416][INFO] [1655108119.809063138] [elevation_net_parser]: height: -48.303890
[16:15:19:416][INFO] [1655108119.809109678] [elevation_net_parser]: depth: 998.000000
[16:15:19:416][INFO] [1655108119.809155592] [elevation_net_parser]: height: -32.527466
[16:15:19:416][INFO] [1655108119.809202548] [elevation_net_parser]: depth: 998.000000
[16:15:19:416][INFO] [1655108119.809247880] [elevation_net_parser]: height: -32.710201
[16:15:19:416][INFO] [1655108119.809294669] [elevation_net_parser]: depth: 998.000000
[16:15:19:416][INFO] [1655108119.809340542] [elevation_net_parser]: height: -33.014767
[16:15:19:417][INFO] [1655108119.809387165] [elevation_net_parser]: depth: 998.000000
[16:15:19:417][INFO] [1655108119.809433454] [elevation_net_parser]: height: -35.451283
[16:15:19:417][INFO] [1655108119.809480202] [elevation_net_parser]: depth: 998.000000
[16:15:19:417][INFO] [1655108119.809527158] [elevation_net_parser]: height: -38.192360
[16:15:19:417][INFO] [1655108119.809573906] [elevation_net_parser]: depth: 998.000000[16:15:19:417][INFO] [1655108119.809619820] [elevation_net_parser]: height: -34.233025  
[16:15:19:417][INFO] [1655108119.809667235] [elevation_net_parser]: depth: 998.000000  
[16:15:19:417][INFO] [1655108119.809713357] [elevation_net_parser]: height: -34.233025  
[16:15:19:417][INFO] [1655108119.809759397] [elevation_net_parser]: depth: 998.000000  
[16:15:19:417][INFO] [1655108119.809805686] [elevation_net_parser]: height: -33.014767  
[16:15:19:417][INFO] [1655108119.809852643] [elevation_net_parser]: depth: 998.000000  
[16:15:19:417][INFO] [1655108119.809899307] [elevation_net_parser]: height: -34.354851  
[16:15:19:417][INFO] [1655108119.809945930] [elevation_net_parser]: depth: 998.000000  
[16:15:19:417][INFO] [1655108119.809991844] [elevation_net_parser]: height: -35.024891  
[16:15:19:417][INFO] [1655108119.810038384] [elevation_net_parser]: depth: 998.000000  
[16:15:19:417][INFO] [1655108119.810084715] [elevation_net_parser]: height: -41.298916  
[16:15:19:417][INFO] [1655108119.810131296] [elevation_net_parser]: depth: 998.000000  
[16:15:19:417][INFO] [1655108119.810268706] [elevation_net_parser]: height: -33.745720  
[16:15:19:417][INFO] [1655108119.810317745] [elevation_net_parser]: depth: 998.000000  
[16:15:19:417][INFO] [1655108119.810364285] [elevation_net_parser]: height: -32.710201  
[16:15:19:417][INFO] [1655108119.810410741] [elevation_net_parser]: depth: 998.000000  
```

# FAQ
