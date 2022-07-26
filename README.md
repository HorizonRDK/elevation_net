# 功能介绍

elevation_net package是使用hobot_dnn package开发的高程网络检测算法示例，
在地平线X3开发板上使用高程网络模型和室内数据利用BPU处理器进行模型推理。
package订阅sensors/msg/Image(encoding必须为“nv12”)类型的话题，检测出Image基于像素的深度和高度信息，
同时package将深度和高度信息进行处理，发布为PointCloud2类型的3维点云话题。

算法支持的室内环境检测信息如下：

```
1. 高度
2. 深度
```
package对外发布包含3D定位信息的发布为PointCloud2类型话题，用户可以订阅发布的PointCloud2数据用于应用开发。
PointCloud2是ROS内置的数据结构，位于sensor_msg内，cloud内每个点云包含x, y, z, height，分别表示点云相对于相机坐标系的三维坐标以及高度信息。
描述如下所示：
````
header:  // 点云的头信息
  seq: 963 //
  stamp:  // 时间戳
    secs: 1653692850
    nsecs: 453970527
  frame_id: "camera"
height: 1   // If the cloud is unordered, height is 1
width: 518400  //点云的长度
fields:  //  sensor_msgs/PointField[] fields 
  - name: "x"  // x轴坐标
    offset: 0
    datatype: 7        // 	uint8 INT8    = 1
			//	uint8 UINT8   = 2
			//	uint8 INT16   = 3
			//	uint8 UINT16  = 4
			//	uint8 INT32   = 5
			//	uint8 UINT32  = 6
			//	uint8 FLOAT32 = 7
			//	uint8 FLOAT64 = 8
    count: 1
  - name: "y"  // y轴坐标
    offset: 4
    datatype: 7
    count: 1
  - name: "z"  // z轴坐标
    offset: 8
    datatype: 7
    count: 1
  - name: height  // 高度
    offset: 12
    datatype: 7
    count: 1

is_bigendian: False
point_step: 16 // Length of a point in bytes 一个点占的比特数 
row_step: 8294400 // Length of a row in bytes 一行的长度占用的比特数
data: [ .......................................................... ] //  Actual point data, size is (row_step*height)
is_dense: True // 没有非法数据点
````

# 编译

## 依赖库

- opencv:3.4.5

ros package：

- sensor_msgs
- cv_bridge
- dnn_node
- hbm_img_msgs
- ai_msgs

其中cv_bridge为ROS开源的package，需要手动安装，具体安装方法（仅限X3 Ubuntu系统上编译有效）：

```cpp
# 方法1，直接使用apt安装，以cv_bridge安装举例
sudo apt-get install ros-foxy-cv-bridge -y

# 方法2，使用rosdep检查并自动安装pkg编译的依赖项
# 安装ros pkg依赖下载⼯具rosdep
sudo apt-get install python3-pip
sudo pip install rosdep
sudo rosdep init
rosdep update
# 在ros的⼯程路径下执⾏安装依赖，需要指定pkg所在路径。默认为所有pkg安装依赖，也可以指定为某个pkg安装依赖
rosdep install -i --from-path . --rosdistro foxy -y
```

dnn_node是在地平线X3开发板上利用BPU处理器进行模型推理的pkg，定义在hobot_dnn中。

hbm_img_msgs为自定义的图片消息格式，用于shared mem场景下的图片传输，hbm_img_msgs pkg定义在hobot_msgs中，因此如果使用shared mem进行图片传输，需要下载hobot_msgs。

ai_msgs为自定义的消息格式，用于算法模型推理后，发布推理结果，ai_msgs pkg定义在hobot_msgs中。

## 开发环境

- 编程语言: C/C++
- 开发平台: X3/X86
- 系统版本：Ubuntu 20.0.4
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

## 编译

 支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式。

#### 编译选项

1、CV_BRIDGE_PKG

- cv_bridge pkg依赖的使能开关，默认关闭（OFF），编译时使用-DCV_BRIDGE_PKG=ON命令打开。
- 如果打开，编译和运行会依赖cv_bridge pkg，支持使用订阅到的rgb8和nv12格式图片进行模型推理。
- 如果关闭，编译和运行不依赖cv_bridge pkg，只支持使用订阅到的nv12格式图片进行模型推理。

2、BUILD_HBMEM

- shared mem（共享内存传输）使能开关，默认关闭（OFF），只有在docker中使用tros编译时才会打开（aarch64_toolchainfile.cmake中设置）。
- 如果打开，编译和运行会依赖hbm_img_msgs pkg，并且需要使用tros进行编译。
- 如果关闭，编译和运行不依赖hbm_img_msgs pkg，支持使用原生ros和tros进行编译。
- 对于shared mem通信方式，当前只支持订阅nv12格式图片。

### Ubuntu板端编译

1、编译环境确认

- 当前编译终端已设置ROS环境变量：`source /opt/ros/foxy/setup.bash`。
- 已安装ROS2编译工具colcon。安装的ROS不包含编译工具colcon，需要手动安装colcon。colcon安装命令：`apt update; apt install python3-colcon-common-extensions`
- 已安装dnn node package
- 已安装cv_bridge package（安装方法见Dependency部分）
- 已安装ai_msgs

2、编译

- 编译命令：`colcon build --packages-select elevation_net`
- 编译和运行会依赖cv_bridge pkg，不使用shared mem通信方式。支持使用订阅到的rgb8和nv12格式图片进行模型推理。


### Docker交叉编译

1. 编译环境确认

   - 在docker中编译，并且docker中已经安装好TogetherROS。docker安装、交叉编译说明、TogetherROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。

2. 编译

   - 编译命令：

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

## 注意事项

# 使用介绍

## 依赖

- elevation_net package：发布PointCloud2

## 参数

| 参数名                 | 类型        | 解释                                        | 是否必须 | 支持的配置           | 默认值                        |
| ---------------------- | ----------- | ------------------------------------------- | -------- | -------------------- | ----------------------------- |
| is_sync_mode           | int         | 同步/异步推理模式。0：异步模式；1：同步模式 | 否       | 0/1                  | 0                             |
| model_file_name        | std::string | 推理使用的模型文件                          | 否       | 根据实际模型路径配置 | config/elevation.hbm           |
| pointcloud_pub_topic_name  | std::string | 发布pointcloud2点云数据 | 是       | 根据实际部署环境配置 | /elevation_net/points |
| image_sub_topic_name   | std::string | 订阅nv12格式的sensors_msg::msg::Image图像，用于提取高度和深度信息     | 是       | 根据实际部署环境配置 | /image_raw     |

## 运行

编译成功后，将生成的install路径拷贝到地平线X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行：

### **Ubuntu**

运行方式1，使用ros2 run启动：

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# config中为示例使用的模型，根据实际安装路径进行拷贝
# 如果是板端编译（无--merge-install编译选项），拷贝命令为cp -r install/PKG_NAME/lib/PKG_NAME/config/ .，其中PKG_NAME为具体的package名。
cp -r install/lib/elevation_net/config/ .

# 启动图片发布pkg
ros2 run mipi_cam mipi_cam --ros-args -p out_format:=nv12 -p image_width:=960 -p image_height:=544 -p io_method:=shared_mem --log-level error &
# 启动jpeg图片编码&发布pkg
ros2 run hobot_codec hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=ros -p out_format:=jpeg -p sub_topic:=/hbmem_img -p pub_topic:=/image_jpeg --ros-args --log-level error &

# 启动高程网络pkg
ros2 run elevation_net elevation_net --ros-args -p shared_men:=1 -p config_file_path:=./config --ros-args --log-level info 

```
运行方式2，使用launch文件启动：
```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# config中为示例使用的模型，根据实际安装路径进行拷贝
# 如果是板端编译（无--merge-install编译选项），拷贝命令为cp -r install/PKG_NAME/lib/PKG_NAME/config/ .，其中PKG_NAME为具体的package名。
cp -r install/lib/elevation_net/config/ .

# 启动launch文件
ros2 launch install/share/elevation_net/launch/hobot_elevation_net.launch.py

```

### **Linux**

```
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# config中为示例使用的模型，根据实际安装路径进行拷贝
cp -r install/lib/mono2d_body_detection/config/ .

# 启动图片发布pkg
./install/lib/mipi_cam/mipi_cam --ros-args -p out_format:=nv12 -p image_width:=960 -p image_height:=544 -p io_method:=shared_mem --log-level error &
# 启动jpeg图片编码&发布pkg
./install/lib/hobot_codec/hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=ros -p out_format:=jpeg -p sub_topic:=/hbmem_img -p pub_topic:=/image_jpeg --ros-args --log-level error &
# 启动web展示pkg
./install/lib/websocket/websocket --ros-args -p image_topic:=/image_jpeg -p image_type:=mjpeg -p smart_topic:=/hobot_mono2d_body_detection --log-level error &

# 启动单目rgb人体、人头、人脸、人手框和人体关键点检测pkg
./install/lib/mono2d_body_detection/mono2d_body_detection

```


# 结果分析

## 结果展示

```
[16:48:10:123]root@ubuntu:/userdata# ros2 run elevation_net elevation_net --ros-args -p shared_men:=1 -p config_file_path:=./config --ros-args --log-level info 
[16:48:16:539][WARN] [1653986896.992123541] [example]: This is dnn node example!
[16:48:16:788][WARN] [1653986897.229717401] [elevation_dection]: Parameter:
[16:48:16:788]config_file_path_:./config
[16:48:16:788]shared_men:0
[16:48:16:789] is_sync_mode_: 1
[16:48:16:789] model_file_name_: ./config/elevation.hbm
[16:48:16:824][INFO] [1653986897.230397724] [dnn]: Node init.
[16:48:16:825][INFO] [1653986897.230647470] [elevation_dection]: Set node para.
[16:48:16:825][INFO] [1653986897.230904133] [dnn]: Model init.
[16:48:16:825][BPU_PLAT]BPU Platform Version(1.3.1)!
[16:48:16:825][HBRT] set log level as 0. version = 3.13.27
[16:48:16:826][DNN] Runtime version = 1.8.4_(3.13.27 HBRT)
[16:48:16:941][000:000] (model.cpp:244): Empty desc, model name: elevation, input branch:0, input name:inputquanti-_output
[16:48:16:974][000:000] (model.cpp:244): Empty desc, model name: elevation, input branch:1, input name:inputquanti2-_output
[16:48:16:974][000:001] (model.cpp:313): Empty desc, model name: elevation, output branch:0, output name:output_block1quanticonvolution0_conv_output
[16:48:16:975][INFO] [1653986897.395739633] [dnn]: The model input 0 width is 960 and height is 512
[16:48:16:975][INFO] [1653986897.396065545] [dnn]: The model input 1 width is 960 and height is 512
[16:48:16:975][INFO] [1653986897.396335707] [dnn]: Task init.
[16:48:16:976][INFO] [1653986897.399969317] [dnn]: Set task_num [2]
[16:48:16:976][INFO] [1653986897.400288978] [elevation_dection]: The model input width is 960 and height is 512
[16:48:17:083][WARN] [1653986897.537355789] [elevation_dection]: Create subscription with topic_name: /image_raw
[16:48:17:233][INFO] [1653986897.686988444] [msg pub]: msg_pub_topic_name: elevation_net
[16:48:17:248][WARN] [1653986897.687478270] [elevation_dection]: start success!!!
[16:48:17:296][INFO] [1653986897.744999535] [elevation_net_node]: Recved img encoding: nv12, h: 540, w: 960, step: 960, frame_id: default_cam, stamp: 1653986897.547066678, data size: 777600
[16:48:18:183][INFO] [1653986898.636813142] [elevation_net_node]: Dnn node begin to predict
[16:48:18:315]fx_inv_: 0.000604548
[16:48:18:315]fy_inv_: 0.000604445
[16:48:18:315]cx_inv_: -0.604389
[16:48:18:315]cy_inv_: -0.318132
[16:48:18:315]nx: 0
[16:48:18:315]ny: 0
[16:48:18:315]nz: 1
[16:48:18:315]camera_height: 1
[16:48:18:333]model out width:480, height:256
[16:48:18:421]src_w_stride:8
[16:48:18:421]depth: 998
[16:48:18:421]height: -42.6999
[16:48:18:421]depth: 998
[16:48:18:421]height: -25.3397
[16:48:18:421]depth: 998
[16:48:18:421]height: -22.1114
[16:48:18:421]depth: 998
[16:48:18:421]height: -25.3397
[16:48:18:421]depth: 998
[16:48:18:421]height: -21.9895
[16:48:18:421]depth: 998
[16:48:18:421]height: -48.3039
[16:48:18:421]depth: 998
[16:48:18:421]height: -32.5275
[16:48:18:421]depth: 998
[16:48:18:421]height: -32.7102
[16:48:18:421]depth: 998
[16:48:18:421]height: -33.0148
[16:48:18:421]depth: 998
[16:48:18:422]height: -35.4513
[16:48:18:422]depth: 998
[16:48:18:422]height: -38.1924
[16:48:18:422]depth: 998
[16:48:18:422]height: -34.233
[16:48:18:422]depth: 998
[16:48:18:422]height: -34.233
[16:48:18:422]depth: 998
[16:48:18:422]height: -33.0148
[16:48:18:422]depth: 998
[16:48:18:423]height: -34.3549
[16:48:18:423]depth: 998
[16:48:18:423]height: -35.0249

# 常见问题
