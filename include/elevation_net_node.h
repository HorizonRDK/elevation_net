// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#ifdef CV_BRIDGE_PKG_ENABLED
#include "cv_bridge/cv_bridge.h"
#endif
#include "ai_msgs/msg/capture_targets.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "dnn_node/dnn_node.h"
#include "include/elevation_net_output_parser.h"
#include "include/image_utils.h"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#ifdef SHARED_MEM_ENABLED
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#endif

#ifndef INCLUDE_ELEVATIONNET_NODE_H_
#define INCLUDE_ELEVATIONNET_NODE_H_

using rclcpp::NodeOptions;

using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::DnnNodePara;
using hobot::dnn_node::ModelTaskType;
using hobot::dnn_node::TaskId;

using hobot::dnn_node::DNNInput;
using hobot::dnn_node::DNNResult;
using hobot::dnn_node::NV12PyramidInput;

struct ElevationNetOutput : public DnnNodeOutput {
  std::shared_ptr<std_msgs::msg::Header> image_msg_header = nullptr;
  uint32_t src_img_width;
  uint32_t src_img_height;
  std::string image_name;
};

class ElevationNetNode : public DnnNode {
 public:
  ElevationNetNode(const std::string &node_name,
                   const NodeOptions &options = NodeOptions());
  ~ElevationNetNode() override;

 protected:
  int SetNodePara() override;
  int SetOutputParser() override;
  int PostProcess(const std::shared_ptr<DnnNodeOutput> &outputs) override;

 private:
  int Start();
  int Predict(std::vector<std::shared_ptr<DNNInput>> &inputs,
              const std::shared_ptr<std::vector<hbDNNRoi>> rois,
              std::shared_ptr<DnnNodeOutput> dnn_output);
  int PredictByImage(const std::string image);
  void RosImgProcess(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
#ifdef SHARED_MEM_ENABLED
  void SharedMemImgProcess(
      const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
#endif

 private:
  // 输入参数
  std::string config_file_path_ = "./config";
  int is_sync_mode_ = 1;
  int shared_mem_ = 0;

  std::string model_file_name_ = "config/elevation.hbm";
  std::string model_name_ = "elevation";
  ModelTaskType model_task_type_ = ModelTaskType::ModelInferType;
  int model_input_width_ = 960;
  int model_input_height_ = 512;
  int model_out_width_ = 480;
  int model_out_height_ = 256;
  const int32_t model_output_count_ = 1;
  const int32_t elevation_output_index_ = 0;

  float score_threshold_ = 0.3;
  bool read_detlist_success_ = false;
  std::map<std::string, int> dettype_list_;

  std::chrono::high_resolution_clock::time_point output_tp_;
  int output_frameCount_ = 0;
  std::mutex frame_stat_mtx_;
  std::string msg_pub_topic_name_ = "elevation_net";
  std::shared_ptr<ImageUtils> image_utils_;

#ifdef SHARED_MEM_ENABLED
  rclcpp::SubscriptionHbmem<hbm_img_msgs::msg::HbmMsg1080P>::ConstSharedPtr
      sharedmem_img_subscription_ = nullptr;
  std::string sharedmem_img_topic_name_ = "/hbmem_img";
#endif
  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr
      ros_img_subscription_ = nullptr;
  // 目前只支持订阅原图，可以使用压缩图"/image_raw/compressed" topic
  // 和sensor_msgs::msg::CompressedImage格式扩展订阅压缩图
  std::string ros_img_topic_name_ = "/image_raw";

  std::string pointcloud_pub_topic_name_ = "/elevation_net/points";
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_ =
      nullptr;
  int PubPointcCloud(ElevationNetResult *det_result, uint32_t src_img_width,
                     uint32_t src_img_height);
  int image_shift_ = 24;
  std::vector<std::vector<float>> kCalibMatrix_{
      {746.2463540682126, 0.0, 971.6589299894808, 0.0},
      {0.0, 750.2202098997767, 514.5994408429885, 0.0},
      {0.0, 0.0, 1.0, 0.0}};
};

#endif  // INCLUDE_ELEVATIONNET_NODE_H_
