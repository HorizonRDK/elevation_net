// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "include/elevation_net_node.h"

#include <unistd.h>

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "dnn_node/dnn_node.h"
#include "include/image_utils.h"
#include "rclcpp/rclcpp.hpp"
#ifdef CV_BRIDGE_PKG_ENABLED
#include <cv_bridge/cv_bridge.h>
#endif
#include "include/elevation_net_output_parser.h"

using hobot::easy_dnn::OutputDescription;
using hobot::easy_dnn::OutputParser;

ElevationNetNode::ElevationNetNode(const std::string &node_name,
                                   const NodeOptions &options)
    : DnnNode(node_name, options) {
  this->declare_parameter<int>("is_sync_mode", is_sync_mode_);
  this->declare_parameter<std::string>("config_file_path", config_file_path_);
  this->declare_parameter<int>("shared_men", shared_mem_);

  this->get_parameter<int>("is_sync_mode", is_sync_mode_);
  this->get_parameter<std::string>("config_file_path", config_file_path_);
  this->get_parameter<int>("shared_mem", shared_mem_);
  model_file_name_ = config_file_path_ + "/elevation.hbm";

  std::stringstream ss;
  ss << "Parameter:"
     << "\nconfig_file_path_:" << config_file_path_
     << "\nshared_men:" << shared_mem_ << "\n is_sync_mode_: " << is_sync_mode_
     << "\n model_file_name_: " << model_file_name_;
  RCLCPP_WARN(rclcpp::get_logger("elevation_dection"), "%s", ss.str().c_str());
  if (Start() == 0) {
    RCLCPP_WARN(rclcpp::get_logger("elevation_dection"), "start success!!!");
  } else {
    RCLCPP_WARN(rclcpp::get_logger("elevation_dection"), "start fail!!!");
  }
}

ElevationNetNode::~ElevationNetNode() {}

int ElevationNetNode::Start() {
  int ret = Init();
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("elevation_dection"), "Init failed!");
    return ret;
  }

  ret = GetModelInputSize(0, model_input_width_, model_input_height_);
  if (ret < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("elevation_dection"),
                 "Get model input size fail!");
    return ret;
  }
  RCLCPP_INFO(rclcpp::get_logger("elevation_dection"),
              "The model input width is %d and height is %d",
              model_input_width_, model_input_height_);

  image_utils_ = std::make_shared<ImageUtils>();
  image_utils_->Init();
  if (shared_mem_) {
#ifdef SHARED_MEM_ENABLED
    RCLCPP_WARN(rclcpp::get_logger("elevation_dection"),
                "Create hbmem_subscription with topic_name: %s",
                sharedmem_img_topic_name_.c_str());
    sharedmem_img_subscription_ =
        this->create_subscription_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
            sharedmem_img_topic_name_, 10,
            std::bind(&ElevationNetNode::SharedMemImgProcess, this,
                      std::placeholders::_1));
#else
    RCLCPP_ERROR(rclcpp::get_logger("elevation_dection"),
                 "Unsupport shared mem");
#endif
  } else {
    RCLCPP_WARN(rclcpp::get_logger("elevation_dection"),
                "Create subscription with topic_name: %s",
                ros_img_topic_name_.c_str());
    ros_img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        ros_img_topic_name_, 10,
        std::bind(&ElevationNetNode::RosImgProcess, this,
                  std::placeholders::_1));
  }
  RCLCPP_INFO(rclcpp::get_logger("msg pub"), "msg_pub_topic_name: %s",
              msg_pub_topic_name_.data());
  return 0;
}

int ElevationNetNode::SetNodePara() {
  RCLCPP_INFO(rclcpp::get_logger("elevation_dection"), "Set node para.");
  if (!dnn_node_para_ptr_) {
    return -1;
  }
  dnn_node_para_ptr_->model_file = model_file_name_;
  dnn_node_para_ptr_->model_name = model_name_;
  dnn_node_para_ptr_->model_task_type = model_task_type_;
  dnn_node_para_ptr_->task_num = 2;
  return 0;
}

int ElevationNetNode::SetOutputParser() {
  // set output parser
  auto model_manage = GetModel();
  if (!model_manage || !dnn_node_para_ptr_) {
    RCLCPP_ERROR(rclcpp::get_logger("elevation_net_node"), "Invalid model");
    return -1;
  }

  if (model_manage->GetOutputCount() < model_output_count_) {
    RCLCPP_ERROR(rclcpp::get_logger("elevation_net_node"),
                 "Error! Model %s output count is %d, unmatch with count %d",
                 dnn_node_para_ptr_->model_name.c_str(),
                 model_manage->GetOutputCount(), model_output_count_);
    return -1;
  }

  std::shared_ptr<OutputParser> elevation_out_parser =
      std::make_shared<ElevationNetOutputParser>();
  model_manage->SetOutputParser(elevation_output_index_, elevation_out_parser);
  return 0;
}

int ElevationNetNode::PostProcess(
    const std::shared_ptr<DnnNodeOutput> &node_output) {
  struct timespec time_start = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_start);
  ai_msgs::msg::Perf perf;
  perf.set__type("PostProcess");
  perf.stamp_start.sec = time_start.tv_sec;
  perf.stamp_start.nanosec = time_start.tv_nsec;

  auto elevation_output =
      std::dynamic_pointer_cast<ElevationNetOutput>(node_output);
  if (elevation_output) {
    std::stringstream ss;
    ss << "elevation output dection info";
    if (elevation_output->image_msg_header) {
      ss << ", frame_id: " << elevation_output->image_msg_header->frame_id
         << ", stamp: " << elevation_output->image_msg_header->stamp.sec << "."
         << elevation_output->image_msg_header->stamp.nanosec;
    }
    RCLCPP_INFO(rclcpp::get_logger("elevation_net_node"), "%s",
                ss.str().c_str());
  }

  const auto &outputs = node_output->outputs;
  RCLCPP_INFO(rclcpp::get_logger("elevation_net_node"), "outputs size: %d",
              outputs.size());
  if (outputs.empty() ||
      static_cast<int32_t>(outputs.size()) < model_output_count_) {
    RCLCPP_ERROR(rclcpp::get_logger("elevation_net_node"), "Invalid outputs");
    return -1;
  }
  int smart_fps = 0;
  {
    auto tp_now = std::chrono::system_clock::now();
    std::unique_lock<std::mutex> lk(frame_stat_mtx_);
    output_frameCount_++;
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_now - output_tp_)
                        .count();
    if (interval >= 1000) {
      RCLCPP_WARN(rclcpp::get_logger("elevation_net_node"), "Smart fps = %d",
                  output_frameCount_);
      smart_fps = output_frameCount_;
      output_frameCount_ = 0;
      output_tp_ = std::chrono::system_clock::now();
    }
  }

  auto *det_result = dynamic_cast<ElevationNetResult *>(
      outputs[elevation_output_index_].get());
  if (!det_result) {
    RCLCPP_INFO(rclcpp::get_logger("elevation_net_node"), "invalid cast");
  } else {
    PubPointcCloud(det_result, elevation_output->src_img_width,
                   elevation_output->src_img_height);
  }
  return 0;
}

int ElevationNetNode::Predict(std::vector<std::shared_ptr<DNNInput>> &inputs,
                              const std::shared_ptr<std::vector<hbDNNRoi>> rois,
                              std::shared_ptr<DnnNodeOutput> dnn_output) {
  return Run(inputs, dnn_output, rois, is_sync_mode_ == 1 ? true : false);
}

int ElevationNetNode::PredictByImage(const std::string image) {
  // std::string image = "./config/images/charging_base.png";
  if (access(image.c_str(), R_OK) == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("elevation_net_node"),
                 "Image: %s not exist!", image.c_str());
    return -1;
  }

  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::vector<std::shared_ptr<hobot::easy_dnn::NV12PyramidInput>> pyramid_list;
  int ret = 0;
  // bgr img，支持将图片resize到模型输入size
  ret = image_utils_->GetNV12Pyramid(image, ImageType::BGR, model_input_height_,
                                     model_input_width_, pyramid_list);
  if (ret != 0 || pyramid_list.size() != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("elevation_net_node"), "Get Nv12 pym fail");
    return -1;
  }

  // 2. 使用pyramid创建DNNInput对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入
  auto inputs =
      std::vector<std::shared_ptr<DNNInput>>{pyramid_list[0], pyramid_list[1]};
  auto dnn_output = std::make_shared<ElevationNetOutput>();
  dnn_output->src_img_width = 1920;
  dnn_output->src_img_height = 1080;
  dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header->set__frame_id("test_frame");
  dnn_output->image_msg_header->set__stamp(rclcpp::Time());
  dnn_output->image_name = image;

  // 3. 开始预测
  ret = Predict(inputs, nullptr, dnn_output);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono3d_indoor_detection"),
                 "predict img failed!");
  }
  return ret;
}

void ElevationNetNode::RosImgProcess(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
  if (!img_msg || !rclcpp::ok()) {
    return;
  }

  std::stringstream ss;
  ss << "Recved img encoding: " << img_msg->encoding
     << ", h: " << img_msg->height << ", w: " << img_msg->width
     << ", step: " << img_msg->step
     << ", frame_id: " << img_msg->header.frame_id
     << ", stamp: " << img_msg->header.stamp.sec << "."
     << img_msg->header.stamp.nanosec
     << ", data size: " << img_msg->data.size();
  RCLCPP_INFO(rclcpp::get_logger("elevation_net_node"), "%s", ss.str().c_str());

  auto tp_start = std::chrono::system_clock::now();

  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::vector<std::shared_ptr<hobot::easy_dnn::NV12PyramidInput>> pyramid_list;
  int ret = 0;
  if ("rgb8" == img_msg->encoding) {
#ifdef CV_BRIDGE_PKG_ENABLED
    auto cv_img =
        cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(img_msg), "bgr8");

    {
      auto tp_now = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                          tp_now - tp_start)
                          .count();
      RCLCPP_DEBUG(rclcpp::get_logger("elevation_net_node"),
                   "after cvtColorForDisplay cost ms: %d", interval);
    }

    ret = image_utils_->GetNV12Pyramid(cv_img->image, model_input_height_,
                                       model_input_width_, pyramid_list);
#else
    RCLCPP_ERROR(rclcpp::get_logger("elevation_net_node"),
                 "Unsupport cv bridge");
#endif
  } else if ("nv12" == img_msg->encoding) {
    ret = image_utils_->GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char *>(img_msg->data.data()), img_msg->height,
        img_msg->width, model_input_height_, model_input_width_, pyramid_list);
  }

  if (ret != 0 || pyramid_list.size() != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("elevation_net_node"), "Get Nv12 pym fail");
    return;
  }

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start)
            .count();
    RCLCPP_DEBUG(rclcpp::get_logger("elevation_net_node"),
                 "after GetNV12Pyramid cost ms: %d", interval);
  }

  RCLCPP_INFO(rclcpp::get_logger("elevation_net_node"),
              "Dnn node begin to predict");
  // 2. 使用pyramid创建DNNInput对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入
  auto inputs =
      std::vector<std::shared_ptr<DNNInput>>{pyramid_list[0], pyramid_list[1]};
  auto dnn_output = std::make_shared<ElevationNetOutput>();
  dnn_output->src_img_width = img_msg->width;
  dnn_output->src_img_height = img_msg->height;
  dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header->set__frame_id(img_msg->header.frame_id);
  dnn_output->image_msg_header->set__stamp(img_msg->header.stamp);
  // dnn_output->image_msg_header->set__frame_id(std::to_string(img_msg->index));
  // dnn_output->image_msg_header->set__stamp(img_msg->time_stamp);

  // 3. 开始预测
  ret = Predict(inputs, nullptr, dnn_output);
  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start)
            .count();
    RCLCPP_DEBUG(rclcpp::get_logger("elevation_net_node"),
                 "after Predict cost ms: %d", interval);
  }
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("elevation_net_node"),
                 "predict img failed!");
  }
  return;
}

#ifdef SHARED_MEM_ENABLED
void ElevationNetNode::SharedMemImgProcess(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr img_msg) {
  if (!img_msg || !rclcpp::ok()) {
    return;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("elevation_net_node"), "go into shared mem");

  // dump recved img msg
  // std::ofstream ofs("img_" + std::to_string(img_msg->index) + "." +
  // std::string(reinterpret_cast<const char*>(img_msg->encoding.data())));
  // ofs.write(reinterpret_cast<const char*>(img_msg->data.data()),
  //   img_msg->data_size);

  auto tp_start = std::chrono::system_clock::now();

  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::vector<std::shared_ptr<hobot::easy_dnn::NV12PyramidInput>> pyramid_list;
  int ret = 0;
  if ("nv12" ==
      std::string(reinterpret_cast<const char *>(img_msg->encoding.data()))) {
    ret = image_utils_->GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char *>(img_msg->data.data()), img_msg->height,
        img_msg->width, model_input_height_, model_input_width_, pyramid_list);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("elevation_net_node"),
                "share mem unsupported img encoding: %s", img_msg->encoding);
  }

  if (ret != 0 || pyramid_list.size() != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("elevation_net_node"), "Get Nv12 pym fail");
    return;
  }

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start)
            .count();
    RCLCPP_DEBUG(rclcpp::get_logger("elevation_net_node"),
                 "after GetNV12Pyramid cost ms: %d", interval);
  }

  // 2. 使用pyramid创建DNNInput对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入
  auto inputs =
      std::vector<std::shared_ptr<DNNInput>>{pyramid_list[0], pyramid_list[1]};
  auto dnn_output = std::make_shared<ElevationNetOutput>();
  dnn_output->src_img_width = img_msg->width;
  dnn_output->src_img_height = img_msg->height;
  dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header->set__frame_id(std::to_string(img_msg->index));
  dnn_output->image_msg_header->set__stamp(img_msg->time_stamp);

  // 3. 开始预测
  ret = Predict(inputs, nullptr, dnn_output);
  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start)
            .count();
    RCLCPP_DEBUG(rclcpp::get_logger("elevation_net_node"),
                 "after Predict cost ms: %d", interval);
  }

  // 4. 处理预测结果，如渲染到图片或者发布预测结果
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("elevation_net_node"),
                 "Run predict failed!");
  }
  return;
}
#endif

int ElevationNetNode::PubPointcCloud(ElevationNetResult *det_result,
                                     uint32_t src_img_width,
                                     uint32_t src_img_height) {
  if (cloud_pub_ == nullptr) {
    this->get_parameter_or("pointcloud_pub_topic_name",
                           pointcloud_pub_topic_name_,
                           pointcloud_pub_topic_name_);
    cloud_pub_ = this->create_publisher<
            sensor_msgs::msg::PointCloud2>(pointcloud_pub_topic_name_, 5);
  }
  cv::Mat depth = cv::Mat(model_out_height_, model_out_width_, CV_32FC1,
     det_result->depth_result.values.data());
  cv::Mat height = cv::Mat(model_out_height_, model_out_width_, CV_32FC1,
     det_result->height_result.values.data());

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "camera";
  cloud.header.stamp = rclcpp::Clock().now();
  sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
  cloud_mod.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32, "height", 1,
      sensor_msgs::msg::PointField::FLOAT32);
  cloud_mod.resize(depth.rows * depth.cols);
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  float down_ratio = 1.f * src_img_width / depth.cols;
  for (int j = 0; j < depth.rows; ++j) {
    for (int i = 0; i < depth.cols; ++i) {
      float d, h;
      if (depth.type() == CV_32F) {
        d = depth.at<float>(j, i);
        h = height.at<float>(j, i);
      } else if (depth.type() == CV_64F) {
        d = depth.at<double>(j, i);
        h = height.at<double>(j, i);
      } else {
        RCLCPP_WARN(rclcpp::get_logger("elevation_net"),
                    "unsupport depth type for publishing pointcloud");
        return -1;
      }
      float x, y, z;
      z = d - kCalibMatrix_[2][3];
      x = (i * d * down_ratio - kCalibMatrix_[0][3] - kCalibMatrix_[0][2] * z) /
          kCalibMatrix_[0][0];
      y = ((j + image_shift_ / down_ratio) * d * down_ratio -
           kCalibMatrix_[1][3] - kCalibMatrix_[1][2] * z) /
          kCalibMatrix_[1][1];
      iter_x[0] = x;
      iter_x[1] = y;
      iter_x[2] = z;
      iter_x[3] = h;
      ++iter_x;
    }
  }
  cloud.is_dense = true;
  cloud_pub_->publish(cloud);
  return 0;
}
