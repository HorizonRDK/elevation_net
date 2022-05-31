// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <dirent.h>

#include <opencv2/core.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "include/elevation_net_node.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  RCLCPP_WARN(rclcpp::get_logger("example"), "This is dnn node example!");
  rclcpp::spin(std::make_shared<ElevationNetNode>("elevation_net_node"));
  rclcpp::shutdown();
  return 0;
}
