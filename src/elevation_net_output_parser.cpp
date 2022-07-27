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

#include "include/elevation_net_output_parser.h"

#include "dnn/hb_dnn_ext.h"
#include "include/image_utils.h"
#include "rclcpp/rclcpp.hpp"

#define ENABLE_NEON 1
#if defined(__ARM_NEON__) || (defined(__ARM_NEON) && defined(__aarch64__))
#define ARM_NEON_BACK_UP 1
#else
#define ARM_NEON_BACK_UP 0
#endif
#if ARM_NEON_BACK_UP && ENABLE_NEON
#include <arm_neon.h>
#define ELEVATION_NEON 1
#else
#define ELEVATION_NEON 0
#endif

#include <algorithm>
#include <cassert>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "opencv2/opencv.hpp"

int get_tensor_hwc_index(std::shared_ptr<DNNTensor> tensor, int *h_index,
                         int *w_index, int *c_index) {
  if (tensor->properties.tensorLayout == HB_DNN_LAYOUT_NHWC) {
    *h_index = 1;
    *w_index = 2;
    *c_index = 3;
  } else if (tensor->properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
    *c_index = 1;
    *h_index = 2;
    *w_index = 3;
  } else {
    return -1;
  }
  return 0;
}

int32_t ElevationNetOutputParser::Parse(
    std::shared_ptr<ElevationNetResult> &output,
    std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
    std::shared_ptr<OutputDescription> &output_description,
    std::shared_ptr<DNNTensor> &output_tensor) {
  static int print = 1;
  if (print) {
    //  f37_1080p
    //  cameraMatrix:
    //  [1654.12761216529, 0, 999.737219159173;
    //  0, 1654.410891702582, 526.3207674289991;
    //  0, 0, 1]
    //  cameraMatrix_inv:
    //  [0.0006045482782860858, 0, -0.6043894145811973;
    //  0, 0.0006044447633990631, -0.3181318317406348;
    //  0, 0, 1]
    //  distCoeffs:
    //  [0.1194292740855249, -0.7251445245675587, 0.003460052616720042,
    //  -0.0008500186617909979, 0.7460054560423278]
    RCLCPP_INFO(rclcpp::get_logger("elevation_net_parser"), "fx_inv_: %f",
                desc_.fx_inv_);
    RCLCPP_INFO(rclcpp::get_logger("elevation_net_parser"), "fy_inv_: %f",
                desc_.fy_inv_);
    RCLCPP_INFO(rclcpp::get_logger("elevation_net_parser"), "cx_inv_: %f",
                desc_.cx_inv_);
    RCLCPP_INFO(rclcpp::get_logger("elevation_net_parser"), "cy_inv_: %f",
                desc_.cy_inv_);
    RCLCPP_INFO(rclcpp::get_logger("elevation_net_parser"), "nx_: %f",
                desc_.nx_);
    RCLCPP_INFO(rclcpp::get_logger("elevation_net_parser"), "ny_: %f",
                desc_.ny_);
    RCLCPP_INFO(rclcpp::get_logger("elevation_net_parser"), "nz_: %f",
                desc_.nz_);
    RCLCPP_INFO(rclcpp::get_logger("elevation_net_parser"), "camera_height: %f",
                desc_.cam_H_);
    print = 0;
  }

  std::shared_ptr<ElevationNetResult> result;
  if (!output) {
    result = std::make_shared<ElevationNetResult>();
    result->Reset();
    output = result;
  } else {
    result = std::dynamic_pointer_cast<ElevationNetResult>(output);
    result->Reset();
  }

  if (!output_tensor) {
    RCLCPP_INFO(rclcpp::get_logger("elevation_net_parser"),
                "tensor layout error.");
    return -1;
  }

  hbSysFlushMem(&(output_tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  int ret = PostProcess(output_tensor, result);
  if (ret != 0) {
    RCLCPP_INFO(rclcpp::get_logger("elevation_net_parser"),
                "postprocess return error, code = %d", ret);
  }
  return ret;
}

inline float32x4_t vshlq_n_f32_u32(uint32_t shift) {
  return vcvtq_f32_u32(vshlq_n_u32(vdupq_n_u32(1), shift));
}

// one million accuracy
inline float32x4_t vrecpeq_f32_high(float32x4_t value) {
  float32x4_t reciprocal = vrecpeq_f32(value);
  return vmulq_f32(vrecpsq_f32(value, reciprocal), reciprocal);
}

inline float32x4_t vrsqrteq_f32_high(float32x4_t val) {
  float32x4_t e = vrsqrteq_f32(val);
  return vrecpeq_f32_high(vmulq_f32(vrsqrtsq_f32(vmulq_f32(e, e), val), e));
}

inline float32x4_t vrsqrteq_f32_high_inv(float32x4_t val) {
  float32x4_t e = vrsqrteq_f32(val);
  return vmulq_f32(vrsqrtsq_f32(vmulq_f32(e, e), val), e);
}

inline float32x4_t vcvtq_fixed_f32_s32(const int32_t *value,
                                       float32x4_t shift_f) {
  float32x4_t reciprocal = vrecpeq_f32_high(shift_f);
  return vmulq_f32(vcvtq_f32_s32(vld1q_s32(value)), reciprocal);
}

inline float GetFloatByInt(int32_t value, uint32_t shift) {
  float ret_x = value;
  if (value != 0) {
    int *ix = reinterpret_cast<int *>(&ret_x);
    (*ix) -= shift * 0x00800000;
  }
  return ret_x;
}

void DumpModelPrediction(std::shared_ptr<DNNTensor> &output_tensor) {
  std::ofstream elevation_result("elevation_result2.txt", std::ios::out);
  int output_width, output_height;
  int h_idx, w_idx, c_idx;
  get_tensor_hwc_index(output_tensor, &h_idx, &w_idx, &c_idx);
  output_width = output_tensor->properties.validShape.dimensionSize[w_idx];
  output_height = output_tensor->properties.validShape.dimensionSize[h_idx];
  char *data = reinterpret_cast<char *>(output_tensor->sysMem[0].virAddr);
  for (int h = 0; h < output_height; ++h) {
    for (int w = 0; w < output_width; ++w) {
      int *value = reinterpret_cast<int *>(data);
      GetFloatByInt(*value, 14);
      elevation_result << *value << std::endl;
      switch (output_tensor->properties.tensorType) {
        case HB_DNN_TENSOR_TYPE_S32:
        case HB_DNN_TENSOR_TYPE_U32:
          data += (4 * output_tensor->properties.alignedShape.dimensionSize[3]);
          break;
        default:
          RCLCPP_INFO(rclcpp::get_logger("elevation_net_parser"),
                      "unimplemented data_type: %d",
                      output_tensor->properties.tensorType);
          break;
      }
    }
  }
  elevation_result.close();
}

void ElevationNetOutputParser::GenerateFeaturePoints(int width, int height) {
  for (int h = 0; h < height; ++h) {
    for (int w = 0; w < width; ++w) {
      float x = w * desc_.fx_inv_ + desc_.cx_inv_;
      float y = h * desc_.fy_inv_ + desc_.cy_inv_;
      float bias = desc_.nx_ * x + desc_.ny_ * y + desc_.nz_;
      points_.push_back(bias);
    }
  }
}

int32_t ElevationNetOutputParser::PostProcess(
    std::shared_ptr<DNNTensor> &output_tensor,
    std::shared_ptr<ElevationNetResult> &det_result) {
  int h_idx, w_idx, c_idx;
  get_tensor_hwc_index(output_tensor, &h_idx, &w_idx, &c_idx);
  if (points_.empty()) {
    model_output_width_ =
        output_tensor->properties.validShape.dimensionSize[w_idx];
    model_output_height_ =
        output_tensor->properties.validShape.dimensionSize[h_idx];
    RCLCPP_INFO(rclcpp::get_logger("elevation_net_parser"),
                "model out width: %d, height: %d", model_output_width_,
                model_output_height_);
    GenerateFeaturePoints(model_output_width_, model_output_height_);
  }

  det_result->depth_result.values.resize(model_output_width_ *
                                         model_output_height_);
  det_result->height_result.values.resize(model_output_width_ *
                                          model_output_height_);

  uint8_t shift = output_tensor->properties.shift.shiftData[0];
  auto &aligned_shape = output_tensor->properties.alignedShape;
  uint32_t src_w_stride = aligned_shape.dimensionSize[3];

  int *gamma_fix_ptr = nullptr;
  if (output_tensor->properties.tensorType == HB_DNN_TENSOR_TYPE_S32 ||
      output_tensor->properties.tensorType == HB_DNN_TENSOR_TYPE_U32) {
    gamma_fix_ptr = reinterpret_cast<int *>(output_tensor->sysMem[0].virAddr);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("elevation_net_parser"),
                "unimplemented data_type: %d",
                output_tensor->properties.tensorType);
  }
#if ELEVATION_NEON
  GetFrameOutPut_NEON(shift, src_w_stride,
                      det_result->depth_result.values.data(),
                      det_result->height_result.values.data(), gamma_fix_ptr);
#else
  GetFrameOutPut(shift, src_w_stride, det_result->depth_result.values.data(),
                 det_result->height_result.values.data(), gamma_fix_ptr);
#endif
  return 0;
}

void ElevationNetOutputParser::GetFrameOutPut_NEON(uint32_t shift,
                                                   uint32_t src_w_stride,
                                                   void *depth_ptr,
                                                   void *height_ptr,
                                                   void *gamma_ptr) {
  //  std::ofstream out("elevation_neon.txt", std::ios::out);
  if (model_output_width_ % 4 != 0) {
    return GetFrameOutPut(shift, src_w_stride, depth_ptr, height_ptr,
                          gamma_ptr);
  }
  int *gamma_fix_ptr = static_cast<int *>(gamma_ptr);
  float *points_ptr = points_.data();
  static float32x4_t shift_neon = vshlq_n_f32_u32(shift);
  static float32x4_t min_neon = vdupq_n_f32(1e-3f);
  static float32x4_t max_neon = vdupq_n_f32(15.f);
  int align_width = model_output_width_ / 4;
  int gamma[4];
  for (int h = 0; h < model_output_height_; ++h) {
    for (int w = 0; w < align_width; ++w) {
      gamma[0] = *gamma_fix_ptr;
      gamma_fix_ptr += src_w_stride;
      gamma[1] = *gamma_fix_ptr;
      gamma_fix_ptr += src_w_stride;
      gamma[2] = *gamma_fix_ptr;
      gamma_fix_ptr += src_w_stride;
      gamma[3] = *gamma_fix_ptr;
      gamma_fix_ptr += src_w_stride;
      float32x4_t gamma_neon = vcvtq_fixed_f32_s32(gamma, shift_neon);
      float32x4_t point_neon = vld1q_f32(points_ptr + 4 * w);
      float32x4_t val_neon = vminq_f32(
          vmaxq_f32(vsubq_f32(gamma_neon, point_neon), min_neon), max_neon);
      float32x4_t depth_neon = vmulq_n_f32(vrecpeq_f32(val_neon), desc_.cam_H_);
      float32x4_t height_neon = vmulq_f32(gamma_neon, depth_neon);
      vst1q_f32(static_cast<float *>(depth_ptr) + 4 * w, depth_neon);
      vst1q_f32(static_cast<float *>(height_ptr) + 4 * w, height_neon);
      for (int i = 0; i < 4; i++) {
        RCLCPP_INFO(rclcpp::get_logger("elevation_net_parser"), "depth: %f",
                    *(reinterpret_cast<float *>(depth_ptr) + i + 4 * w));
        RCLCPP_INFO(rclcpp::get_logger("elevation_net_parser"), "height: %f",
                    *(reinterpret_cast<float *>(height_ptr) + i + 4 * w));
      }
    }
  }
}

void ElevationNetOutputParser::GetFrameOutPut(uint32_t shift,
                                              uint32_t src_w_stride,
                                              void *depth_ptr, void *height_ptr,
                                              void *gamma_ptr) {
  // std::ofstream out("elevation_cpu.txt", std::ios::out);
  int *gamma_fix_ptr = static_cast<int *>(gamma_ptr);
  float *points_ptr = points_.data();
  float *dp = static_cast<float *>(depth_ptr);
  float *hp = static_cast<float *>(height_ptr);
  for (int h = 0; h < model_output_height_; ++h) {
    for (int w = 0; w < model_output_width_; ++w) {
      float gamma = GetFloatByInt(*gamma_fix_ptr, shift);
      gamma_fix_ptr += src_w_stride;
      auto &point = *points_ptr++;
      float *depth = dp++;
      float *height = hp++;
      float val = gamma - point;
      if (fabs(val) < 1e-6) {
        val += 1e-3;
      }
      *depth = desc_.cam_H_ / val;
      *height = gamma * *depth;
      if (*depth < 0) *depth = 0;
      if (*height < 0) *height = 0;
      if (*depth > 2) *depth = 2;
      if (*height > 2) *height = 2;
      //        out << "gamma: " << gamma << std::endl;
      //        out << "depth: " << *depth << std::endl;
      //        out << "height: " << *height << std::endl;
    }
  }
}
