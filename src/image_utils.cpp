/*
 * @Description: Elevation method predict depth and elevation
 * @Author:  huaiyu.zhang@horizon.ai
 * @Date: 2021-07-01 10:30:32
 * @Author:  huaiyu.zhang@horizon.ai
 * @Date: 2021-07-01 10:30:32
 * @LastEditors  :  huaiyu.zhang@horizon.ai
 * @LastEditTime : 2021-07-01 10:30:32
 * @Copyright 2017~2021 Horizon Robotics, Inc.
 */
#include "include/image_utils.h"

#include <features.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"

int ImageUtils::Init() {
  model_input_width_ = 960;
  model_input_height_ = 512;
  single_mode_ = true;
  Iw_t_1_ =
      cv::Mat::zeros(model_input_height_ * 3 / 2, model_input_width_, CV_8UC1);

  SetIdGrid();

  return 0;
}

void ImageUtils::SetIdGrid() {
  cv::Mat pixel_coords;
  std::vector<int> i_range(model_input_height_), j_range(model_input_width_);

  cv::Mat i_range_mat(1, model_input_height_, CV_32S, i_range.data()),
      j_range_mat(1, model_input_width_, CV_32S, j_range.data());

  std::iota(i_range.begin(), i_range.end(), 0);
  std::iota(j_range.begin(), j_range.end(), 0);

  i_range_mat =
      cv::repeat(i_range_mat.t(), 1, model_input_width_).reshape(0, 1);
  j_range_mat = cv::repeat(j_range_mat, model_input_height_, 1).reshape(0, 1);

  cv::Mat ones =
      cv::Mat::ones(1, model_input_width_ * model_input_height_, CV_32S);

  H_ = cv::Mat(3, 3, CV_32F);
  current_pixel_coords_ =
      cv::Mat(3, model_input_height_ * model_input_width_, CV_32F);

  j_range_mat.copyTo(current_pixel_coords_.row(0));
  i_range_mat.copyTo(current_pixel_coords_.row(1));
  ones.copyTo(current_pixel_coords_.row(2));
}

inline bool between(float value, int lowerBound, int upperBound) {
  return (value >= lowerBound && value <= upperBound);
}

void ImageUtils::Nv12ToRGB(cv::Mat &nv_img, cv::Mat &rgb_img) {
  cv::cvtColor(nv_img, rgb_img, cv::COLOR_YUV2BGR_NV12);
}

void ImageUtils::RGBToNv12(cv::Mat &rgb_img, cv::Mat &nv_img) {
  cv::Mat yuv;
  cv::cvtColor(rgb_img, yuv, cv::COLOR_RGB2YUV_I420);
  cv::cvtColor(yuv, nv_img, cv::COLOR_YUV2RGB_NV12);
}

void ImageUtils::WarpImageOpenCv(cv::Mat &img, cv::Mat &warped_img) {
  *H_.ptr<float>(0, 0) = 1.4138173e+00f;
  *H_.ptr<float>(0, 1) = 7.2942251e-01f;
  *H_.ptr<float>(0, 2) = -4.1327905e+02f;
  *H_.ptr<float>(1, 0) = -2.2388131e-03f;
  *H_.ptr<float>(1, 1) = 1.8361104e+00f;
  *H_.ptr<float>(1, 2) = -2.3476155e+02f;
  *H_.ptr<float>(2, 0) = -4.4721410e-06f;
  *H_.ptr<float>(2, 1) = 7.4460846e-04f;
  *H_.ptr<float>(2, 2) = 1.f;

  //  H_ = Camera_K * (R_ * t_ * N_.t() / cam_.height) * Camera_K_inv_;
  auto start = std::chrono::high_resolution_clock ::now();
  cv::warpPerspective(img, warped_img, H_.inv(), img.size());
  auto warp = std::chrono::high_resolution_clock ::now();
  std::cout << "w: "
            << std::chrono::duration_cast<std::chrono::microseconds>(warp -
                                                                     start)
                   .count()
            << std::endl;
}

void ImageUtils::WarpImage(cv::Mat &img, cv::Mat &warped_img) {
  *H_.ptr<float>(0, 0) = 1.4138173e+00f;
  *H_.ptr<float>(0, 1) = 7.2942251e-01f;
  *H_.ptr<float>(0, 2) = -4.1327905e+02f;
  *H_.ptr<float>(1, 0) = -2.2388131e-03f;
  *H_.ptr<float>(1, 1) = 1.8361104e+00f;
  *H_.ptr<float>(1, 2) = -2.3476155e+02f;
  *H_.ptr<float>(2, 0) = -4.4721410e-06f;
  *H_.ptr<float>(2, 1) = 7.4460846e-04f;
  *H_.ptr<float>(2, 2) = 1.f;

  auto start = std::chrono::high_resolution_clock ::now();
  //  H_ = Camera_K * (R_ * t_ * N_.t() / cam_.height) * Camera_K_inv_;
  auto new_pixel_coords = H_ * current_pixel_coords_;
  auto multi = std::chrono::high_resolution_clock ::now();

  //  auto &&X = new_pixel_coords.row(0);
  //  auto &&Y = new_pixel_coords.row(1);
  //  auto &&Z = new_pixel_coords.row(2);

  //  cv::divide(X, Z, X_norm, 2.f / (model_input_width_ - 1), CV_32F);
  //  cv::divide(Y, Z, Y_norm, 2.f / (model_input_height_ - 1), CV_32F);
  //
  //  X_norm -= 1;
  //  Y_norm -= 1;

  auto norm = std::chrono::high_resolution_clock ::now();
  BilinearSampler(img, new_pixel_coords, warped_img);
  auto sampler = std::chrono::high_resolution_clock ::now();
  std::cout
      << "m, n, s: "
      << std::chrono::duration_cast<std::chrono::microseconds>(multi - start)
             .count()
      << " "
      << std::chrono::duration_cast<std::chrono::microseconds>(norm - multi)
             .count()
      << " "
      << std::chrono::duration_cast<std::chrono::microseconds>(sampler - norm)
             .count()
      << std::endl;
}

void ImageUtils::BilinearSampler(cv::Mat &img, const cv::Mat &grid,
                                 cv::Mat &warped_img) {
  const auto *gridx = grid.ptr<float>(0, 0);
  const auto *gridy = grid.ptr<float>(grid.rows / 3, 0);
  const auto *gridz = grid.ptr<float>(grid.rows * 2 / 3, 0);
  int o_h = warped_img.rows, o_w = warped_img.cols;
  int i_h = img.rows, i_w = img.cols;

  if (img.type() != CV_8UC3) {
    return;
  }

  for (int h = 0; h < o_h; ++h) {
    for (int w = 0; w < o_w; ++w) {
      auto &warp_pixel_color = warped_img.at<cv::Vec3b>(h, w);
      int grid_index = h * o_w + w;

      //      float y_real = (*(gridy + grid_index) + 1) * (i_h - 1) / 2;
      //      float x_real = (*(gridx + grid_index) + 1) * (i_w - 1) / 2;

      float z_real = *(gridz + grid_index);
      float y_real = *(gridy + grid_index) / z_real;
      float x_real = *(gridx + grid_index) / z_real;

      // NOLINTNEXTLINE
      int top_left_y = static_cast<int>(std::floor(y_real));
      // NOLINTNEXTLINE
      int top_left_x = static_cast<int>(std::floor(x_real));
      float top_left_y_w = 1.f - (y_real - top_left_y);
      float top_left_x_w = 1.f - (x_real - top_left_x);

      cv::Vec3b top_left_v = 0;
      cv::Vec3b top_right_v = 0;
      cv::Vec3b bottom_left_v = 0;
      cv::Vec3b bottom_right_v = 0;

      {
        if (between(top_left_x, 0, i_w - 1) && between(top_left_y, 0, i_h - 1))
          top_left_v = img.at<cv::Vec3b>(top_left_y, top_left_x);
        if (between(top_left_x + 1, 0, i_w - 1) &&
            between(top_left_y, 0, i_h - 1))
          top_right_v = img.at<cv::Vec3b>(top_left_y, top_left_x + 1);
        if (between(top_left_x, 0, i_w - 1) &&
            between(top_left_y + 1, 0, i_h - 1))
          bottom_left_v = img.at<cv::Vec3b>(top_left_y + 1, top_left_x);
        if (between(top_left_x + 1, 0, i_w - 1) &&
            between(top_left_y + 1, 0, i_h - 1))
          bottom_right_v = img.at<cv::Vec3b>(top_left_y + 1, top_left_x + 1);
      }

      warp_pixel_color =
          top_left_v * top_left_y_w * top_left_x_w +
          top_right_v * top_left_y_w * (1.f - top_left_x_w) +
          bottom_left_v * (1.f - top_left_y_w) * top_left_x_w +
          bottom_right_v * (1.f - top_left_y_w) * (1.f - top_left_x_w);
    }
  }
}

void SaveAsNv12(uint16_t width, uint16_t height, void *img) {
  std::string resolution = std::to_string(width) + "_" + std::to_string(height);
  std::ofstream nv12(resolution + ".yuv", std::ios::out | std::ios::binary);
  nv12.write(static_cast<const char *>(img), width * height * 3 / 2);
  nv12.close();
}

void bgr_to_nv12(uint8_t *bgr, int height, int width, cv::Mat &nv12_img) {
  cv::Mat bgr_mat(height, width, CV_8UC3, bgr);
  cv::Mat yuv_mat;
  cv::cvtColor(bgr_mat, yuv_mat, cv::COLOR_BGR2YUV_I420);

  uint8_t *yuv = yuv_mat.ptr<uint8_t>();
  nv12_img = cv::Mat(height * 3 / 2, width, CV_8UC1);
  uint8_t *nv12 = nv12_img.ptr<uint8_t>();

  int uv_height = height / 2;
  int uv_width = width / 2;
  // copy y data
  int y_size = uv_height * uv_width * 4;
  memcpy(nv12, yuv, y_size);

  // copy uv data
  int uv_stride = uv_width * uv_height;
  uint8_t *uv_data = nv12 + y_size;
  for (int i = 0; i < uv_stride; ++i) {
    *(uv_data++) = *(yuv + y_size + i);
    *(uv_data++) = *(yuv + y_size + +uv_stride + i);
  }
}

int ImageUtils::FillTensor(std::shared_ptr<NV12PyramidInput> &pym,
                           uint8_t *y_src, uint8_t *uv_src) {
  int y_len = model_input_width_ * model_input_height_;
  int uv_len = y_len >> 1;
  auto *y_dst = reinterpret_cast<uint8_t *>(pym->y_vir_addr);
  auto *uv_dst = reinterpret_cast<uint8_t *>(pym->uv_vir_addr);
  memcpy(y_dst, y_src, y_len);
  memcpy(uv_dst, uv_src, uv_len);
#if 0
  static int file = 1;
  static bool save = 1;
  if (save) {
    char *buf = new char[y_len + uv_len];
    memcpy(buf, y_dst, y_len);
    memcpy(buf + y_len, uv_dst, uv_len);
    cv::Mat tmp(model_input_height_ * 3 / 2, model_input_width_, CV_8UC1, buf);
    cv::Mat bgr_(model_input_height_, model_input_width_, CV_8UC3);
    cv::cvtColor(tmp, bgr_, CV_YUV2BGR_NV12);
    cv::imwrite("img_pym.jpg", bgr_);
    save = 0;
  }
#endif
  return 0;
}

int ImageUtils::CopyImagetoMat(uint8_t *y_src_in, uint8_t *uv_src_in,
                               int32_t image_width, int32_t image_height) {
  return CopyPymImageAlign(
      Iw_t_1_.data, Iw_t_1_.data + model_input_height_ * model_input_width_,
      y_src_in, uv_src_in, image_width, image_height);
}

int ImageUtils::CopyPymImageAlign(uint8_t *y_dst, uint8_t *uv_dst,
                                  uint8_t *y_src_in, uint8_t *uv_src_in,
                                  int32_t image_width, int32_t image_height) {
  //  如果模型的高小于图像的高，那么只复制下半边的图像
  //  如果模型的宽小于图像的宽，那么只复制中间部分的图像
  uint32_t height_hdiff = (image_height - model_input_height_);
  uint32_t width_hdiff = (image_width - model_input_width_) >> 1;

  auto *y_src = reinterpret_cast<uint8_t *>(y_src_in + width_hdiff +
                                            height_hdiff * image_width);
  auto *uv_src = reinterpret_cast<uint8_t *>(
      uv_src_in + (width_hdiff + height_hdiff * image_width) / 2);

  if (height_hdiff == 0 && width_hdiff == 0) {
    memcpy(y_dst, y_src, model_input_width_ * model_input_height_);
    memcpy(uv_dst, uv_src, model_input_width_ * model_input_height_ / 2);
  } else {
    // copy y data
    for (int hh = 0; hh < model_input_height_; hh++) {
      memcpy(y_dst, y_src, model_input_width_);
      y_dst += model_input_width_;
      y_src += image_width;
    }
    // copy uv data
    for (int hh = 0; hh < model_input_height_ / 2; hh++) {
      memcpy(uv_dst, uv_src, model_input_width_);
      uv_dst += model_input_width_;
      uv_src += image_width;
    }
  }

  return 0;
}

void ImageUtils::FillTensorFromPym(std::shared_ptr<NV12PyramidInput> pym,
                                   uint8_t *y_src_in, uint8_t *uv_src_in,
                                   int32_t image_width, int32_t image_height) {
  auto *y_dst = reinterpret_cast<uint8_t *>(pym->y_vir_addr);
  auto *uv_dst = reinterpret_cast<uint8_t *>(pym->uv_vir_addr);
  CopyPymImageAlign(y_dst, uv_dst, y_src_in, uv_src_in, image_width,
                    image_height);
}

int ImageUtils::GetNV12Pyramid(
    const std::string &image_file, ImageType image_type, int scaled_img_height,
    int scaled_img_width,
    std::vector<std::shared_ptr<NV12PyramidInput>> &pym_list) {
  static bool is_first = true;
  for (int i = 0; i < 2; ++i) {
    pym_list.push_back(NewNV12Pym(model_input_width_, model_input_height_));
  }
  if (ImageType::NV12 == image_type) {
    std::ifstream ifs(image_file, std::ios::in | std::ios::binary);
    if (!ifs) {
      pym_list.clear();
      return -1;
    }
    ifs.seekg(0, std::ios::end);
    int len = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    char *data = new char[len];
    ifs.read(data, len);

    int y_img_len = len / 3 * 2;
    int uv_img_len = len / 3;
    if (is_first || single_mode_) {
      FillTensorFromPym(pym_list[0], reinterpret_cast<uint8_t *>(data),
                        reinterpret_cast<uint8_t *>(data + y_img_len),
                        scaled_img_width, scaled_img_height);
      FillTensorFromPym(pym_list[1], reinterpret_cast<uint8_t *>(data),
                        reinterpret_cast<uint8_t *>(data + y_img_len),
                        scaled_img_width, scaled_img_height);
      is_first = false;
    } else {
      cv::Mat tensor_img = Iw_t_1_;
      if (true) {
        cv::Mat rgb_img(model_input_height_, model_input_width_, CV_8UC3);
        cv::Mat warp_img(model_input_height_, model_input_width_, CV_8UC3);
        cv::Mat warp_nv12(model_input_height_ * 3 / 2, model_input_width_,
                          CV_8UC1);
        Nv12ToRGB(Iw_t_1_, rgb_img);
        WarpImageOpenCv(rgb_img, warp_img);
        //  cv::imwrite("./warp.jpg", warp_img);
        RGBToNv12(rgb_img, warp_nv12);
        tensor_img = warp_nv12;
      }
      FillTensor(pym_list[0], tensor_img.data,
                 tensor_img.data + model_input_height_ * model_input_width_);
      FillTensorFromPym(pym_list[1], reinterpret_cast<uint8_t *>(data),
                        reinterpret_cast<uint8_t *>(data + y_img_len),
                        scaled_img_width, scaled_img_height);
    }
    if (!single_mode_) {
      CopyImagetoMat(reinterpret_cast<uint8_t *>(data),
                     reinterpret_cast<uint8_t *>(data + y_img_len),
                     scaled_img_width, scaled_img_height);
    }
  } else if (ImageType::BGR == image_type) {
    cv::Mat bgr_mat = cv::imread(image_file, cv::IMREAD_COLOR);
    int original_img_height = bgr_mat.rows;
    int original_img_width = bgr_mat.cols;
    cv::Mat resized_mat, nv12_mat;
    if (original_img_height != (uint32_t)model_input_height_ ||
        original_img_width != (uint32_t)model_input_width_) {
      cv::resize(bgr_mat, resized_mat,
                 cv::Size(model_input_width_, model_input_height_));
    } else {
      resized_mat = bgr_mat;
    }

    bgr_to_nv12(resized_mat.data, model_input_height_, model_input_width_,
                nv12_mat);
    // SaveAsNv12(model_input_width_, model_input_height_, nv12_mat.data);
    if (is_first) {
      Iw_t_1_ = nv12_mat;
      is_first = false;
    }
    FillTensor(pym_list[0], Iw_t_1_.data,
               Iw_t_1_.data + model_input_height_ * model_input_width_);
    FillTensor(pym_list[1], nv12_mat.data,
               nv12_mat.data + model_input_height_ * model_input_width_);
    Iw_t_1_ = nv12_mat;
  }
  return 0;
}

int ImageUtils::GetNV12Pyramid(
    const std::string &image_file, int scaled_img_height, int scaled_img_width,
    int &original_img_height, int &original_img_width,
    std::vector<std::shared_ptr<NV12PyramidInput>> &pym_list) {
  static bool is_first = true;
  for (int i = 0; i < 2; ++i) {
    pym_list.push_back(NewNV12Pym(model_input_width_, model_input_height_));
  }

  cv::Mat bgr_mat = cv::imread(image_file, cv::IMREAD_COLOR);
  original_img_height = bgr_mat.rows;
  original_img_width = bgr_mat.cols;
  cv::Mat resized_mat, nv12_mat;
  if (original_img_height != (uint32_t)model_input_height_ ||
      original_img_width != (uint32_t)model_input_width_) {
    cv::resize(bgr_mat, resized_mat,
               cv::Size(model_input_width_, model_input_height_));
  } else {
    resized_mat = bgr_mat;
  }

  bgr_to_nv12(resized_mat.data, model_input_height_, model_input_width_,
              nv12_mat);
  SaveAsNv12(model_input_width_, model_input_height_, nv12_mat.data);
  if (is_first) {
    Iw_t_1_ = nv12_mat;
    is_first = false;
  }
  FillTensor(pym_list[0], Iw_t_1_.data,
             Iw_t_1_.data + model_input_height_ * model_input_width_);
  FillTensor(pym_list[1], nv12_mat.data,
             nv12_mat.data + model_input_height_ * model_input_width_);
  Iw_t_1_ = nv12_mat;
  return 0;
}

std::shared_ptr<NV12PyramidInput> ImageUtils::NewNV12Pym(int32_t width,
                                                         int32_t height) {
  std::shared_ptr<NV12PyramidInput> pyramid = nullptr;
  auto *y = new hbSysMem;
  auto *uv = new hbSysMem;
  auto w_stride = ALIGN_16(width);
  hbSysAllocCachedMem(y, height * w_stride);
  hbSysAllocCachedMem(uv, height / 2 * w_stride);

  hbSysFlushMem(y, HB_SYS_MEM_CACHE_CLEAN);
  hbSysFlushMem(uv, HB_SYS_MEM_CACHE_CLEAN);
  auto pym_in = new NV12PyramidInput;
  pym_in->width = width;
  pym_in->height = height;
  pym_in->y_vir_addr = y->virAddr;
  pym_in->y_phy_addr = y->phyAddr;
  pym_in->y_stride = w_stride;
  pym_in->uv_vir_addr = uv->virAddr;
  pym_in->uv_phy_addr = uv->phyAddr;
  pym_in->uv_stride = w_stride;
  pyramid = std::shared_ptr<NV12PyramidInput>(
      pym_in, [y, uv](NV12PyramidInput *pym_in) {
        // Release memory after deletion
        hbSysFreeMem(y);
        hbSysFreeMem(uv);
        delete y;
        delete uv;
        delete pym_in;
      });
  return pyramid;
}

int ImageUtils::GetNV12Pyramid(
    const cv::Mat &bgr_mat, int scaled_img_height, int scaled_img_width,
    std::vector<std::shared_ptr<NV12PyramidInput>> &pym_list) {
  static bool is_first = true;
  for (int i = 0; i < 2; ++i) {
    pym_list.push_back(NewNV12Pym(model_input_width_, model_input_height_));
  }
  int original_img_height = bgr_mat.rows;
  int original_img_width = bgr_mat.cols;
  cv::Mat resized_mat, nv12_mat;
  if (original_img_height != (uint32_t)model_input_height_ ||
      original_img_width != (uint32_t)model_input_width_) {
    cv::resize(bgr_mat, resized_mat,
               cv::Size(model_input_width_, model_input_height_));
  } else {
    resized_mat = bgr_mat;
  }

  bgr_to_nv12(resized_mat.data, model_input_height_, model_input_width_,
              nv12_mat);
  SaveAsNv12(model_input_width_, model_input_height_, nv12_mat.data);
  if (is_first) {
    Iw_t_1_ = nv12_mat;
    is_first = false;
  }
  FillTensor(pym_list[0], Iw_t_1_.data,
             Iw_t_1_.data + model_input_height_ * model_input_width_);
  FillTensor(pym_list[1], nv12_mat.data,
             nv12_mat.data + model_input_height_ * model_input_width_);
  Iw_t_1_ = nv12_mat;
  return 0;
}

int ImageUtils::GetNV12PyramidFromNV12Img(
    const char *in_img_data, int in_img_height, int in_img_width,
    int scaled_img_height, int scaled_img_width,
    std::vector<std::shared_ptr<NV12PyramidInput>> &pym_list) {
  static bool is_first = true;
  for (int i = 0; i < 2; ++i) {
    pym_list.push_back(NewNV12Pym(model_input_width_, model_input_height_));
  }

  char *data = nullptr;
  cv::Mat img_nv12;
  if (in_img_width != scaled_img_width || in_img_height != scaled_img_height) {
    cv::Mat nv12_tmp(in_img_height * 3 / 2, in_img_width, CV_8UC1,
                     const_cast<char *>(in_img_data));
    cv::Mat bgr_mat(in_img_height, in_img_width, CV_8UC3);
    cv::cvtColor(nv12_tmp, bgr_mat, CV_YUV2BGR_NV12);
    // cv::imwrite("img1.jpg", bgr_mat);
    cv::Mat resized_mat;
    cv::resize(bgr_mat, resized_mat,
               cv::Size(model_input_width_, model_input_height_));
    ImageUtils::BGRToNv12(resized_mat, img_nv12);
    data = reinterpret_cast<char *>(img_nv12.data);
  } else {
    data = const_cast<char *>(in_img_data);
  }

  int y_img_len = model_input_width_ * model_input_height_;
  int uv_img_len = y_img_len / 2;
  if (is_first || single_mode_) {
    FillTensorFromPym(pym_list[0], reinterpret_cast<uint8_t *>(data),
                      reinterpret_cast<uint8_t *>(data + y_img_len),
                      scaled_img_width, scaled_img_height);
    FillTensorFromPym(pym_list[1], reinterpret_cast<uint8_t *>(data),
                      reinterpret_cast<uint8_t *>(data + y_img_len),
                      scaled_img_width, scaled_img_height);
    is_first = false;
  } else {
    cv::Mat tensor_img = Iw_t_1_;
    if (true) {
      cv::Mat rgb_img(model_input_height_, model_input_width_, CV_8UC3);
      cv::Mat warp_img(model_input_height_, model_input_width_, CV_8UC3);
      cv::Mat warp_nv12(model_input_height_ * 3 / 2, model_input_width_,
                        CV_8UC1);
      Nv12ToRGB(Iw_t_1_, rgb_img);
      WarpImageOpenCv(rgb_img, warp_img);
      //  cv::imwrite("./warp.jpg", warp_img);
      RGBToNv12(rgb_img, warp_nv12);
      tensor_img = warp_nv12;
    }
    FillTensor(pym_list[0], tensor_img.data,
               tensor_img.data + model_input_height_ * model_input_width_);
    FillTensorFromPym(pym_list[1], reinterpret_cast<uint8_t *>(data),
                      reinterpret_cast<uint8_t *>(data + y_img_len),
                      scaled_img_width, scaled_img_height);
  }
  if (!single_mode_) {
    CopyImagetoMat(reinterpret_cast<uint8_t *>(data),
                   reinterpret_cast<uint8_t *>(data + y_img_len),
                   scaled_img_width, scaled_img_height);
  }
  return 0;
}

int32_t ImageUtils::BGRToNv12(cv::Mat &bgr_mat, cv::Mat &img_nv12) {
  auto height = bgr_mat.rows;
  auto width = bgr_mat.cols;

  if (height % 2 || width % 2) {
    std::cerr << "input img height and width must aligned by 2!";
    return -1;
  }
  cv::Mat yuv_mat;
  cv::cvtColor(bgr_mat, yuv_mat, cv::COLOR_BGR2YUV_I420);
  if (yuv_mat.data == nullptr) {
    std::cerr << "yuv_mat.data is null pointer" << std::endl;
    return -1;
  }

  auto *yuv = yuv_mat.ptr<uint8_t>();
  if (yuv == nullptr) {
    std::cerr << "yuv is null pointer" << std::endl;
    return -1;
  }
  img_nv12 = cv::Mat(height * 3 / 2, width, CV_8UC1);
  auto *ynv12 = img_nv12.ptr<uint8_t>();

  int32_t uv_height = height / 2;
  int32_t uv_width = width / 2;

  // copy y data
  int32_t y_size = height * width;
  memcpy(ynv12, yuv, y_size);

  // copy uv data
  uint8_t *nv12 = ynv12 + y_size;
  uint8_t *u_data = yuv + y_size;
  uint8_t *v_data = u_data + uv_height * uv_width;

  for (int32_t i = 0; i < uv_width * uv_height; i++) {
    *nv12++ = *u_data++;
    *nv12++ = *v_data++;
  }
  return 0;
}
