#pragma once
#include <opencv2/opencv.hpp>

namespace DepthModel {

struct Box {
  cv::Vec3f position;
  cv::Vec3f size;

  Box(const cv::Vec3f& pos, const cv::Vec3f& sz);
};

} // DepthModel
