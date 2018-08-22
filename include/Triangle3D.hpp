#pragma once
#include <opencv2/opencv.hpp>

namespace DepthModel {

class Triangle3D {
  public:
    cv::Vec3f verties[3];
    Triangle3D();
    Triangle3D(const cv::Vec3f& a, const cv::Vec3f& b, const cv::Vec3f& c);
    Triangle3D(const Triangle3D& other);
    Triangle3D& operator= (const Triangle3D& other);
};

} // DepthModel
