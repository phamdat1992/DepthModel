/**
 * @author: quangloc99
 */

#pragma once
#include <opencv2/opencv.hpp>

namespace DepthModel {

class RayLine {
  public:
    // Const được dùng vì chủ yếu mình muốn là direction không được
    // gán lại (immutablle).
    cv::Point3f endPoint;
    /**
     * Notes: độ dài của direction luôn là 1 và được chuẩn hóa
     * ngay khi khởi tạo.
     */
    cv::Vec3f direction;

    RayLine() {}
    RayLine(cv::Point3f _endPoint, cv::Vec3f _direction);

    /**
     * Copy constructor
     */
    RayLine(const RayLine& other);
    
    /**
     * @return The projection point of the input point on the ray
     */
    cv::Point3f projection(const cv::Point3f& point) const;

    RayLine& operator= (const RayLine& other);
};


class ColorRayLine : public RayLine {
  public:
    cv::Vec3b color;

    ColorRayLine() {}

    ColorRayLine(cv::Point3f _endPoint, cv::Vec3f _direction, cv::Vec3b _color);
    ColorRayLine(const ColorRayLine& other);
    ColorRayLine& operator=(const ColorRayLine& other);
};


bool operator==(const RayLine&, const RayLine&);
inline bool operator!=(const RayLine& u, const RayLine& v) {
    return !(u == v);
}

bool operator==(const ColorRayLine&, const ColorRayLine&);
inline bool operator!=(const ColorRayLine& u, const ColorRayLine& v) {
    return !(u == v);
}

} // DepthModel
