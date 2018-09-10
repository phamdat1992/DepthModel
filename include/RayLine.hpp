/**
 * @author: quangloc99
 */

#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

namespace DepthModel {

class RayLine {
  public:
    // Const được dùng vì chủ yếu mình muốn là direction không được
    // gán lại (immutablle).
    cv::Vec3f endPoint;
    /**
     * Notes: độ dài của direction luôn là 1 và được chuẩn hóa
     * ngay khi khởi tạo.
     */
    cv::Vec3f direction;

    RayLine() {}
    RayLine(cv::Vec3f _endPoint, cv::Vec3f _direction);

    /**
     * Copy constructor
     */
    RayLine(const RayLine& other);
    
    /**
     * @return The projection point of the input point on the ray
     */
    cv::Vec3f projection(const cv::Vec3f & point) const;

    RayLine& operator= (const RayLine& other);

    virtual cv::viz::WArrow toVizWidget(float length = 3);
};


class ColorRayLine : public RayLine {
  public:
    cv::Vec3b color;

    ColorRayLine() {}

    ColorRayLine(cv::Vec3f _endPoint, cv::Vec3f _direction, cv::Vec3b _color);
    ColorRayLine(const ColorRayLine& other);
    ColorRayLine& operator=(const ColorRayLine& other);

    virtual cv::viz::WArrow toVizWidget(float length = 3) override;
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
