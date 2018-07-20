/**
 * @author: quangloc99
 */

#pragma once
#include <opencv2/opencv.hpp>

namespace DepthModel {

class RayLine {
  public:
    cv::Point3f endPoint;
    cv::Point3f direction;

    RayLine(cv::Point3f _endPoint, cv::Point3f _direction);

    /**
     * Copy constructor
     */
    RayLine(const RayLine& other);
    
    /**
     * @return The projection point of the input point on the ray
     */
    cv::Point3f projection(const cv::Point3f& point);
};

class ColorRayLine : public RayLine {
  public:
    cv::Vec3b color;

    ColorRayLine(cv::Point3f _endPoint, cv::Point3f _direction, cv::Vec3b _color);
    ColorRayLine(const ColorRayLine& other);
};

} // DepthModel
