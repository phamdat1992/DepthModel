#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include "PoseWrapper.hpp"

namespace DepthModel {

float colorDistance(const cv::Vec3b& u, const cv::Vec3b& v);

PoseWrapper lookAt(const cv::Vec3f& from, const cv::Vec3f& to, const cv::Vec3f& tmp = cv::Vec3f(0, 1, 0));
cv::Vec3f coordinatesOnGlobe(float gamma, float theta, float radius);

int createRangeTrackbar(const std::string& name, const std::string& winName, int* value, int minVal, int maxVal);

struct Vec3iHash {
    size_t operator()(const cv::Vec3i&) const;
};

float distance2(cv::Vec3f a, const cv::Vec3f& b);

inline float distance(const cv::Vec3f& a, const cv::Vec3f& b) {
    return cv::sqrt(distance2(a, b));
}

} // DepthModel

