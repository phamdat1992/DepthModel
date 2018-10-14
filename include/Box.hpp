#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

namespace DepthModel {

class Box {
public:
    cv::Vec3f position;
    cv::Vec3f size;

    Box(const cv::Vec3f& pos, const cv::Vec3f& sz);

    cv::viz::WCube toVizWidget(const cv::viz::Color& color = cv::viz::Color::white());
};

} // DepthModel
