#include <opencv2/opencv.hpp>
#include "Box.hpp"

using namespace cv;
namespace DepthModel {

Box::Box(const cv::Vec3f& pos, const cv::Vec3f& sz)
  : position(pos), size(sz)
{}

viz::WCube Box::toVizWidget(const viz::Color& color) {
    return viz::WCube(Point3f(this->position), Point3f(this->position + this->size), true, color);
}

} // DepthModel
