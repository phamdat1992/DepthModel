#include <opencv2/opencv.hpp>
#include "Box.hpp"

using namespace cv;
namespace DepthModel {

Box::Box(const cv::Vec3f& pos, const cv::Vec3f& sz)
  : position(pos), size(sz)
{}

} // DepthModel
