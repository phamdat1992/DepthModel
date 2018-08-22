#include <opencv2/opencv.hpp>
#include "Box.hpp"
#include "RayLine.hpp"
#include "Triangle3D.hpp"

namespace DepthModel {

namespace Geometry {

bool inside(const Box& box, const cv::Vec3f& point);
bool inside(const Box& box, const Triangle3D& triangle);

bool intersect(const RayLine& ray, const Triangle3D& triangle);
bool intersect(const RayLine& ray, const Box& box);

} // Geometry

} // DepthModel
