#include <opencv2/opencv.hpp>
#include "CameraRayCalculator.hpp"

using namespace cv;
  
namespace DepthModel {
CameraRayCalculator::CameraRayCalculator(
  float focalLength,
  float sensorPhysicalWidth, float sensorPhysicalHeight,
  int imageWidth, int imageHeight
)
    : camMat((Mat_<float>(3, 3) <<
        focalLength * sensorPhysicalWidth / imageWidth , 0, imageWidth / 2.0,
        0, focalLength * sensorPhysicalHeight / imageHeight, imageHeight / 2.0,
        0, 0, 1
    ))
{ }

CameraRayCalculator::CameraRayCalculator(Mat _camMat)
    : camMat(_camMat.clone())
{ }

CameraRayCalculator::CameraRayCalculator(const CameraRayCalculator& other)
    : camMat(other.camMat.clone())
{}

Vec3f CameraRayCalculator::getRay(float x, float y) {
    float* data = (float*)this->camMat.data;
    x -= data[2];
    y -= data[5];
    x /= data[0];
    y /= data[4];
    return -Vec3f(x, y, 1);
}

} // DepthModel
