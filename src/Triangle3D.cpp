#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstring>
#include "RayLine.hpp"
#include "Triangle3D.hpp"

using namespace cv;

namespace DepthModel {

Triangle3D::Triangle3D()
  : Triangle3D(Vec3f(), Vec3f(), Vec3f())
{}

Triangle3D::Triangle3D(
    const Vec3f& a,
    const Vec3f& b,
    const Vec3f& c
) {
    vertices[0] = a;
    vertices[1] = b;
    vertices[2] = c;
}

Triangle3D::Triangle3D(const Triangle3D& other) {
    for (int i = 3; i--; ) {
        vertices[i] = other.vertices[i];
    }
}

Triangle3D& Triangle3D::operator= (const Triangle3D& other) {
    for (int i = 3; i--; ) {
        vertices[i] = other.vertices[i];
    }
    return *this;
}

viz::WPolyLine Triangle3D::toVizWidget() {
    Mat points(4, 1, CV_32FC3);
    memcpy(points.data, this->vertices, sizeof(this->vertices));
    points.at<Vec3f>(3, 0) = this->vertices[0];
    return viz::WPolyLine(points, viz::Color::cyan());
}

} // DepthModel
