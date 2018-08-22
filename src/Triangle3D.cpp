#include <opencv2/opencv.hpp>
#include "RayLine.hpp"
#include "Triangle3D.hpp"

using namespace cv;

namespace DepthModel {

Triangle3D::Triangle3D() {}
Triangle3D::Triangle3D(
    const Vec3f& a,
    const Vec3f& b,
    const Vec3f& c
) {
    verties[0] = a;
    verties[1] = b;
    verties[2] = c;
}

Triangle3D::Triangle3D(const Triangle3D& other) {
    for (int i = 3; i--; ) {
        verties[i] = other.verties[i];
    }
}

Triangle3D& Triangle3D::operator= (const Triangle3D& other) {
    for (int i = 3; i--; ) {
        verties[i] = other.verties[i];
    }
    return *this;
}

} // DepthModel
