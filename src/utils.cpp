#include <opencv2/opencv.hpp>
#include <string>
#include "PoseWrapper.hpp"
#include "utils.hpp"

using namespace cv;
using namespace std;

namespace DepthModel {

float colorDistance(const Vec3b& u, const Vec3b& v) {
    float ans = 0;
#define sqr(x) ((x) * (x))
    ans += sqr(u[0] - v[0]);
    ans += sqr(u[1] - v[1]);
    ans += sqr(u[2] - v[2]);
#undef sqr
    return ans;
}

// Base on this code http://www.songho.ca/opengl/gl_camera.html
PoseWrapper lookAt(
    const Vec3f& from, const Vec3f& to, const Vec3f& tmp
) { 
    Vec3f forward = normalize(from - to); 
    Vec3f left = normalize(tmp).cross(forward); 
    Vec3f up = forward.cross(left); 
 
    Vec3f trans;
 
    Mat rotMat = (Mat_<float>(3, 3) <<
      left[0], left[1], left[2],
      up[0], up[1], up[2],
      forward[0], forward[1], forward[2]
    );
 
    trans[0]= -left.dot(from);
    trans[1]= -up.dot(from);
    trans[2]= -forward.dot(from);
 
    return PoseWrapper(rotMat, trans);
}

Vec3f coordinatesOnGlobe(float gamma, float theta, float radius) {
    Vec3f ans;
    ans[1] = cos(-theta) * radius;
    radius *= sin(-theta);
    ans[0] = sin(gamma) * radius;
    ans[2] = cos(gamma) * radius;
    return ans;
}

int createRangeTrackbar(
    const string& name, const string& winName,
    int* value, int minVal, int maxVal
) {
    int ans = createTrackbar(name, winName, value, maxVal - minVal + 1);
    setTrackbarMin(name, winName, minVal);
    setTrackbarMax(name, winName, maxVal);
    return ans;
}

size_t Vec3iHash::operator()(const Vec3i& u) const {
    hash <int> ih;
    return ih(u[0]) ^ ih(u[1]) ^ ih(u[2]);
}

float distance2(Vec3f a, const Vec3f& b) {
    a -= b;
    return (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}


} // DepthModel
