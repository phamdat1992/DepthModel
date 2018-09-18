#include <opencv2/opencv.hpp>
#include <cmath>
#include <limits>
#include <utility>
#include "Box.hpp"
#include "RayLine.hpp"
#include "Triangle3D.hpp"
#include "geometry.hpp"

using namespace cv;
using namespace std;
namespace DepthModel::Geometry {

bool inside(const Box& box, const Vec3f& point) {
#define inrange(i) \
    point[i] >= box.position[i] && \
    point[i] <= box.position[i] + box.size[i]

  return inrange(0) && inrange(1) && inrange(2);
#undef inrange
}

bool inside(const Box& box, const Triangle3D& triangle) {
#define check(i) \
    inside(box, triangle.vertices[i])
    return check(0) && check(1) && check(2);
#undef check
}

bool intersect(const RayLine& ray, const Triangle3D& triangle) {
    // This function uses algorithm described in this answer:
    // https://stackoverflow.com/a/42752998
    // with some modifications.
    // 
    // My explaination for the above algorithm:
    // 1) Pick 2 point that far away in both direction, let's call them q1, q2.
    // 2) Test if q1 and q2 lie on different sides of the plane that contains the triangle.
    // 3) Test if the intersection point of the line with the plane lie inside the triangle.
    //
    // Modification:
    // In step 1) pick q1 is the ray's endpoint, p2 is far away from q1 in the ray's direction.
    // Step 2 and 3 stay the same. In step 2, if q1 and q2 do not lie on diffirent sides of the plane,
    // the triangle is lie "behind" the ray's endpoint.
    const float veryBig = 1e8;

    // Step 1:
    const Vec3f& q1 = ray.endPoint;
    Vec3f q2 = q1 + ray.direction * veryBig;

    Vec3f q1a = triangle.vertices[0] - q1;
    Vec3f q1b = triangle.vertices[1] - q1;
    Vec3f q1c = triangle.vertices[2] - q1;

    Vec3f q2a = triangle.vertices[0] - q2;
    Vec3f q2b = triangle.vertices[1] - q2;
    Vec3f q2c = triangle.vertices[2] - q2;

#define sign(x) ((x) < 0 ? -1 : x > 0)

    // Step 2:
    if (sign(q1c.dot(q1a.cross(q1b))) * sign(q2c.dot(q2a.cross(q2b))) > 0) return false;
    
    // Step 3:
    Vec3f q1q2 = q2 - q1;
    float x = sign(q1b.dot(q1q2.cross(q1a)));
    float y = sign(q1c.dot(q1q2.cross(q1b)));
    float z = sign(q1a.dot(q1q2.cross(q1c)));
    if (x * y < 0) return false;
    if (x * z < 0) return false;
    return true;
#undef sign
}

/**
 * The idea got from this site.
 * https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection
 */
bool intersect(const RayLine& ray, const Box& box) {
    float tmax = numeric_limits<float>::max();
    float tmin = numeric_limits<float>::min();
    for (int i = 0; i < 3; ++i) {
      float curmin = (box.position[i] - ray.endPoint[i]) / ray.direction[i];
      float curmax = (box.position[i] + box.size[i] - ray.endPoint[i]) / ray.direction[i];
      if (curmin > curmax) swap(curmin, curmax);
      if (curmin > tmax || tmin > curmax) return false;
      if (curmin > tmin) tmin = curmin;
      if (curmax < tmax) tmax = curmax;
    }
    return true; 
}

/**
 * The idea was also got from here
 * https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection
 */
float getIntersectionDistance_noChecking(const RayLine& ray, const Triangle3D& triangle) {
    Vec3f normal = (triangle.vertices[1] - triangle.vertices[0]).cross(triangle.vertices[2] - triangle.vertices[0]);
    return (triangle.vertices[0] - ray.endPoint).dot(normal) / ray.direction.dot(normal);
}

Vec3f getIntersection_noChecking(const RayLine& ray, const Triangle3D& triangle) {
    return ray.endPoint + getIntersectionDistance_noChecking(ray, triangle) * ray.direction;
}

} // DepthModel::Geometry
