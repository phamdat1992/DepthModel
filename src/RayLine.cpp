#include <opencv2/opencv.hpp>
#include "RayLine.hpp"

using namespace cv;

namespace DepthModel {


/**
 * RayLine definition
 */

RayLine::RayLine(Point3f _endPoint, Point3f _direction)
  : endPoint(_endPoint)
  , direction(_direction)
{}

RayLine::RayLine(const RayLine& other)
  : RayLine(other.endPoint, other.direction)
{}

Point3f RayLine::projection(const Point3f& point)
{
    // Yo here is some math :D
    // The projection point of input point on the ray
    // is actually the projection point of the ray's endpoint
    // on the plane cross the input point with ray's direction
    // as normal vector.
    // And the solution is here:
    // https://math.stackexchange.com/a/100766
    float t = this->direction.dot(point - this->endPoint);
#define sqr(x) ((x) * (x))
    t /= (
        sqr(this->direction.x) +
        sqr(this->direction.y) +
        sqr(this->direction.z)
    );
#undef sqr
    return this->endPoint + this->direction * t;
}


/**
 * ColorRayLine definition
 */
ColorRayLine::ColorRayLine(
    Point3f _endPoint, Point3f _direction, Vec3b _color
)
    : RayLine(_endPoint, _direction)
    , color(_color)
{}

ColorRayLine::ColorRayLine(const ColorRayLine& other)
    : ColorRayLine(other.endPoint, other.direction, other.color)
{}

} // DepthModel
