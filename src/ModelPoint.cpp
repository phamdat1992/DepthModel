#include <opencv2/opencv.hpp>
#include "ModelPoint.hpp"
#include "ModelContainer.hpp"
#include "RayLine.hpp"

using namespace cv;

namespace DepthModel {

const ModelPoint ModelPoint::NON_EXIST_POINT (
    ColorRayLine(), nullptr 
);

ModelPoint::ModelPoint()
    : position(Vec3f())
    , modelContainer(NULL)
{}

ModelPoint::ModelPoint(
    const ColorRayLine& firstCaptureRay,
    ModelContainer* _modelContainer,
    float distanceToRayEndPoint /* 0 */
)
    : captureRay(firstCaptureRay)
    , modelContainer(_modelContainer)
{
    if (!_modelContainer) return;
    this->position = _modelContainer->getNearestPoint(
        Vec3f(this->captureRay.endPoint) +
        this->captureRay.direction * distanceToRayEndPoint
    );
}

ModelPoint::ModelPoint(const ModelPoint& other)
    : ModelPoint(
        other.captureRay,
        other.modelContainer
    )
{
    this->position = other.position;
}

ModelPoint ModelPoint::move(float distance) const {
    ModelPoint ans(*this);
    Vec3f realPos = ans.getRealPosition();
    realPos += Vec3f(ans.captureRay.direction * distance);
    ans.position = this->modelContainer->getNearestPoint(realPos);
    return ans;
}

Vec3f ModelPoint::getRealPosition() const {
    return this->captureRay.projection(this->position);
}

bool ModelPoint::notExist() {
    return this->captureRay.direction == Vec3f();
}

ModelPoint& ModelPoint::operator=(const ModelPoint& other) {
    this->position = other.position;
    this->captureRay = other.captureRay;
    this->modelContainer = other.modelContainer;
    return *this;
}


bool operator==(const ModelPoint& u, const ModelPoint& v) {
    return (
        u.position == v.position &&
        u.captureRay == v.captureRay &&
        u.modelContainer == v.modelContainer
    );
}

} // DepthModel
