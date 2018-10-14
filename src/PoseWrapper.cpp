#include <opencv2/opencv.hpp>
#include "PoseWrapper.hpp"

using namespace cv;

namespace DepthModel {

PoseWrapper::PoseWrapper(Mat _rotMat, Vec3f _transVec)
    : rotMat(_rotMat)
    , transVec(_transVec)
{}

PoseWrapper::PoseWrapper(const PoseWrapper& other)
    : PoseWrapper(other.rotMat, other.transVec)
{}

Vec3f PoseWrapper::apply(Vec3f v, size_t flag /* 3 */) const {
    if (flag & PoseWrapper::ROTATE_FLAG) {
        v = Vec3f((float*)Mat(this->rotMat * Mat(v)).data);
    }
    if (flag & PoseWrapper::TRANSLATE_FLAG) {
        v += this->transVec;
    }
    return v;
}

PoseWrapper PoseWrapper::inv() const {
    PoseWrapper ans(this->rotMat.inv(), this->transVec);
    Mat temp = -ans.rotMat * Mat(this->transVec);
    ans.transVec = Vec3f((float*)temp.data);
    return ans;
}

PoseWrapper& PoseWrapper::operator=(const PoseWrapper& other) {
    this->rotMat = other.rotMat;
    this->transVec = other.transVec;
    return *this;
}

} // DepthModel
