#pragma once
#include <opencv2/opencv.hpp>

namespace DepthModel {

class PoseWrapper {
public:
    enum {
        ROTATE_FLAG = 1,
        TRANSLATE_FLAG = 2
    };
    cv::Mat rotMat;
    cv::Vec3f transVec;

    PoseWrapper() {}
    PoseWrapper(cv::Mat _rotMat, cv::Vec3f _transVec);
    PoseWrapper(const PoseWrapper& other);

    cv::Vec3f apply(cv::Vec3f v, size_t flag = ROTATE_FLAG | TRANSLATE_FLAG) const;
    PoseWrapper inv() const;

    PoseWrapper& operator=(const PoseWrapper& other);
};

} // DepthModel
