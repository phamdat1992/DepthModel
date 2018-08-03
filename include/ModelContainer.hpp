#pragma once

#include <opencv2/opencv.hpp>
#include "PoseWrapper.hpp"

namespace DepthModel {

class ModelContainer {
protected:
    /**
     * Notes: điểm gốc là tâm của hộp.
     */
    PoseWrapper pose;

    /**
     * Cache lại inverse của Pose
     */
    PoseWrapper inversePose;
public:
    /**
     * Kích thước 3 trục tọa độ của model, tính bằng số ô/trục.
     */
    cv::Vec3i nCell;

    /**
     * Kích thước của 1 ô trong model.
     */
    cv::Vec3f cellSize;

    ModelContainer() {}
    ModelContainer(const cv::Vec3i& _nCell, const cv::Vec3f& _cellSize);
    ModelContainer(const ModelContainer& other);

    /**
     * @return Điểm nguyên trong model gần với point nhất
     */
    cv::Vec3f getNearestPoint(cv::Vec3f point) const;

    bool contains(cv::Vec3f point) const;

    inline const PoseWrapper& getPose() const {
        return this->pose;
    }
    inline void setPose(const PoseWrapper& _pose) {
        this->pose = _pose;
        this->inversePose = this->pose.inv();
    }
};

} // DepthModel
