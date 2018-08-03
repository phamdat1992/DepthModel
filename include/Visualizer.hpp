#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include "CameraRayCalculator.hpp"
#include "ModelBuilder.hpp"
#include "ModelContainer.hpp"
#include "ModelPoint.hpp"
#include "PoseWrapper.hpp"

namespace DepthModel {

class Visualizer {
public:
    static cv::Scalar containerColor;
    static cv::Scalar cameraColor;
    static float cameraThickness;
    static float pointRadius;

    PoseWrapper viewerPose;
    cv::Mat camMat;

    cv::Mat result;

    Visualizer() {}

    void drawModelContainer(const ModelContainer& container);

    void drawCamera(
        cv::Size2f camSize,
        CameraRayCalculator& rayCaltor,
        const PoseWrapper& camPose
    );

    void drawPoints(std::vector<ModelPoint> points);
    void drawModel(ModelBuilderBase* model);
};

} // DepthModel
