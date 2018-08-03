#pragma once 

#include <opencv2/opencv.hpp>
#include <vector>
#include "CameraRayCalculator.hpp"
#include "ModelContainer.hpp"
#include "ModelPoint.hpp"
#include "PointBagInterface.hpp"
#include "PoseWrapper.hpp"
#include "RayLine.hpp"

namespace DepthModel {

class ModelBuilderBase {
protected:
    ModelContainer container;
    PointBagInterface* pointBag;
    CameraRayCalculator cameraRayCalculator;

    bool initialized;

    ModelBuilderBase(
        const ModelContainer& _container,
        PointBagInterface* _pointBag,
        const CameraRayCalculator& _cameraRayCalculator
    );

    /**
     * Sử dụng để khởi tạo tập điểm ban đầu.
     * Phương thức được gọi trong phương thức process.
     * @return true nếu khởi tạo thành công.
     */
    virtual bool init(cv::Mat imageFrame, const PoseWrapper& pose) = 0;

    /**
     * Phương thức xử lý chính.
     * Phương thức được gọi trong phương thức process.
     */
    virtual void mainProcess(cv::Mat imageFrame, const PoseWrapper& pose) = 0;

public:

    ModelBuilderBase(const ModelBuilderBase& other);

    virtual ~ModelBuilderBase();

    virtual void process(cv::Mat imageFrame, const PoseWrapper& pose);

    inline const ModelContainer& getContainer() const {
        return this->container;
    }

    inline std::vector<ModelPoint> getAllPoints() {
        return this->pointBag->getAllPoints();
    }

    inline const CameraRayCalculator& getCameraRayCalculator() {
        return this->cameraRayCalculator;
    }
};

} // DepthModel

