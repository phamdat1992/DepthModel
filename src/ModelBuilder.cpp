#include <opencv2/opencv.hpp>
#include "CameraRayCalculator.hpp"
#include "ModelContainer.hpp"
#include "PointBagInterface.hpp"
#include "ModelBuilder.hpp"

using namespace cv;

namespace DepthModel {

ModelBuilderBase::ModelBuilderBase(
    const ModelContainer& _container,
    PointBagInterface* _pointBag,
    const CameraRayCalculator& _cameraRayCalculator
)
    : container(_container)
    , pointBag(_pointBag)
    , cameraRayCalculator(_cameraRayCalculator)
    , initialized(false)
{
}


ModelBuilderBase::~ModelBuilderBase() {
    delete this->pointBag;
}

ModelBuilderBase::ModelBuilderBase(const ModelBuilderBase& other)
    : ModelBuilderBase(other.container, other.pointBag, other.cameraRayCalculator)
{}

void ModelBuilderBase::process(Mat imageFrame, const PoseWrapper& pose) {
    if (!this->initialized) {
        this->initialized = this->init(imageFrame, pose);
    } else {
        this->mainProcess(imageFrame, pose);
    }
}

viz::WWidgetMerger ModelBuilderBase::toVizWidget() {
    viz::WWidgetMerger ans;
    ans.addWidget(this->pointBag->toVizWidget());
    return ans;
}
    
} // DepthModel
