#include <opencv2/opencv.hpp>
#include <vector>
#include <unordered_map>
#include "ModelBuilder.hpp"
#include "ModelPoint.hpp"
#include "PointBagInterface.hpp"
#include "PoseWrapper.hpp"
#include "RayLine.hpp"
#include "utils.hpp"

namespace DepthModel {

/**
 * Thiết kế dựa trên code của Giang
 */
class BruteForcePointBag : public PointBagInterface {
protected:
    std::unordered_map<cv::Vec3i, ModelPoint, Vec3iHash> data;
    float precision;

    cv::Vec3i toVec3i(const cv::Vec3f& u);

public:   
    float distanceThresHold;
    BruteForcePointBag(float disThresHold, float _precision = 0.001);

    virtual void insert(const ModelPoint& point) override;
    virtual const ModelPoint& getFirstCross(const RayLine& ray) override;

    virtual const ModelPoint& get(const cv::Vec3f& coordinates) override;

    virtual std::vector<ModelPoint> getAllPoints() override;

    virtual void remove(const ModelPoint& point) override;

    virtual void clear() override;
};


class BruteForceModelBuilder : public ModelBuilderBase {
protected:
    float placementDistance;
    virtual bool init(cv::Mat imageFrame, const PoseWrapper& pose) override;
    virtual void mainProcess(cv::Mat imageFrame, const PoseWrapper& pose) override;
public:
    static float colorDistanceThresHold;
    BruteForceModelBuilder(
        const ModelContainer& _container,
        float bruteForcePointBagParam,
        const CameraRayCalculator& _cameraRayCalculator,
        float _placementDistance
    );
};

} // DepthModel
