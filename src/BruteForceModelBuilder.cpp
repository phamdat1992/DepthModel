#include <opencv2/opencv.hpp>
#include <limits>
#include <vector>
#include "ModelPoint.hpp"
#include "PointBagInterface.hpp"
#include "PoseWrapper.hpp"
#include "RayLine.hpp"
#include "utils.hpp"
#include "BruteForceModelBuilder.hpp"

using namespace cv;

using namespace std;

namespace DepthModel {

BruteForcePointBag::BruteForcePointBag(
    float disThresHold,
    float _precision
)
    : distanceThresHold(disThresHold)
    , precision(_precision)
{}

Vec3i BruteForcePointBag::toVec3i(const Vec3f& u) {
    return Vec3i(
        cvRound(u[0] / this->precision),
        cvRound(u[1] / this->precision),
        cvRound(u[2] / this->precision)
    );
}

void BruteForcePointBag::insert(const ModelPoint& point) {
    this->data[this->toVec3i(point.position)] = point;
}

const ModelPoint& BruteForcePointBag::getFirstCross(const RayLine& ray) {
    //ModelPoint ans = ModelPoint::NON_EXIST_POINT;
    //float minDis = -1;
    //for (const ModelPoint& point: this->data) {
        //float dis = norm(point->position - ray.projection(point));
        //if (dis > this->distanceThresHold) continue;
        //dis = norm();
    //}
    return ModelPoint::NON_EXIST_POINT;  // currently do nothing
}

const ModelPoint& BruteForcePointBag::get(const Vec3f& coordinates) {
    auto itor = this->data.find(this->toVec3i(coordinates));
    if (itor == this->data.end()) {
      return ModelPoint::NON_EXIST_POINT;
    }
    return itor->second;
};

vector<ModelPoint> BruteForcePointBag::getAllPoints() {
    vector<ModelPoint> ans;
    for (auto& i: this->data) {
        ans.push_back(i.second);
    }
    return ans;
}

void BruteForcePointBag::remove(const ModelPoint& point) {
    auto itor = this->data.find(this->toVec3i(point.position));
    if (itor != this->data.end()) {
        this->data.erase(itor);
    }
}

void BruteForcePointBag::clear() {
    this->data.clear();
}

viz::WCloud BruteForcePointBag::toVizWidget() {
    vector<Point3f> points;
    vector<Vec3b> colors;
    for (auto& i: this->data) {
        auto& point = i.second;
        points.push_back(point.position);
        colors.push_back(point.getColor());
    }
    return viz::WCloud(points, colors);
}



BruteForceModelBuilder::BruteForceModelBuilder(
    const ModelContainer& _container,
    float bruteForcePointBagParam,
    const CameraRayCalculator& _cameraRayCalculator,
    float _placementDistance
)
    : ModelBuilderBase(
        _container,
        new BruteForcePointBag(bruteForcePointBagParam),
        _cameraRayCalculator
    )
    , placementDistance(_placementDistance)
{
}

float BruteForceModelBuilder::colorDistanceThresHold = 4000;

bool BruteForceModelBuilder::init(Mat imageFrame, const PoseWrapper& pose) {
    PoseWrapper invPose = pose.inv();
    unsigned char* imgdat = imageFrame.data;
    int rows = imageFrame.rows, cols = imageFrame.cols;
    Vec3f campos = invPose.transVec;

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            unsigned char* cell = imgdat + (r * cols + c) * 3;
            Vec3b color(cell);
            Vec3f dir = this->cameraRayCalculator.getRay(c, r);

            double moveDis = norm(dir * this->placementDistance); // because dir[2] == 1
            dir = invPose.apply(dir, PoseWrapper::ROTATE_FLAG);
            ColorRayLine ray(campos, dir, color);

            ModelPoint u(ray, &(this->container), moveDis);
            if (!this->container.contains(u.position)) {
                continue;
            }
            this->pointBag->insert(u);
        }
    }
    return true;
}

void BruteForceModelBuilder::mainProcess(Mat imageFrame, const PoseWrapper& pose) {
    unsigned char* imgdat = imageFrame.data;
    int rows = imageFrame.rows, cols = imageFrame.cols;

    float baseMoveDistance = norm(this->container.cellSize);

    vector<ModelPoint> allPoints = this->getAllPoints();
    this->pointBag->clear();
    for (ModelPoint& u: allPoints) {
        int cnt = 0;
        float moveDistance = baseMoveDistance;
        while (true)
        {
            Mat posMat = this->cameraRayCalculator.camMat * Mat(pose.apply(u.position));
            float* posDat = (float*) posMat.data;

            int r = cvRound(posDat[1] / posDat[2]);
            int c = cvRound(posDat[0] / posDat[2]);

            if (c < 0 or c >= cols) break;
            if (r < 0 or r >= rows) break;

            unsigned char* cell = imgdat + (r * cols + c) * 3;

            Vec3b color(cell);
            if (colorDistance(color, u.getColor()) < colorDistanceThresHold) {
                break;
            }
            u = u.move(moveDistance);
            //moveDistance += baseMoveDistance;
            if (!this->container.contains(u.position)) {
                break;
            }
            //if (++cnt == 1) break;
        }
        if (this->container.contains(u.position)) {
            this->pointBag->insert(u);
        }
    }
}


} // DepthModel
