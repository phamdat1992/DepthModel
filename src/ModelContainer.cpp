#include <opencv2/opencv.hpp>
#include "ModelContainer.hpp"

using namespace cv;

namespace DepthModel {

ModelContainer::ModelContainer(const Vec3i& _nCell, const Vec3f& _cellSize)
    : nCell(_nCell)
    , cellSize(_cellSize)
{}

ModelContainer::ModelContainer(const ModelContainer& other)
    : ModelContainer(other.nCell, other.cellSize)
{
    this->setPose(other.pose);
}

Vec3f ModelContainer::getNearestPoint(Vec3f point) const {
    point = this->pose.apply(point);
  
    for (int i = 0; i < 3; ++i) {
      float half = this->nCell[i] * this->cellSize[i] / 2;
      float& ans = point[i];
      ans += half;
      ans = cvRound(ans / this->cellSize[i]) * this->cellSize[i];
      ans -= half;
    }
    
    return this->inversePose.apply(point);
}

bool ModelContainer::contains(Vec3f point) const {
    point = this->pose.apply(point);
    for (int i = 0; i < 3; ++i) {
        float half = this->nCell[i] * this->cellSize[i] / 2;
        if (point[i] > half or point[i] < -half) return false;
    }
    return true;
}

} // DepthModel
