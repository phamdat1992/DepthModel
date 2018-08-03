#pragma once

#include <opencv2/opencv.hpp>
#include "RayLine.hpp"
#include "ModelContainer.hpp"

namespace DepthModel {

class ModelPoint {
public:
    static const ModelPoint NON_EXIST_POINT;
    /**
     * Tọa độ xấp xỉ của điểm trên lưới tọa độ của model.
     * Tọa độ chính xác không sử dụng vì mục đích tính toán của phương pháp này.
     */
    cv::Vec3f position;

    /**
     * Tia đầu tiên hứng điểm
     */
    ColorRayLine captureRay;

    ModelContainer* modelContainer;

    ModelPoint();

    /**
     * @param firstCatureRay
     * @param _modelProperties
     * @param distanceToRayEndPoint khoảng cách ban đầu điểm đến mút của tia.
     */
    ModelPoint(const ColorRayLine& firstCaptureRay, ModelContainer* ModelContainer, float distanceToRayEndPoint = 0);
    ModelPoint(const ModelPoint&);
    ~ModelPoint() {}

    /**
     * Đẩy điểm màu đi với khoảng cách là distance
     * @return điểm mới đã được đẩy đi khoảng cách distance.
     * Notes: nên đẩy điểm màu mới khoảng tối thiểu là max chiều
     *        dài của 1 cell trong model.
     */
    ModelPoint move(float distance) const;

    bool notExist();

    inline cv::Vec3b getColor() {
        return this->captureRay.color;
    }

    ModelPoint& operator=(const ModelPoint& other);

protected:
    /**
     * Trả về tọa độ chính xác của điểm màu
     */
    cv::Vec3f getRealPosition() const;
};

bool operator==(const ModelPoint&, const ModelPoint&);
inline bool operator!=(const ModelPoint& u, const ModelPoint& v) {
  return !(u == v);
}


} // DepthModel
