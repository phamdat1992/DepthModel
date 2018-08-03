#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include "RayLine.hpp"

namespace DepthModel {

class ModelPoint;

/**
 * Interface cấu trúc dữ liệu quản lý các điểm của model.
 * Interface for a data structure that can manage points of model.
 *
 * Notes: đặt tên là interface theo naming convention của google.
 */
class PointBagInterface {
public:
    PointBagInterface() {}

    virtual void insert(const ModelPoint& point) = 0;

    /**
     * @return Điểm gần nhất mà RayLine đi qua trong tập điểm hoặc ModelPoint::NON_EXIST_POINT nếu không tồn tại điểm
     */
    virtual const ModelPoint& getFirstCross(const RayLine& ray) = 0;

    /**
     * @return Điểm có tọa độ bằng coordinate hoặc ModelPoint::NON_EXIST_POINT nếu điểm không tại.
     */
    virtual const ModelPoint& get(const cv::Vec3f& coordinate) = 0;

    virtual std::vector<ModelPoint> getAllPoints() = 0;

    virtual void remove(const ModelPoint& point) = 0;

    virtual void clear() = 0;
};

} // DepthModel
