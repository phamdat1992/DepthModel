#pragma once
#include <opencv2/opencv.hpp>
#include <functional>
#include <vector>
#include <utility>
#include "Box.hpp"

namespace DepthModel {

template<typename T>
class Octree {
  public:
    typedef T data_type;
    typedef std::function<bool(const Box&, const T&)> fitCheckFunc;

    Octree() {}

    Octree(
        const cv::Vec3f& _position,
        const cv::Vec3f& _size,
        fitCheckFunc fn,
        size_t _nDataThreshold = 8,
        size_t _nLevelThreshold = 8
    )
        : boundingBox(_position, _size)
        , size(_size)
        , fitCheck(fn)
        , nDataThreshold(_nDataThreshold)
        , nLevelThreshold(_nLevelThreshold)
        , child{0}
    {
    }

    ~Octree() {
        if (!this->child[0]) return;
        for (int i = 0; i < 8; ++i) {
            delete this->child[i];
        }
    }

    bool insert(const data_type& dat) {
        if (!fitCheck(this->boundingBox, dat)) {
            return false;
        }
        if (this->child[0]) {
            for (int i = 0; i < 8; ++i) {
                if (this->child[i]->insert(dat)) {
                    return false;
                }
            }
        }
        this->data.push_back(dat);
        if (this->data.size() > this->nDataThreshold) {
            this->extendTree();
        }
    }


    inline const std::vector<data_type>& getData() {
        return this->data;
    }

    inline const Box& getBoundingBox() const {
        return this->boundingBox;
    }

    inline const Octree* getChild(int num) {
        return this->child[num];
    }

    cv::viz::WWidgetMerger toVizWidget() {
        cv::viz::WWidgetMerger ans;
        ans.addWidget(this->boundingBox.toVizWidget());
        for (auto& i: this->data) {
            ans.addWidget(i.toVizWidget());
        }
        if (!this->child[0]) return ans;
        for (int i = 8; i--; ) {
            ans.addWidget(this->child[i]->toVizWidget());
        }
        return ans;
    }
    
  protected:
    Box boundingBox;
    cv::Vec3f position;
    cv::Vec3f size;
    fitCheckFunc fitCheck;
    size_t nDataThreshold;
    size_t nLevelThreshold;

    Octree<data_type> *child[8];
    std::vector<data_type> data;

    void extendTree() {
        if (this->child[0] != NULL) {
            return;
        }
        if (nLevelThreshold == 0) {
            return;
        }
        cv::Vec3f newSize = this->size / 2;
        for (int i = 0; i < 8; ++i) {
          cv::Vec3f newPos = this->position + cv::Vec3f(
                i & 1 ? newSize[0] : 0,
                i & 2 ? newSize[1] : 0,
                i & 4 ? newSize[2] : 0
            );
            this->child[i] = new Octree<data_type>(
                newPos, newSize,
                this->fitCheck,
                this->nDataThreshold, this->nLevelThreshold - 1
            );
        }
        int f = 0;
        for (int i = 0; i < (int)this->data.size(); ++i) {
            data_type& d = this->data[i];
            bool canBePush = false;
            for (int j = 0; j < 8; ++j) {
                if (!this->child[j]->insert(d)) continue;
                canBePush = true;
                break;
            }
            if (canBePush) continue;
            std::swap(this->data[i], this->data[f++]);
        }
        this->data.resize(f);
    }
};

} // DepthModel
