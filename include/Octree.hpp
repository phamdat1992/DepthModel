#pragma once
#include <opencv2/opencv.hpp>
#include <functional>
#include <list>
#include <utility>
#include <iostream>
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
        , fitCheck(fn)
        , nDataThreshold(_nDataThreshold)
        , nLevelThreshold(_nLevelThreshold)
        , nElements(0)
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
        //std::cerr << getPosition() << ' ' << getSize() << ' ' << nLevelThreshold << std::endl;
        if (!fitCheck(this->boundingBox, dat)) {
            return false;
        }
        ++this->nElements;
        if (this->child[0]) {
            for (int i = 0; i < 8; ++i) {
                if (this->child[i]->insert(dat)) {
                    return true;
                }
            }
        }
        this->data.push_back(dat);
        if (this->data.size() > this->nDataThreshold) {
            this->extendTree();
        }
        return true;
    }

    bool remove(const data_type& dat) {
        if (!fitCheck(this->boundingBox, dat)) {
            return false;
        }
        --this->nElements;
        bool foundAtChild = false;
        if (!this->isLeaf()) {
            for (int i = 8; i--; ) {
                if (!this->data[i]->remove(dat)) continue;
                foundAtChild = true;
                break;
            }
        }
        if (!foundAtChild) {
            for (auto i = this->data.begin(); i != this->data.end(); ++i) {
                if (*i != dat) continue;
                this->data.erase(i);
                break;
            }
        }
        if (this->nElements <= this->nDataThresHold) {
            this->shirnk();
        }
        return true;
    }


    inline const std::list<data_type>& getData() const {
        return this->data;
    }

    inline const Box& getBoundingBox() const {
        return this->boundingBox;
    }

    inline const Octree* getChild(int num) const {
        return this->child[num];
    }

    inline const cv::Vec3f& getPosition() const {
        return this->boundingBox.position;
    }

    inline const cv::Vec3f& getSize() const {
        return this->boundingBox.size;
    }

    inline bool isLeaf() const {
        return !this->child[0];
    }

    inline size_t getNElements() const {
        return this->nElements;
    }

    cv::viz::WWidgetMerger toVizWidget() {
        cv::viz::WWidgetMerger ans;
        ans.addWidget(this->boundingBox.toVizWidget());
        //for (auto& i: this->data) {
            //ans.addWidget(i.toVizWidget());
        //}
        if (!this->child[0]) return ans;
        for (int i = 8; i--; ) {
            ans.addWidget(this->child[i]->toVizWidget());
        }
        return ans;
    }
    
  protected:
    Box boundingBox;
    fitCheckFunc fitCheck;
    size_t nDataThreshold;
    size_t nLevelThreshold;
    size_t nElements;

    Octree<data_type> *child[8];
    std::list<data_type> data;

    void extendTree() {
        if (this->child[0] != NULL) {
            return;
        }
        if (nLevelThreshold <= 0) {
            return;
        }
        cv::Vec3f newSize = this->getSize() / 2;
        for (int i = 0; i < 8; ++i) {
            cv::Vec3f newPos = this->getPosition() + cv::Vec3f(
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

        for (int i = 0, sz = this->data.size(); i < sz; ++i) {
            data_type& d = this->data.front();
            bool canBePush = false;
            for (int j = 0; j < 8; ++j) {
                if (!this->child[j]->insert(d)) continue;
                canBePush = true;
                break;
            }
            if (!canBePush) {
                this->data.push_back(d);
            }
            this->data.pop_front();
        }
    }

    void shrinkTree() {
        if (this->isLeaf()) return;
        for (int i = 8; i--; ) {
            this->child[i]->shrinkTree();
            this->data.splice(this->data.end(), this->child[i]->data);  // O(1) babe =)
            delete this->data[i];
        }
    }

};

} // DepthModel
