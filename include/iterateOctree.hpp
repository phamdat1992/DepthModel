#pragma once 

#include <opencv2/opencv.hpp>
#include <vector>
#include "Octree.hpp"
#include "Box.hpp"
#include "Triangle3D.hpp"
#include "RayLine.hpp"
#include "geometry.hpp"

namespace DepthModel {

struct _getAllOctreeBoxWithRay {
  template<typename T>
  std::vector<Box> operator()(const Octree<T>& root, const RayLine& ray) {
    ans.clear();
    dfsHelper<T>(root, ray);
    return ans;
  }

private:
  std::vector<Box> ans;
  template<typename T> 
  void dfsHelper(const Octree<T>& root, const RayLine& ray) {
    if (!Geometry::intersect(ray, root.getBoundingBox())) {
      return ;
    }
    if (root.isLeaf()) {
      ans.push_back(root.getBoundingBox());
      return ;
    }
    for (int i = 8; i--; ) {
      dfsHelper<T>(*root.getChild(i), ray);
    }
  }
} getAllOctreeBoxWithRay;

  
} // DepthModel
