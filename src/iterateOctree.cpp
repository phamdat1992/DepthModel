#include <opencv2/opencv.hpp>
#include <algorithm>
#include <iostream>
#include <limits>
#include <vector>
#include "Octree.hpp"
#include "Triangle3D.hpp"
#include "RayLine.hpp"
#include "geometry.hpp"
#include "iterateOctree.hpp"
#include "utils.hpp"

using namespace std;
using namespace cv;

namespace DepthModel {

Triangle3D* getFirstTriangleIntersectWithRay(const Octree<Triangle3D>& root, const RayLine& ray) {
    if (!Geometry::intersect(ray, root.getBoundingBox())) return NULL;

    float curSquareDistance = numeric_limits<float>::infinity();
    Triangle3D* ans = NULL;

    for (const Triangle3D& triangle: root.getData()) {
        if (!Geometry::intersect(ray, triangle)) continue;
        float sqrdis = Geometry::getIntersectionDistance_noChecking(ray, triangle);
        if (sqrdis < curSquareDistance) {
            curSquareDistance = sqrdis;
            if (!ans) ans = new Triangle3D();
            *ans = triangle;
        }
    }


    if (root.isLeaf()) return ans;

    vector<int> candidateChildren;
    
    for (int i = 8; i--; ) {
        if (Geometry::intersect(ray, root.getChild(i)->getBoundingBox())) {
            candidateChildren.push_back(i);
        }
    }

    sort(candidateChildren.begin(), candidateChildren.end(), [&](int u, int v) -> bool {
        const Octree<Triangle3D>& treeu = *root.getChild(u);
        const Octree<Triangle3D>& treev = *root.getChild(v);
        Vec3f posu = treeu.getPosition() + treeu.getSize() / 2;
        Vec3f posv = treev.getPosition() + treev.getSize() / 2;
        return distance2(posu, ray.endPoint) < distance2(posv, ray.endPoint);
    });

    for (int i = 0; i < (int)candidateChildren.size(); ++i) {
        Triangle3D* triangle = getFirstTriangleIntersectWithRay(
            *root.getChild(candidateChildren[i]),
            ray
        );
        if (!triangle) continue;
        float sqrdis = Geometry::getIntersectionDistance_noChecking(ray, *triangle);
        if (sqrdis < curSquareDistance) {
            curSquareDistance = sqrdis;
            if (!ans) ans = new Triangle3D();
            ans = triangle;
        }
        break;
    }
    return ans;
}

} // DepthModel

