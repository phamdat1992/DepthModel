#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <string>
#include <vector>
#include "CameraRayCalculator.hpp"
#include "BruteForceModelBuilder.hpp"
#include "Visualizer.hpp"
#include "utils.hpp"
#include "Octree.hpp"
#include "Triangle3D.hpp"
#include "geometry.hpp"
#include "iterateOctree.hpp"

using namespace cv;
using namespace std;
using namespace viz;
using namespace DepthModel;

int main(int, char**) {
    viz::Viz3d displayWindow("Display window");
    displayWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem(10));
    //displayWindow.showWidget("Point cloud", mb->toVizWidget());

    viz::Mesh teapotMesh = viz::Mesh::load("teapot.ply");

    Vec3f minCoor(INT_MAX, INT_MAX, INT_MAX), maxCoor(INT_MIN, INT_MIN, INT_MIN);
    Vec3f* cloudPoints = teapotMesh.cloud.ptr<Vec3f>();
    for (int i = 0; i < teapotMesh.cloud.cols; ++i) {
        Vec3f& point = cloudPoints[i];
        for (int f = 0; f < 3; ++f) {
            minCoor[f] = min(minCoor[f], point[f]);
            maxCoor[f] = max(maxCoor[f], point[f]);
        }
    }

    Octree<Triangle3D> octree(minCoor, maxCoor - minCoor, static_cast<bool(*)(const Box&, const Triangle3D&)>(Geometry::inside), 8, 16);
    for (unsigned int* i = teapotMesh.polygons.ptr<unsigned int>(), f = 0; f < teapotMesh.polygons.cols; f+= 4) {
        assert(*(i++) == 3);
        octree.insert(Triangle3D(
            cloudPoints[*(i++)],
            cloudPoints[*(i++)],
            cloudPoints[*(i++)]
        ));
    }
    displayWindow.showWidget("Teapot", viz::WMesh(teapotMesh));
    //displayWindow.showWidget("Octree", octree.toVizWidget());


  
    displayWindow.spinOnce(33, true);
    while(!displayWindow.wasStopped()) {
        RayLine theFckingRay = cameraPoseToCameraRay(displayWindow.getViewerPose());
        displayWindow.showWidget("Ray", viz::WSphere(Vec3d(theFckingRay.endPoint + theFckingRay.direction * 5), 0.05));
        viz::WWidgetMerger boxes;
    
        for (int i = 8; i--; ) {
          for (auto b: getAllOctreeBoxWithRay(*octree.getChild(i), theFckingRay)) {
            boxes.addWidget(b.toVizWidget(viz::Color::red()));
          }
        }
    
        displayWindow.showWidget("Boxes", boxes);

        //clog << theFckingRay.endPoint << ' ' << theFckingRay.direction << endl;
        Triangle3D* theFckingTriangle = getFirstTriangleIntersectWithRay(octree, theFckingRay);
        bool hasTriangle = !!theFckingTriangle;
        if (!theFckingTriangle) {
            //clog << "No fcking triangle" << endl;
        } else {
            displayWindow.showWidget("Intersected triangle", theFckingTriangle->toVizWidget());
            //displayWindow.showWidget(
                //"intersection point",
                //viz::WSphere(Vec3d(Geometry::getIntersection_noChecking(theFckingRay, *theFckingTriangle)), 0.3)
            //);
            delete theFckingTriangle;
        }
        displayWindow.spinOnce(33, true);
        displayWindow.removeWidget("Ray");
        displayWindow.removeWidget("Boxes");
        if (hasTriangle)
          displayWindow.removeWidget("Intersected triangle");
    }

    return 0;
}
