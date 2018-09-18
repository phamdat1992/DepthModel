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
        //cout << i[0] << endl;
        if (*(i++) != 3) assert(false); // seem like if I do assert(*(i++) == 3), it will not working, maybe it is compiler's optimization.
        //cout << i[0] << ' ' << i[1] << ' ' << i[2] << endl;
        //clog << "COOL" << endl;
        bool ok = octree.insert(Triangle3D(
            cloudPoints[*(i++)],
            cloudPoints[*(i++)],
            cloudPoints[*(i++)]
        ));
        assert(ok);
    }
    displayWindow.showWidget("Teapot", viz::WMesh(teapotMesh));
    //displayWindow.showWidget("Octree", octree.toVizWidget());
    //displayWindow.setRenderingProperty("Teapot", REPRESENTATION, REPRESENTATION_WIREFRAME);
  
    displayWindow.spinOnce(33, true);
    while(!displayWindow.wasStopped()) {
        displayWindow.showWidget("crosshair", viz::WText("+", Point(displayWindow.getWindowSize()) / 2 - Point(5, 15), 20));
        RayLine theFckingRay = cameraPoseToCameraRay(displayWindow.getViewerPose());
    
        viz::WWidgetMerger boxes;
        bool hasBox = 0;
        for (int i = 8; i--; ) {
          for (auto b: getAllOctreeBoxWithRay(*octree.getChild(i), theFckingRay)) {
            hasBox = 1;
            boxes.addWidget(b.toVizWidget(viz::Color::red()));
          }
        }
    
        //if (hasBox) 
          //displayWindow.showWidget("Boxes", boxes);

        clog << endl;
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
    }

    return 0;
}
