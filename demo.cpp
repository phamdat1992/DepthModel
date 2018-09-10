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

const string capture_output_dir = "./capture-output";

Mat getCamMatrix(const FileNode& camParams) {
    return (Mat_<float>(3, 3) <<
        (float)camParams["FocalLength"][0], 0, (float)camParams["PrincipalPoint"][0],
        0, (float)camParams["FocalLength"][1], (float)camParams["PrincipalPoint"][1],
        0, 0, 1
    );
}

ModelContainer getModelContainer(const FileNode& containerParam) {
    ModelContainer ans;
    containerParam["nCell"] >> ans.nCell;
    containerParam["cellSize"] >> ans.cellSize;
    return ans;
}

class DevData {
public:
    int viewerGamma, viewerTheta, viewerRadius;
    Vec3i containerPose;

    bool displayUI;

    DevData(bool _displayUI)
        : displayUI(_displayUI)
    {
        this->createTrackbars();
        this->readData();
    }

    void createTrackbars() {
        if (!this->displayUI) return;
        namedWindow("Devdatas", WINDOW_NORMAL);
        moveWindow("Devdatas", 800, 0);
        createRangeTrackbar("viewer gamma", "Devdatas", &viewerGamma, -180, 180);
        createRangeTrackbar("viewer theta", "Devdatas", &viewerTheta, 0, 180);
        createRangeTrackbar("viewer radius", "Devdatas", &viewerRadius, 5, 500);

        createRangeTrackbar("container x", "Devdatas", &containerPose[0], -500, 500);
        createRangeTrackbar("container y", "Devdatas", &containerPose[1], -500, 500);
        createRangeTrackbar("container z", "Devdatas", &containerPose[2], -500, 500);
    }

    void writeData() {
        FileStorage fs("devdata.yml", FileStorage::WRITE);
        fs 
            << "viewerPose" << "[:" << viewerGamma << viewerTheta << viewerRadius << "]"
            << "containerPose" << containerPose;
        cout << "Written data to devdata.yml" << endl;
        fs.release();
    }

    bool readData() {
        FileStorage fs("devdata.yml", FileStorage::READ);
        if (!fs.isOpened()) {
            cout << "Fail to read devdata.yml" << endl;
            return false;
        }
        fs["containerPose"] >> containerPose;
        viewerGamma = fs["viewerPose"][0];
        viewerTheta = fs["viewerPose"][1];
        viewerRadius = fs["viewerPose"][2];
        cout << "Read data from devdata.yml" << endl;
        fs.release();
        if (this->displayUI) {
          destroyWindow("Devdatas");
          this->createTrackbars();
        }
        return true;
    }
   
};

vector<PoseWrapper> readAllCameraPose(const string& filename) {/*{{{*/
    FileStorage fs(filename, FileStorage::READ);
    auto rot = fs["Rotations"];
    auto tran = fs["Translations"];
    int n = fs["Camera Count"];
    assert((int)rot.size() == n and (int)tran.size() == n);

    vector<PoseWrapper> ans(n);
    for (int i = 0; i < n; ++i) {
        Mat t;
        rot[i] >> ans[i].rotMat;
        tran[i] >> t;


        if (ans[i].rotMat.type() != CV_32FC1) {
            ans[i].rotMat.convertTo(ans[i].rotMat, CV_32FC1);
        }
        ans[i].rotMat = ans[i].rotMat.inv();
        
        t.convertTo(t, CV_32FC1);
        ans[i].transVec = Vec3f((float*)t.data);
        ans[i].transVec = Vec3f(0, 0, -500);
        //ans[i] = ans[i].inv();
        //for (int f = 0; f < 3; ++f) ans[i].transVec[f] *= 100;

    }
    return ans;
}/*}}}*/

int main(int, char**) {
    FileStorage fs("config.json", FileStorage::READ);
      
    bool displayui = true;
    if ((int)fs["viewer"]["frame-pose"] != 0) displayui = false;
    DevData dd(displayui);

    int imgHeight = (int)fs["image-height"];
    int imgWidth = (int)fs["image-width"];
    Scalar bgColor;
    fs["background-color"] >> bgColor;

    Mat camMat = getCamMatrix(fs["camera-params"]);

    CameraRayCalculator crc(camMat);
    ModelContainer container = getModelContainer(fs["model"]["container"]);
    container.setPose(lookAt(Vec3f(dd.containerPose), Vec3f(dd.containerPose) + Vec3f(0, 0, 1)));

    BruteForceModelBuilder *mb = new BruteForceModelBuilder(
        container, 10, crc,100 
    );
    Mat frame1 = imread("frame1.png");
    Mat frame2 = imread("frame2.png");

    PoseWrapper pose1 = lookAt(Vec3f(0, 30, 0), Vec3f(0, 15, 70));
    PoseWrapper pose2 = lookAt(Vec3f(90, 30, 35), Vec3f(41, 16.75, 77));

    Visualizer vis;
    vis.camMat = camMat;

    namedWindow("Visualize model", WINDOW_AUTOSIZE);

    mb->process(frame1, pose1);
    cout << "Processed frame1" << endl;
    mb->process(frame2, pose2);
    cout << "Processed frame2" << endl;

    int framepose = (int)fs["viewer"]["frame-pose"];

    auto render = [&]() -> void{
        container.setPose(lookAt(Vec3f(dd.containerPose), Vec3f(dd.containerPose) + Vec3f(0, 0, 1)));

        if (framepose == 0) {
          float toRad = atan(1) * 4 / 180.0;
          vis.viewerPose = lookAt(
              Vec3f(dd.containerPose) +
              coordinatesOnGlobe(
                  dd.viewerGamma * toRad,
                  dd.viewerTheta * toRad,
                  dd.viewerRadius
              ),
              Vec3f(dd.containerPose)
          );
        } else if (framepose == 1) vis.viewerPose = pose1;
        else vis.viewerPose = pose2;
        vis.result = Mat(imgHeight, imgWidth, CV_8UC3, bgColor);

        vis.drawModelContainer(container);
        vis.drawModel(mb);

        vis.drawCamera(Size2f(imgWidth, imgHeight), crc, pose1);
        vis.drawCamera(Size2f(imgWidth, imgHeight), crc, pose2);
        bool frameBlend;
        fs["viewer"]["frame-blend"] >> frameBlend;
        if (frameBlend) {
            if (framepose == 1)
                addWeighted(vis.result, .65, frame1, .35, 0.0, vis.result);
            if (framepose == 2)
                addWeighted(vis.result, .65, frame2, .35, 0.0, vis.result);

        }
    };

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
    clog << minCoor << ' ' << maxCoor << endl;

    Octree<Triangle3D> octree(minCoor, maxCoor - minCoor, static_cast<bool(*)(const Box&, const Triangle3D&)>(Geometry::inside), 8, 16);
    for (unsigned int* i = teapotMesh.polygons.ptr<unsigned int>(), f = 0; f < teapotMesh.polygons.cols; f+= 4) {
        assert(*(i++) == 3);
        octree.insert(Triangle3D(
            cloudPoints[*(i++)],
            cloudPoints[*(i++)],
            cloudPoints[*(i++)]
        ));
    }

    ColorRayLine theFckingRay(minCoor, maxCoor - minCoor, Color::cyan());
    displayWindow.showWidget("Teapot", viz::WMesh(teapotMesh));
    //displayWindow.showWidget("Octree", octree.toVizWidget());
    displayWindow.showWidget("Ray", theFckingRay.toVizWidget(10));

    viz::WWidgetMerger boxes;

    for (int i = 8; i--; ) {
      for (auto b: getAllOctreeBoxWithRay(*octree.getChild(i), theFckingRay)) {
        boxes.addWidget(b.toVizWidget(viz::Color::red()));
      }
    }

    displayWindow.showWidget("Boxes", boxes);

    while(!displayWindow.wasStopped())
    {
        displayWindow.spinOnce(16, true);
    }


    //Visualizer::cameraThickness = 50;
    //for (;;) {
        //render();
        //imshow("Visualize model", vis.result);
        //int k = waitKey(50);
        //if (k == 27 or k == 'q' or k == 'Q') {
            //break;
        //}
        //if (k == 'r' or k == 'R') {
            //dd.readData();
        //}
        //if (k == 'w' or k == 'W') {
            //dd.writeData();
        //}
        //if (k == 's' or k == 'S') {
            //string filename = "";
            //while (filename == "") {
                //cout << "Saving screenshot with file name: ";
                //cin >> filename;
            //}
            //filename = capture_output_dir + "/" + filename + ".png";
            //imwrite(filename, vis.result);
            //cout << "Saved " << filename << endl;
        //}
        //if ((k == 'p' or k == 'P') and framepose == 0) {
            //cout << "Start record video with rotate gamma" << endl;
            //string filename = capture_output_dir + "/recored-vid.mp4";
            //VideoWriter vw(
                //filename,
                //VideoWriter::fourcc('P','I','M','1'),
                //30,
                //Size(imgWidth, imgHeight)
            //);
            //for (int i = 180; i >= -180; --i) {
                //dd.viewerGamma = i;
                //render();
                //imshow("Visualize model", vis.result);
                //vw << vis.result;
                //waitKey(10);
            //}
            //dd.readData();
            //cout << "Record complete" << endl;
            //vw.release();
        //}
    //}

    delete mb;
    fs.release();
    return 0;
}
