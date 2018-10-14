#include <opencv2/opencv.hpp>
#include <algorithm>
#include <vector>
#include "ModelBuilder.hpp"
#include "PoseWrapper.hpp"
#include "Visualizer.hpp"

using namespace cv;
using namespace std;

namespace DepthModel {
Scalar Visualizer::containerColor(255, 0, 0);
Scalar Visualizer::cameraColor(255, 255, 255);
float Visualizer::cameraThickness = 10;
float Visualizer::pointRadius = 1;

void Visualizer::drawModelContainer(const ModelContainer& container) {
    PoseWrapper invContainerPose = container.getPose().inv();
    Point verties[8];
    for (int i = 0; i < 8; ++i) {
        Vec3f ver;
        for (int f = 0; f < 3; ++f) {
            ver[f] = container.nCell[f] * container.cellSize[f] / 2;
            if ((i >> f) & 1)
                ver[f] = -ver[f];
        }
        ver = invContainerPose.apply(ver);
        ver = viewerPose.apply(ver);
        Mat vermat = camMat * Mat(ver);
        float* dat = (float*) vermat.data;

        verties[i] = Point(cvRound(dat[0] / dat[2]), cvRound(dat[1] / dat[2]));
    }

    for (int i = 0; i < 8; ++i) {
        for (int f = 0; (1 << f) <= i; ++f) {
            if ((~i >> f) & 1) continue;
            line(result, verties[i], verties[i ^ (1 << f)], containerColor);
        }
    }
}

void Visualizer::drawCamera(
    Size2f camSize,
    CameraRayCalculator& rayCaltor,
    const PoseWrapper& camPose
) {
    PoseWrapper invCamPose = camPose.inv();
    Mat centerMat = camMat * Mat(viewerPose.apply(invCamPose.apply(Vec3f())));
    Point center(
        cvRound(centerMat.at<float>(0) / centerMat.at<float>(2)),
        cvRound(centerMat.at<float>(1) / centerMat.at<float>(2))
    );
    Point points[4];
    for (int i = 0; i < 4; ++i) {
        Vec3f pos = rayCaltor.getRay(
          i & 1 ? 0 : camSize.width,
          i & 2 ? 0 : camSize.height
        ) * cameraThickness;
        Mat pointMat = camMat * Mat(viewerPose.apply(invCamPose.apply(pos)));
        float* pointDat = (float*)pointMat.data;
        points[i].x = cvRound(pointDat[0] / pointDat[2]);
        points[i].y = cvRound(pointDat[1] / pointDat[2]);
        line(result, center, points[i], cameraColor);
    }
    line(result, points[0], points[1], cameraColor);
    line(result, points[0], points[2], cameraColor);
    line(result, points[3], points[1], cameraColor);
    line(result, points[3], points[2], cameraColor);
}

void Visualizer::drawPoints(vector<ModelPoint> points) {
    for (ModelPoint& p: points) {
        p.position = this->viewerPose.apply(p.position);
    }
    sort(points.begin(), points.end(),
        [](const ModelPoint& u, const ModelPoint& v) {
            return u.position[2] < v.position[2];
        });
    for (ModelPoint& p: points) {
        Mat pointMat = this->camMat * Mat(p.position);
        float* pointDat = (float*)pointMat.data;
        Point u(pointDat[0] / pointDat[2], pointDat[1] / pointDat[2]);
        circle(result, u, pointRadius, p.getColor(), -1);
    }
}

void Visualizer::drawModel(
    ModelBuilderBase* model
) {
    drawPoints(model->getAllPoints());
    drawModelContainer(model->getContainer());
}

} // DepthModel
