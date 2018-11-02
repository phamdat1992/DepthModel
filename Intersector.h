#pragma once

#ifdef WIN32
#include "pch.h"
#endif

#ifdef LINUX
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"

#include <vector>
#include <cstdio>
#endif

cv::Vec3f CoarseEstimate(cv::Vec3f OCenter, cv::Vec4f TarPlane, cv::Vec3f Point);
std::vector<cv::Mat> CoarseEstimate(cv::Vec3f OCenter, cv::Vec4f TarPlane, std::vector<cv::Vec3f> Points);

cv::Mat FineEstimate(cv::Vec3f OCenter, cv::Vec4f TarPlane, cv::Vec4f PrjPlane);

cv::Mat Estimate(cv::Vec3f OCenter, cv::Vec4f TarPlane, cv::Vec4f PrjPlane);