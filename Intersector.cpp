#include "pch.h"

#ifdef LINUX
#include "Intersector.h"
#endif

#define sqr(x) x * x

cv::Vec3f CoarseEstimate(cv::Vec3f OCenter, cv::Vec4f TarPlane, cv::Vec3f Point)
{
	float First = TarPlane[0];
	float Second = TarPlane[1];
	float Third = TarPlane[2];
	float Fourth = TarPlane[3];
	
	cv::Vec3f TarNorm = cv::Vec3f(First, Second, Third);
	cv::Vec3f LineDir = cv::Vec3f(Point - OCenter);

	float Temp = sqr(First) + sqr(Second) + sqr(Third);
	cv::Vec3f PointOnPlane = cv::Vec3f(cv::Vec3f((First * Fourth) / Temp, (Second * Fourth) / Temp, (Third * Fourth) / Temp));
	PointOnPlane *= -1;
	/*
	std::cout << LineDir[0] << ' ' << LineDir[1] << ' ' << LineDir[2] << '\n';
	std::cout << (PointOnPlane - OCenter).dot(TarNorm) << '\n';
	*/
	float Target = (PointOnPlane - OCenter).dot(TarNorm) / LineDir.dot(TarNorm);
	
	cv::Vec3f TarPoint = (Target * LineDir + OCenter);

	return TarPoint;
}

std::vector<cv::Vec3f> CoarseEstimate(cv::Vec3f OCenter, cv::Vec4f TarPlane, std::vector<cv::Vec3f> Points)
{
	std::vector<cv::Vec3f> TarPoints;
	for (int count = 0; count < Points.size(); count++)
	{
		cv::Vec3f TarPoint = CoarseEstimate(OCenter, TarPlane, Points[count]);
		TarPoints.push_back(TarPoint);
	}

	return TarPoints;
}

cv::Vec3f FineEstimate(cv::Vec3f OCenter, cv::Vec4f TarPlane, cv::Vec4f PrjPlane)
{
	cv::Vec3f TarVec3f;
	return TarVec3f;
}

cv::Vec3f Estimate(cv::Vec3f OCenter, cv::Vec4f TarPlane, cv::Vec4f PrjPlane)
{
	cv::Vec3f TarVec3f;
	return TarVec3f;
}