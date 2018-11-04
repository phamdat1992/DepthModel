// Intersection.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include "Intersector.h"
#include <iostream>

using namespace std;

float a, b, c, d, x, y, z, m, n, o = 0;

int main()
{
	ios_base::sync_with_stdio(0);
	cin >> a >> b >> c >> d >> x >> y >> z >> m >> n >> o;
	/*
	a = b = c = 1;
	d = 1;
	x = y = z = 0;
	m = n = o = 2;
	*/
	cv::Vec4f Plane = cv::Vec4f(a, b, c, d);
	cv::Vec3f OCenter = cv::Vec3f(x, y, z);
	cv::Vec3f TarPoint = cv::Vec3f(m, n, o);

	cv::Vec3f Result = CoarseEstimate(OCenter, Plane, TarPoint);
	cout << Result[0] << ' ' << Result[1] << ' ' << Result[2];

	return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
