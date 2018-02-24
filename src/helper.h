#ifndef HELPER_H
#define HELPER_H

#include<iostream> 
#include<cmath> 

using namespace std;
//const double M_PI = std::atan(1.0) * 4;
// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }


inline double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}


#endif
