#pragma once
#include<iostream> 
#include<vector> 
#include<cmath>
#include "helper.h"

using namespace std;

class CoordinateTransform
{
public:
	CoordinateTransform();
	CoordinateTransform(const CoordinateTransform& coord);
	~CoordinateTransform();

	// member Functions
	int ClosestWaypoint(double x, double y);
	int NextWaypoint(double x, double y, double theta);
	vector<double> getFrenet(double x, double y, double theta);
	vector<double> getXY(double s, double d);
	vector<double> getFrenetVelocities(double x, double y, double vx, double vy);
	void SetMaps(vector<double> maps_x, vector<double> maps_y, vector<double> maps_s, vector<double> maps_dx, vector<double> maps_dy);
	void PrintData();
	// member Variables
	vector<double> maps_x;
	vector<double> maps_y;
	vector<double> maps_s;
	vector<double> maps_d_x;
	vector<double> maps_d_y;
};

