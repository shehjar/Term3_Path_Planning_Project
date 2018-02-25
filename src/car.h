#ifndef CAR_H
#define CAR_H

#include<iostream>
#include<cmath>
#include<vector>
#include<string>
#include<map>
#include"CoordinateTransform.h"
#include"GNB.h"
//using namespace std;

class car{
	public:
	//Member variables
	double x, y, s, d, vx, vy, v, ref_v, ax, ay, a, yaw;
	int id, lane,
     target_lane;    // Configured to change lanes 

	struct prediction{
		double s, d, sdot, ddot;
		int lane; 
	};
	GNB* classifier;
	CoordinateTransform* coord;
	// Member functions
	car();
	~car();
	car(const car& somecar);
	void InitVariables(double x, double y, double yaw, double s, double d, double ref_vel);
	void InitVariables(std::vector<double> sensor_info);
	void UpdateVariables(car* new_car);
	void UpdateVariables(double& car_x, double& car_y, double& car_yaw, double& car_s, double& car_d);
	void UpdateSpeed(double val);
	void SetClassifier(GNB* predictor) { 
		if (classifier == nullptr)
			classifier = predictor; }
	void SetCoordTransform(CoordinateTransform* axis) { 
		if(coord==nullptr) 
			coord = axis; }
	double distFromCar(car anotherCar) {
		return sqrt((anotherCar.x - x)*(anotherCar.x - x) +
			(anotherCar.y - y)*(anotherCar.y - y));
	}
	bool collideWithCar(car anotherCar);
	void PrintState();
};

#endif

