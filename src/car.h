#ifndef CAR_H
#define CAR_H

#include<iostream>
#include<cmath>
#include<vector>
#include<string>
#include<map>
#include"CoordinateTransform.h"
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

	CoordinateTransform coord;
	// Member functions
	car();
	~car();
	car(const car& somecar);
	void InitVariables(double x, double y, double yaw, double s, double d, double ref_vel);
	void InitVariables(std::vector<double> sensor_info);
	void UpdateVariables(car* new_car);
	void UpdateVariables(double& car_x, double& car_y, double& car_yaw, double& car_s, double& car_d);
	//void UpdateState(map<int,vector<prediction> > predictions);
	void UpdateSpeed(double val);
	double distFromCar(car anotherCar);
	bool collideWithCar(car anotherCar);
	void PrintState();
};

#endif

