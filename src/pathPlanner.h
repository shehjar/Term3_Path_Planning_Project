#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include<iostream>
#include<fstream>
#include<cmath>
#include<string>
#include<vector>
#include<map>
#include "car.h"
#include "spline.h"
#include "CoordinateTransform.h"
#include "json.hpp"

using json = nlohmann::json;
const double ref_vel = 49.5;

class pathPlanner{
  public:
    std::vector<double> map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy;
    std::vector<car> vehicles;
    car ego;
    json previous_path_x, previous_path_y;
	CoordinateTransform coord;
	enum LaneDirection {
		left,
		right,
		same
	};
	enum FSMStates {
		KL,
		PLC,
		LCR,
		LCL
	};
	FSMStates CurrentState;
    // Member functions
	pathPlanner() { CurrentState = KL; }
    ~pathPlanner(){}
    void generate_trajectory(vector<double>& next_x_vals, vector<double>& next_y_vals);
    void PopulatingMapWaypoints(string path);
	void UpdatingVehicles(json& sensor_fusion);
	void CleanUpVehicles();
	void InitEgo(json& j);
	void InitializeAndUpdate(json& j);
	void DetectingCollision();
	FSMStates TransitionFunction();
	double CostFunctions();
	void UpdateSpeed(bool too_close);
	void map2carCoord(vector<double>& ptsx, vector<double>& ptsy, vector<double> ref_state);
};
#endif
