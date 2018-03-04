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
#include "GNB.h"

using json = nlohmann::json;
const double REF_VEL = 49.5;
const double W_LC = 10;
const double W_DIST = 10;
const double W_VEL = 10;
const int TOTAL_LANES = 3;
const double MAX_OBS_DIST = 80;
class pathPlanner{
  public:
    std::vector<car> vehicles;
    car ego;
    json previous_path_x, previous_path_y;
	CoordinateTransform coord;
	GNB classifier;
	enum LaneDirection {
		left,
		right,
		same
	};
	enum FSMStates {
		KL,
		LCR,
		LCL
	};
	FSMStates CurrentState;
    // Member functions
	pathPlanner() { CurrentState = KL; classifier = GNB(); }
    ~pathPlanner(){}
    void generate_trajectory(vector<double>& next_x_vals, vector<double>& next_y_vals);
    void PopulatingMapWaypoints(string path);
	void UpdatingVehicles(json& sensor_fusion);
	void CleanUpVehicles();
	void InitEgo(json& j);
	void InitializeAndUpdate(json& j);
	void DetectingCollision();
	void TransitionFunction();
	vector<FSMStates> SuccessorStates();
	double TotalCost(int& intended_lane);
	void ExecuteState();
	double GetLaneSpeed(int& lane);
	void UpdateSpeed(bool too_close);
	void map2carCoord(vector<double>& ptsx, vector<double>& ptsy, vector<double> ref_state);
};
#endif
