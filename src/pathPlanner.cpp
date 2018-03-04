#include "pathPlanner.h"
#include<fstream>
#include<sstream>
#include<string>
#include<vector>
#include<algorithm>

void pathPlanner::PopulatingMapWaypoints(string folderpath){
  ifstream in_map_(folderpath.c_str(), ifstream::in);
  std::vector<double> map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy;

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  coord.SetMaps(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);
}

void pathPlanner::generate_trajectory(vector<double>& next_x_vals, 
    vector<double>& next_y_vals){
  // Populating a vector of points for spline 
  vector<double> ptsx;
  vector<double> ptsy; 
  int prev_size = previous_path_x.size();

  // Reference x,y, yaw states
  double ref_x = ego.x;
  double ref_y = ego.y;
  double ref_yaw = deg2rad(ego.yaw);
  int lane = ego.target_lane;
  double ref_vel = ego.v;

  // If previous size is almost empty
  if (prev_size < 2){
    // use two points that make the path tangent to the car
    double prev_car_x = ref_x - cos(ego.yaw);
    double prev_car_y = ref_y - sin(ego.yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(ref_x);
    ptsy.push_back(prev_car_y);
    ptsy.push_back(ref_y);
  }
  else{
    // Redefine reference state as previous path end point
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];

    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
    // use the two points that make the path tangent to the previous path's  end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // In frenet add evenly 30m spaced points ahead of the starting reference
  vector<double> next_wp0 = coord.getXY(ego.s + 30,(2+4*lane));
  vector<double> next_wp1 = coord.getXY(ego.s + 60,(2+4*lane));
  vector<double> next_wp2 = coord.getXY(ego.s + 90,(2+4*lane));
          
  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  // Shifting the coordinates to the car coordinates by using the reference
  // configuration defined above
  vector<double> ref_state;
  ref_state.push_back(ref_x);
  ref_state.push_back(ref_y); 
  ref_state.push_back(ref_yaw);

  map2carCoord(ptsx, ptsy, ref_state);
  // Create a spline
  tk::spline s;
  s.set_points(ptsx, ptsy);

  //Populating the output vector of points
  // Start with all of the previous path points from last time
  for(int i=0; i < prev_size; i++){
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }
          
  // Calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);
  double N = target_dist/(0.02*ref_vel/2.24);
  double x_add_on = 0;
  
  // Fill up the rest of our path planner after filling it with previous points, here we will always output 30 points
  for(int i=1; i<= 50 - prev_size; i++){
    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);

    x_add_on = x_point;
    double x_ref = x_point;
    double y_ref = y_point;

    // Rotate back to normal in Global coordinates
    x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
    y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);
    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
}

void pathPlanner::map2carCoord(vector<double>& ptsx, vector<double>& ptsy, vector<double> ref_state) {
	// shifting the coordinates to the Car coordinates
	double ref_x = ref_state[0];
	double ref_y = ref_state[1];
	double ref_yaw = ref_state[2];

	for (size_t i = 0; i < ptsx.size(); i++) {
		// shift car reference angle to 0 degrees
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;

		ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
		ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
	}
}
void pathPlanner::UpdateSpeed(bool too_close) {
	if (too_close)
		ego.UpdateSpeed(-0.4);
	else if (ego.v < ego.ref_v)
		ego.UpdateSpeed(0.4);
}

void pathPlanner::UpdatingVehicles(json& sensor_fusion) {
	for (auto& i : sensor_fusion) {
		// check if the car is within reach of Ego- around
		// 80 m radius
		//if (fabs(car_s - ego.s) < 80) {
			car new_car;
			new_car.InitVariables(i);
			new_car.SetClassifier(&classifier);
			new_car.SetCoordTransform(&coord);
			//check if the vehicle list is empty
			if (vehicles.empty())
				vehicles.emplace_back(new_car);
			else {
				// Check if the car is already in the list
				int id = new_car.id;
				auto predicate = [&id](const car& obj) {return obj.id == id; };
				auto foundCar = find_if(vehicles.begin(), vehicles.end(), predicate);
				if (foundCar != vehicles.end())
					foundCar->UpdateVariables(&new_car);
				else
					vehicles.emplace_back(new_car);
			}
		//}
	}
}

void pathPlanner::CleanUpVehicles() {
	double s = ego.s;
	auto predicate = [&s](const car& obj) {return fabs(obj.s - s) > 80; };
	vehicles.erase(std::remove_if(vehicles.begin(), vehicles.end(), predicate), vehicles.end());
}

void pathPlanner::InitEgo(json& j) {
	// j[1] is the data JSON object
	// Main car's localization Data
	double car_x = j[1]["x"];
	double car_y = j[1]["y"];
	double car_s = j[1]["s"];
	double car_d = j[1]["d"];
	double car_yaw = j[1]["yaw"];
	//double car_speed = j[1]["speed"];
	// Previous path data given to the Planner
	auto prev_path_x = j[1]["previous_path_x"];
	auto prev_path_y = j[1]["previous_path_y"];
	// Previous path's end s and d values 
	double end_path_s = j[1]["end_path_s"];
	double end_path_d = j[1]["end_path_d"];
	// Minor adjustments and initializations
	previous_path_x = prev_path_x;
	previous_path_y = prev_path_y;
	int prev_size = previous_path_x.size();
	if (prev_size > 0) {
		car_s = end_path_s;
		ego.UpdateVariables(car_x, car_y, car_yaw, car_s, car_d);
	}
	else {
		// Initializing Ego for the first time
		ego.InitVariables(car_x, car_y, car_yaw, car_s, car_d, REF_VEL);
		ego.SetClassifier(&classifier);
		ego.SetCoordTransform(&coord);
	}
	//ego.PrintState();
}

void pathPlanner::InitializeAndUpdate(json& j) {
	// Localizing main car
	InitEgo(j);
	// Getting Vehicles data from sensor fusion
	auto sensor_fusion = j[1]["sensor_fusion"];
	UpdatingVehicles(sensor_fusion);
}

void pathPlanner::DetectingCollision() {
	bool too_close = false;
	double prev_size = previous_path_x.size();
	for (auto& somecar : vehicles) {
		if(somecar.lane == ego.lane){
			double car_speed = somecar.v;
			double car_s = somecar.s;
			car_s += (double)prev_size*0.02*car_speed;
			if ((car_s > ego.s) && (car_s - ego.s) < 50) {
				too_close = true;
				break;
			}
		}
	}
	// based on collision, update speed
	UpdateSpeed(too_close);
}

double pathPlanner::TotalCost( int& intended_lane) {
	double cost = 0;
	// Calculating the lane change cost
	cost += W_LC * (1 - fabs(intended_lane - ego.lane) / (TOTAL_LANES - 1));
	// Calculating the distance cost from the nearest car 
	// Get nearest car in the front
	double mindist = MAX_OBS_DIST;		// some max value
	double min_vel = REF_VEL;
	for (auto& somecar : vehicles) {
		if ((somecar.lane == intended_lane)&&(somecar.s > ego.s)) {
			// This car is in front of us
			double dist = somecar.s - ego.s;
			if (dist < mindist) {
				mindist = dist;
				//if (somecar.v < REF_VEL)
				min_vel = somecar.v;
			}
		}
	}
	cost += W_DIST * (1 - (MAX_OBS_DIST - mindist) / MAX_OBS_DIST);
	// Calculating the velocity cost from the nearest car in the lane
	cost += W_VEL * (1 - (REF_VEL - min_vel) / REF_VEL);
	return cost;
}

vector<pathPlanner::FSMStates> pathPlanner::SuccessorStates() {
	vector<FSMStates> resulting_states;
	switch (CurrentState)
	{
	case pathPlanner::KL:
		if (ego.lane != 0 && ego.lane != 2) {
			resulting_states.push_back(KL);
			resulting_states.push_back(LCR);
			resulting_states.push_back(LCL);
		}
		else if (ego.lane == 0) {
			resulting_states.push_back(KL);
			resulting_states.push_back(LCR);
		}
		else if (ego.lane == 2) {
			resulting_states.push_back(KL);
			resulting_states.push_back(LCL);
		}
		break;
	case pathPlanner::LCR:
		resulting_states.push_back(KL);
		if (ego.lane != 2)
			resulting_states.push_back(LCR);
		break;
	case pathPlanner::LCL:
		resulting_states.push_back(KL);
		if (ego.lane != 0)
			resulting_states.push_back(LCL);
		break;
	default:
		resulting_states.push_back(KL);
		break;
	}
	return resulting_states;
}

void pathPlanner::TransitionFunction() {
	vector<FSMStates> resulting_states = SuccessorStates();
	vector<double> costs;
	// browsing through all the states
	for (size_t i = 0; i < resulting_states.size(); ++i) {
		FSMStates State = resulting_states[i];
		double cost = 0;
		int intended_lane;
		switch (State)
		{
		case pathPlanner::KL:
			intended_lane = ego.lane;
			break;
		case pathPlanner::LCR:
			intended_lane = ego.lane + 1;
			break;
		case pathPlanner::LCL:
			intended_lane = ego.lane - 1;
			break;
		default:
			intended_lane = ego.lane;
			break;
		}
		cost += TotalCost(intended_lane);
		costs.push_back(cost);
	}
	//Get best state with maximum costs and assign to CurrentState
	int argmax = std::distance(costs.begin(), std::max_element(costs.begin(), costs.end()));
	CurrentState = resulting_states[argmax];
}

double pathPlanner::GetLaneSpeed(int& lane) {
	double min_vel = REF_VEL;
	for (auto& somecar : vehicles) {
		if (somecar.lane == lane && somecar.s >= ego.s && fabs(somecar.s - ego.s) < MAX_OBS_DIST)
			if (somecar.v < min_vel)
				min_vel = somecar.v;
	}
	return min_vel;
}

void pathPlanner::ExecuteState() {
	switch (CurrentState)
	{
	case pathPlanner::KL:
		// assigning reference velocity and lane
		ego.target_lane = ego.lane;
		ego.ref_v = GetLaneSpeed(ego.target_lane);
		break;
	case pathPlanner::LCR:
		ego.target_lane = ego.lane + 1;
		ego.ref_v = GetLaneSpeed(ego.target_lane);
		break;
	case pathPlanner::LCL:
		ego.target_lane = ego.lane - 1;
		ego.ref_v = GetLaneSpeed(ego.target_lane);
		break;
	default:
		break;
	}
	// Check for Collision and lower speed if this exists
	DetectingCollision();
}