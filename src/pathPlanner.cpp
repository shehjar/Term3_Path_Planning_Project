#include "pathPlanner.h"
#include<fstream>
#include<sstream>
#include<string>
#include<vector>

void pathPlanner::PopulatingMapWaypoints(string folderpath){
  ifstream in_map_(folderpath.c_str(), ifstream::in);

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
  int lane = ego.lane;
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
  vector<double> next_wp0 = getXY(ego.s + 30,(2+4*lane), map_waypoints_s, 
      map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(ego.s + 60,(2+4*lane), map_waypoints_s, 
      map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(ego.s + 90,(2+4*lane), map_waypoints_s, 
      map_waypoints_x, map_waypoints_y);
          
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

