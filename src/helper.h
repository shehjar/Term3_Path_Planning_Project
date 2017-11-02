#ifndef HELPER_H
#define HELPER_H

#include<iostream> 
#include<vector> 
#include<cmath> 
#include "spline.h"
#include "car.h"

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(size_t i = 0; i < maps_x.size(); i++){
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen){
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);
	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];
	double heading = atan2( (map_y-y),(map_x-x) );
	double angle = abs(theta-heading);

	if(angle > pi()/4){
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0){
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point
	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef){
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++){
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}
	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;
	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) )){
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();
	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);
	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};
}

void map2carCoord(vector<double>& ptsx, vector<double>& ptsy, vector<double> ref_state){
// shifting the coordinates to the Car coordinates
double ref_x = ref_state[0];
double ref_y = ref_state[1];
double ref_yaw = ref_state[2];

for(size_t i=0; i < ptsx.size(); i++){
  // shift car reference angle to 0 degrees
  double shift_x = ptsx[i]-ref_x;
  double shift_y = ptsy[i]-ref_y;

  ptsx[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
  ptsy[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
  }
}

void generate_trajectory(vector<double>& next_x_vals, vector<double>& next_y_vals,
    car ego, vector<double> map_waypoints_x, vector<double> map_waypoints_y, 
    vector<double> map_waypoints_s, vector<double> previous_path_x, vector<double> previous_path_y){
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
  vector<double> ref_state ={ref_x, ref_y, ref_yaw};
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

#endif
