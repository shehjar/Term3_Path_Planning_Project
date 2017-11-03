#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include<iostream>
#include<fstream>
#include<cmath>
#include<string>
#include<vector>
#include<map>
#include "car.h"
#include "helper.h"
#include "json.hpp"

using json = nlohmann::json;

class pathPlanner{
  public:
    std::vector<double> map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy;
    std::vector<car> vehicles;
    car ego;
    json previous_path_x, previous_path_y;

    // Member functions
    pathPlanner(){ego.v = 0;}
    ~pathPlanner(){}
    void generate_trajectory(vector<double>& next_x_vals, vector<double>& next_y_vals);
    void PopulatingMapWaypoints(string path);
};
#endif
