#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include<iostream>
#include<cmath>
#include<string>
#include<vector>
#include<map>
#include "car.h"
#include "helper.h"

class pathPlanner{
  public:
    vector<double> map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_d;
    vector<car> vehicles;
    car ego;

    // Member functions
    void generate_trajectory(vector<double>& next_x_vals, vector<double>& next_y_vals);
    void populateMapWaypoints(string path);
};
#endif
