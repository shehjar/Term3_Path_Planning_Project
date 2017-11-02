#ifndef CAR_H
#define CAR_H

#include<iostream>
#include<cmath>
#include<vector>
#include<string>
#include<map>

//using namespace std;

class car{
  public:
  //Member variables
  double x, y, s, d, vx, vy, v, ref_v, ax, ay, a, yaw;
  int id, lane, 
      target_lane;    // Configured to change lanes 
  std::string state;

  struct prediction{
    double x, y, s, d, vx, vy, ax, ay;
    int lane; 
  };
  // Member functions
  car();
  ~car();
  void InitVariables(double x, double y, double yaw, double speed, double s, double d, double ref_vel);
  void InitVariables(std::vector<double> sensor_info);
  //void UpdateState(map<int,vector<prediction> > predictions);

  //double distFromCar(car anotherCar);
  //bool collideWithCar(car anotherCar);
};

#endif

