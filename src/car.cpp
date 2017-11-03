#include"car.h"
#include"helper.h"
using namespace std;

car::car(){
  state = "ps";
}

car::~car(){
}

void car::InitVariables(vector<double> sensor_info){
  vx = sensor_info[3];
  vy = sensor_info[4];
  v = sqrt(vx*vx + vy*vy);
  ref_v = v;
  a = 0, ax = 0, ay = 0;
  s = sensor_info[5];
  d = sensor_info[6];
  x = sensor_info[1];
  y = sensor_info[2];
  yaw = atan2(vy,vx);
  // TODO:Update Lane parameter
  lane = (int)d/4;
  target_lane = lane;
  id = sensor_info[0];
}

void car::InitVariables(double car_x, double car_y, double car_yaw, double car_speed,
    double car_s, double car_d, double ref_vel){
  x = car_x;
  y = car_y;
  yaw = car_yaw;
  //v = car_speed;
  vx = v*cos(deg2rad(yaw));
  vy = v*sin(deg2rad(yaw));
  ref_v = ref_vel;
  a = 0, ax = 0, ay = 0;
  s = car_s;
  d = car_d;
  lane = (int)d/4;
  target_lane = lane;
  id = -1;
}
