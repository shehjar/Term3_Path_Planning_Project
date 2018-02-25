#include"car.h"
#include"helper.h"

using namespace std;

car::car(){
  v = 0;
  classifier = nullptr;
}

car::car(const car& somecar){
	x = somecar.x;
	y = somecar.y;
	s = somecar.s;
	d = somecar.d;
	v = somecar.v;
	vx = somecar.vx;
	vy = somecar.vy;
	a = somecar.a;
	ax = somecar.ax;
	ay = somecar.ay;
	yaw = somecar.yaw;
	lane = somecar.lane;
	target_lane = somecar.target_lane;
	id = somecar.id;
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
  yaw = rad2deg(atan2(vy,vx));
  // Set Lane parameter
  lane = (int)d/4;
  target_lane = lane;
  id = sensor_info[0];
}

void car::InitVariables(double car_x, double car_y, double car_yaw,
    double car_s, double car_d, double ref_vel){
  x = car_x;
  y = car_y;
  yaw = car_yaw;
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

void car::UpdateSpeed(double val) {
	v += val;
}

void car::UpdateVariables(car* new_car) {
	vx = new_car->vx;
	vy = new_car->vy;
	v = sqrt(vx*vx + vy * vy);
	a = (new_car->v - v) / 0.02;
	yaw = new_car->yaw;
	ax = a * cos(deg2rad(yaw));
	ay = a * sin(deg2rad(yaw));
	ref_v = v; 
	x = new_car->x;
	y = new_car->y;
	s = new_car->s;
	d = new_car->d;
	lane = (int)d / 4;
	// Target lane shall be gotten by predict function
	target_lane = lane;
}

void car::UpdateVariables(double& car_x, double& car_y, double& car_yaw, double& car_s, double& car_d) {
	x = car_x;
	y = car_y;
	yaw = car_yaw;
	vx = v * cos(deg2rad(yaw));
	vy = v * sin(deg2rad(yaw));
	s = car_s;
	d = car_d;
	lane = (int)d / 4;
}

void car::PrintState() {
	cout << "State for car id: " << id <<" is given below"<< endl;
	cout << "x: " << x << " y: " << y << " s: " << s << " d: " << d << endl;
	cout << "v: " << v << " a: " << a << " yaw: " << yaw << endl;
	cout << "lane: " << lane << " target_lane: " << target_lane << endl;
}