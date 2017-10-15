#include<iostream>
#include<math>
#include<vector>

class car{
  //Member variables
  double x, y, s, d, sdot, ddot, v, yaw, lane;
  string state;
  // Member functions
  car();
  ~car();
  void UpdateVariables(vector<double> sensor_info);
  void UpdateState(string Prediction);
  double distFromCar(car anotherCar);
  bool collideWithCar(car anotherCar);
  Predict();

};

