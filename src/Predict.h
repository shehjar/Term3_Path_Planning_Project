#include<iostream>
#include<vector>
#include<math.h>

class Predict{
  Predict();
  ~Predict();
  // These vaöues are taken from the prediction module for Naive Bayes
  vector<double> P_y = {0.285333, 0.421333, 0.293333};
  vector<vector<double>> mean = {{19.7141, 5.05181, 9.91413, -0.967087},{20.3242, 3.68024, 9.99854, 0.00581204},{19.4772, 2.93405, 9.94717, 0.954022}};
  vector<vector<double>> stddev = {{12.3187, 2.36564, 0.992561, 0.664837},{11.4544, 3.40922, 1.07032, 0.168393},{12.113, 2.31755,0.954257, 0.64832}};
  vector<string> possible_labels = {"left","keep","right"};
  double prob_est (vector<double> x, int y);
  string predict(vector<double> obs);
};
