#pragma once
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <array>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;
using Eigen::ArrayXd;

class GNB
{
public:
	GNB();
	GNB(GNB& classifier);
	~GNB();

	vector<string> possible_labels = { "left","keep","right" };
	array<int, 4> features_selected = { { 1,1,1,1 } };		// 1 :  selected, 0: not selected
	ArrayXd left_means;
	ArrayXd left_sds;
	double left_prior;

	ArrayXd keep_means;
	ArrayXd keep_sds;
	double keep_prior;

	ArrayXd right_means;
	ArrayXd right_sds;
	double right_prior;

	void train(vector<vector<double> > data, vector<string>  labels);
	string predict(vector<double>samples);

private:
	vector<string> Load_Label(string filename);
	vector<vector<double>> Load_State(string filename, array<int,4> features);

};
