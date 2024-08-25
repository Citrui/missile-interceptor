#pragma once
#include <vector>
#include <yaml-cpp/yaml.h>
#include <string>

using namespace std;

class Sim {


public:
	vector<double> timespace;
	
	YAML::Node config;
	double dt;
	double time;
	double max_sim_size;
	
	bool hit_flag;
	bool error_flag;

	double loc[3];
	double max_loc[3];
	string coordinate;



	//-------Êä³öÎÄ¼þ-----
	string outfilepath;



	void init(YAML::Node _config);
	vector<double> creat_time_space(double start, double end, double step);         //np.arange
	void outputinit();
	void error_scan();
};

int missile_sim();