#pragma once

#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include "simulation.h"

class Obj {
private:

	std::string name;
	YAML::Node config;
	Sim *sim;

	double z_para[3];
	double y_para[2];
	double x_para[2];


	//std::vector<double *> gps_loc_log;

	//---txt--
	string outfilepath;



public:
	double pos[3];
	double gps_pos[3];



	void init(YAML::Node _config, Sim *sim);
	void create_target();
	void output_para();      //for test
	void move_obj();
	void save_txt();
	void change_gps(Sim sim);
};
