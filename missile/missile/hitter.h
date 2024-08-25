#pragma once

#include <string>
#include <yaml-cpp/yaml.h>
#include "objection.h"
#include "simulation.h"
#include <Eigen/Dense>

using namespace std;


class Hitter {
private:
	string name;
	YAML::Node config;
	Sim* sim;

	//------坐标相关-------
	double pos[3];
	double last_pos[3];
	double present_pos[3];
	double gps_pos[3];

	double init_target[3];
	vector<vector<double>> obj_pos_hist;

	double estimate_obj_next_pos[3];

	//-------控制行为---------
	double searching_time;
	string delay_time_cfg;
	double delay_time;
	double delay_random[2];
	string tracking_method;
	double guidence_p;
	double guidence_cmp;
	double speed_limit;
	double angle_limit;
	double tolerance;
	double ac_error[3] = {0};

	//----txt-----------------
	string outfilepath;
	


public:
	

	void init(YAML::Node _config, double _init_target[3], double _init_pos[3],Sim *_sim);
	void pos_radar_reading(Obj obj);
	void estimate_new_position();
	void interseptor_tracking();
	void save_txt();
	bool is_hit(Obj *_obj);
	double vector_angle(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);
	Eigen::Vector3d adjust_p3_to_angle_fixed_distance(double a[3], double b[3], double c[3], double target_angle_degrees);
	void change_gps(Sim sim);
	void error_scan();
};



