#include <yaml-cpp/yaml.h>
#include <vector>
#include <fstream>
#include <iostream>
#include "Eigen/Dense"
#include <string>
#include "windows.h"
#include <gl/glew.h> 
#include <GLFW/glfw3.h> 
#include "utils.h"
#include <stdio.h>

//  user
#include "objection.h"
#include "simulation.h"
#include "hitter.h"

using namespace std;

#define PATH "E:\\work\\missile interceptor\\missile\\missile\\config.yaml"

/// <summary>
/// 创建一个仿真时间序列
/// </summary>
/// <param name="start">开始时间</param>
/// <param name="end">结束时间</param>
/// <param name="step">仿真时间片的时间步长</param>
/// <returns>返回时间序列</returns>
vector<double> Sim::creat_time_space(double start, double end, double step) {

    vector<double> timespace;

    for (double t = start; t < end; t += step) {
        timespace.push_back(t);
    }

    return timespace;
}

/// <summary>
/// 仿真对象sim初始化
/// </summary>
/// <param name="_config">导入yaml配置文件信息</param>
void Sim::init(YAML::Node _config) {
    config = _config;

    this->time = 0;
    this->dt = config["simulator"]["STEP_SIZE"].as<double>();
    this->max_sim_size = config["simulator"]["MAX_SIM_SIZE"].as<double>();

    for(int i=0;i<=2;i++)
        this->loc[i] = config["simulator"]["INITIAL_LOC"][i].as<double>();

    for (int i = 0; i <= 1; i++)
        this->max_loc[i] = this->max_sim_size * 0.02 + loc[i];
    this->max_loc[2] = this->max_sim_size * 1000 + loc[2];
    this->coordinate = config["simulator"]["COORDINATE"].as<string>();

    
    
    this->outfilepath = config["simulator"]["OUTPUT_TRAJECTION_PATH"].as<string>();

    this->hit_flag = false;
	this->error_flag = false;
	
	//----异常检测---
	error_scan();
	if (this->error_flag == 1)	return;

	this->timespace = creat_time_space(0, max_sim_size, dt);
    //------创建txt文件----------
    outputinit();
}

/// <summary>
/// 创建输出日志文件
/// </summary>
void Sim::outputinit() {
    std::ofstream outfile(outfilepath);
    if (!outfile) {
        cerr << "file error" << outfilepath << endl;
        
    }
    outfile.close();
}


void Sim::error_scan() {
	if (this->max_sim_size <= 0) {
		this->error_flag = true;
		cout << "Error: MAX_SIM_SIZE不能小于等于0" << endl;
	}

	if (this->dt <= 0) {
		this->error_flag = true;
		cout << "Error: STEP_SIZE不能小于等于0" << endl;
	}
	else if (this->dt > this->max_sim_size) {
		this->error_flag = true;
		cout << "Error: STEP_SIZE不能大于MAX_SIM_SIZE" << endl;
	}

	if (this->coordinate != "GPS" && this->coordinate != "cartesian") {
		this->error_flag = true;
		cout << "COORDINATE配置有误" << endl;
	}


}


/// <summary>
/// 仿真主程序，控制目标弹行为和拦截弹行为
/// </summary>
/// <returns>正常运行结束返回0</returns>
int missile_sim() {
	//---------数据定义------------------
	//YAML::Node config = YAML::LoadFile("config.yaml");
	YAML::Node config = YAML::LoadFile(PATH);


	Obj obj;
	Sim sim;
	vector<Hitter> hitters;

	bool hit_flag = false;

	//=============for config----------------
	double a = 0;


	//---------hitter相关---------
	int num_interceptor = config["interceptor"]["NUM_INTERCEPTOR"].as<int>();
	if (num_interceptor <= 0) {
		cout << "NUM_INTERCEPTOR不能小于等于0" << endl;
		return 1;
	}


	//----------Init----------------------
	sim.init(config);
	obj.init(config, &sim);


	

	//初始化拦截弹
	for (int i = 0; i < num_interceptor; i++) {
		Hitter hitter;

		//配置初始状态
		double rand_target[3];

		double init_pos[3];
		for (int i = 0; i <= 2; i++) {		//一致位置
			init_pos[i] = config["interceptor"]["INITIAL_POSITION"][i].as<double>();
			if (init_pos[i] > sim.max_sim_size || init_pos[i] < 0) {
				cout << "Error: INITIAL_POSITION需要在 0 - MAX_SIM_SIZE 内" << endl;
				return 1;
			}
		}

		for (int i = 0; i <= 2; i++) {
			rand_target[i] = uniform(0, sim.max_sim_size);
		}

		hitter.init(config, rand_target, init_pos, &sim);
		if (sim.error_flag == true)	return 1;
		hitters.push_back(hitter);
	}

	if (sim.error_flag == true)	return 1;
	//----------move-----------------------
	for (; sim.time < sim.max_sim_size; sim.time += sim.dt) {

		for (auto& hitter : hitters) {
			hitter.pos_radar_reading(obj);
			obj.move_obj();
			hitter.estimate_new_position();
			hitter.interseptor_tracking();

			if (sim.coordinate == "GPS")	hitter.change_gps(sim);
			hitter.save_txt();


			//   for test  print estimate
			/*for (int i = 0; i <= 2; i++) {
				cout << hitter.estimate_obj_next_pos[i] << "\t";
			}
			cout << endl;*/
		}
		if (sim.coordinate == "GPS")	obj.change_gps(sim);
		obj.save_txt();
		for (Hitter hitter : hitters) {
			if (hitter.is_hit(&obj) == true) {
				sim.hit_flag = true;
				break;
			}
		}
		if (sim.hit_flag == true) {
			printf("Hit the target!\n");
			break;
		}
	}
	if (sim.hit_flag == false)
		printf("The missiles missed the target!\n");

	//  -----for test output hinst
   /*for (vector<double> pos : hitters[0].obj_pos_hist){
	   for (int i = 0; i <= 2; i++)
		   cout << pos[i] <<"\t";
   cout << endl;
   }*/
	return 0;
}




