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
/// ����һ������ʱ������
/// </summary>
/// <param name="start">��ʼʱ��</param>
/// <param name="end">����ʱ��</param>
/// <param name="step">����ʱ��Ƭ��ʱ�䲽��</param>
/// <returns>����ʱ������</returns>
vector<double> Sim::creat_time_space(double start, double end, double step) {

    vector<double> timespace;

    for (double t = start; t < end; t += step) {
        timespace.push_back(t);
    }

    return timespace;
}

/// <summary>
/// �������sim��ʼ��
/// </summary>
/// <param name="_config">����yaml�����ļ���Ϣ</param>
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
	
	//----�쳣���---
	error_scan();
	if (this->error_flag == 1)	return;

	this->timespace = creat_time_space(0, max_sim_size, dt);
    //------����txt�ļ�----------
    outputinit();
}

/// <summary>
/// ���������־�ļ�
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
		cout << "Error: MAX_SIM_SIZE����С�ڵ���0" << endl;
	}

	if (this->dt <= 0) {
		this->error_flag = true;
		cout << "Error: STEP_SIZE����С�ڵ���0" << endl;
	}
	else if (this->dt > this->max_sim_size) {
		this->error_flag = true;
		cout << "Error: STEP_SIZE���ܴ���MAX_SIM_SIZE" << endl;
	}

	if (this->coordinate != "GPS" && this->coordinate != "cartesian") {
		this->error_flag = true;
		cout << "COORDINATE��������" << endl;
	}


}


/// <summary>
/// ���������򣬿���Ŀ�굯��Ϊ�����ص���Ϊ
/// </summary>
/// <returns>�������н�������0</returns>
int missile_sim() {
	//---------���ݶ���------------------
	//YAML::Node config = YAML::LoadFile("config.yaml");
	YAML::Node config = YAML::LoadFile(PATH);


	Obj obj;
	Sim sim;
	vector<Hitter> hitters;

	bool hit_flag = false;

	//=============for config----------------
	double a = 0;


	//---------hitter���---------
	int num_interceptor = config["interceptor"]["NUM_INTERCEPTOR"].as<int>();
	if (num_interceptor <= 0) {
		cout << "NUM_INTERCEPTOR����С�ڵ���0" << endl;
		return 1;
	}


	//----------Init----------------------
	sim.init(config);
	obj.init(config, &sim);


	

	//��ʼ�����ص�
	for (int i = 0; i < num_interceptor; i++) {
		Hitter hitter;

		//���ó�ʼ״̬
		double rand_target[3];

		double init_pos[3];
		for (int i = 0; i <= 2; i++) {		//һ��λ��
			init_pos[i] = config["interceptor"]["INITIAL_POSITION"][i].as<double>();
			if (init_pos[i] > sim.max_sim_size || init_pos[i] < 0) {
				cout << "Error: INITIAL_POSITION��Ҫ�� 0 - MAX_SIM_SIZE ��" << endl;
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




