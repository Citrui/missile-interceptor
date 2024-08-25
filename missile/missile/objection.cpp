#include "objection.h"
#include <iostream>
#include <string>
#include "utils.h"
#include "simulation.h"
#include <fstream>

using namespace std;

/// <summary>
/// Ŀ�굯��ʼ��
/// </summary>
/// <param name="_config">����yaml�ļ�������Ϣ</param>
/// <param name="_sim">����������sim��Ϣ</param>
void Obj::init(YAML::Node _config, Sim *_sim) {
	
	name = "target";
	config = _config;
	sim = _sim;

	this->create_target();

	outfilepath = config["simulator"]["OUTPUT_TRAJECTION_PATH"].as<string>();

}

/// <summary>
/// �������һ��Ŀ�굯�Ĺ켣
/// </summary>
void Obj::create_target() {
	bool is_valid_trajectory = false;

	double max_sim_size = config["simulator"]["MAX_SIM_SIZE"].as<double>();
	double t_step = config["simulator"]["STEP_SIZE"].as<double>();
	double max_speed = config["objection"]["MAX_RANDOM_SPEED"].as<double>();
	
	double s_z, v_z, a_z;
	double v_x, s_x;
	double v_y, s_y;

	int rand_direction_flag;

	while (!is_valid_trajectory) {
		s_z = uniform(0, max_sim_size);
		v_z = uniform(-max_speed, max_speed);
		a_z = uniform(-5, 0);

		v_x = random_choice(-1, 1) * uniform(0.8, 0.9);
		s_x = random_choice(0, max_sim_size);    //������
		
		v_y = uniform(-max_speed, max_speed);
		s_y = uniform(0.5, max_sim_size);

		bool break_flag = false;    //no  ����û���жϣ������ж� =0 
		for (double t : sim->timespace) {
			double xt, yt, zt;
			xt = v_x * t + s_x;
			yt = v_y * t + s_y;
			zt = s_z + v_z * t + a_z * t * t;

			if (yt > max_sim_size || yt < 0) {
				break_flag = true;
				break;
			}
			if (zt > max_sim_size || zt < 0) {
				break_flag = true;
				break;
			}

			if (xt > max_sim_size || xt < 0) {
				break_flag = true;
				break;
			}
		}

		if (break_flag == false) {
			is_valid_trajectory = true;
			break;
		}
	}

	z_para[0] = s_z;
	z_para[1] = v_z;
	z_para[2] = a_z;

	rand_direction_flag = random_choice(0, 1);
	if (rand_direction_flag == 0) {
		x_para[0] = v_y;
		x_para[1] = s_y;

		y_para[0] = v_x;
		y_para[1] = s_x;
	}
	else if (rand_direction_flag == 1) {
		x_para[0] = v_x;
		x_para[1] = s_x;

		y_para[0] = v_y;
		y_para[1] = s_y;
	}
}

/// <summary>
/// ����켣������Ϣ��������
/// </summary>
void Obj::output_para() {

	string filepath = config["simulator"]["OUTPUT_PARA_PATH"].as<string>();


	ofstream testfile(filepath);

	if (testfile.is_open()) {
		for (double z : z_para) {
			testfile << z << endl;
		}
		for (double y : y_para) {
			testfile << y << endl;
		}
		for (double x : x_para) {
			testfile << x << endl;
		}
		testfile.close();
	}
	else { cout << "file_error" << endl; };
}

/// <summary>
/// Ŀ�굯�ƶ���Ϊ������Ŀ�굯��λ������
/// </summary>
void Obj::move_obj() {
	double dt = sim->time;

	pos[0] = x_para[0] * dt + x_para[1];
	pos[1] = y_para[0] * dt + y_para[1];
	pos[2] = z_para[0] + z_para[1] * dt + z_para[2] * dt * dt;
}

/// <summary>
/// ����Ŀ�굯��λ����Ϣ�������־��
/// </summary>
void Obj::save_txt() {

	ofstream outfile(outfilepath, ios::app);

	outfile << sim->time << "\t" << this->name << "\t";
	for (int i = 0; i <= 2; i++) {
		if(this->sim->coordinate == "cartesian")
			outfile << this->pos[i] << "\t";
		else if(this->sim->coordinate == "GPS")
			outfile << this->gps_pos[i] << "\t";
	}
	outfile << endl;

	outfile.close();
}

/// <summary>
/// ����ά����ת��ΪСƽ��GPS����
/// </summary>
/// <param name="sim">����������sim��Ϣ</param>
void Obj::change_gps(Sim sim) {
	for (int i = 0; i <= 1; i++)
		gps_pos[i] = pos[i] * 0.02 + sim.loc[i];
	gps_pos[2] = pos[2] * 1000 + sim.loc[2];
}
