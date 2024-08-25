#include "hitter.h"
#include <yaml-cpp/yaml.h>
#include "objection.h"
#include <vector>
#include <string>
#include "utils.h"
#include <cmath>
#include <fstream>
#include <iostream>


using namespace std;

/// <summary>
/// ��ʼ�����ص�����
/// </summary>
/// <param name="_config">����yaml�����ļ�</param>
/// <param name="_init_target">���ص�����ʱ��ʼ����λ��</param>
/// <param name="_init_pos">���ص�����ĳ�ʼ����</param>
/// <param name="_sim">����������sim����</param>
void Hitter::init(YAML::Node _config, double _init_target[3], double _init_pos[3],Sim *_sim) {
	this->name = "hitter";
	this->config = _config;
	this->sim = _sim;

	//--------����----------
	for (int i = 0; i <= 2; i++) {
		this->pos[i] = _init_pos[i];
		this->last_pos[i] = _init_pos[i];
		this->present_pos[i] = _init_pos[i];
		this->init_target[i] = _init_target[i];
		this->estimate_obj_next_pos[i] = _init_pos[i];
	}

	//---------����---------------
	this->searching_time = config["interceptor"]["SEARCHING_TIME"].as<double>();
	this->delay_time_cfg = config["interceptor"]["DELAY_TIME"].as<string>();
	this->delay_random[0] = config["interceptor"]["DELAY_TIME_RANDOM_RANGE"][0].as<double>();
	this->delay_random[0] = config["interceptor"]["DELAY_TIME_RANDOM_RANGE"][1].as<double>();
	this->tracking_method = config["interceptor"]["TRACKING_MATHOD"].as<string>();
	this->guidence_p = config["interceptor"]["PROPORTIONAL_GUIDANCE_P"].as<double>();
	this->guidence_cmp = config["interceptor"]["PROPORTIONAL_GUIDANCE_COMPENSATE"].as<double>();
	this->speed_limit = config["interceptor"]["SPEED_LIMIT"].as<double>();
	this->angle_limit = config["interceptor"]["ANGLE_LIMIT"].as<double>();
	this->tolerance = config["interceptor"]["HIT_TOLERANCE"].as<double>();

	if (this->delay_time_cfg == "random")
		this->delay_time = uniform(this->delay_random[0], this->delay_random[1]);
	else{
		this->delay_time = stod(delay_time_cfg);
	}

	this->error_scan();
	if (sim->error_flag == true)		return;
	//------txt------------------
	outfilepath = config["simulator"]["OUTPUT_TRAJECTION_PATH"].as<string>();
}

/// <summary>
/// ��ȡĿ�굯��������Ϣ
/// </summary>
/// <param name="obj">����Ŀ�굯����obj</param>
void Hitter::pos_radar_reading(Obj obj) {

	vector<double> temp;
	for (int i = 0; i <= 2; i++)
		temp.push_back(obj.pos[i]);

	obj_pos_hist.push_back(temp);
}

/// <summary>
/// ����Ŀ�굯�Ĺ켣Ԥ��һʱ��ƬĿ�굯���ֵ�λ��
/// </summary>
void Hitter::estimate_new_position() {
	double v1_x, v2_x;
	double v1_y, v2_y;
	double v1_z, v2_z;
	double a_z;

	if (this->obj_pos_hist.size() > 3) {
		v1_x = (this->obj_pos_hist[this->obj_pos_hist.size() - 1][0] - this->obj_pos_hist[this->obj_pos_hist.size() - 2][0]) / this->sim->dt;
		v2_x = (this->obj_pos_hist[this->obj_pos_hist.size() - 2][0] - this->obj_pos_hist[this->obj_pos_hist.size() - 3][0]) / this->sim->dt;

		v1_y = (this->obj_pos_hist[this->obj_pos_hist.size() - 1][1] - this->obj_pos_hist[this->obj_pos_hist.size() - 2][1]) / this->sim->dt;
		v2_y = (this->obj_pos_hist[this->obj_pos_hist.size() - 2][1] - this->obj_pos_hist[this->obj_pos_hist.size() - 3][1]) / this->sim->dt;
		
		v1_z = (this->obj_pos_hist[this->obj_pos_hist.size() - 1][2] - this->obj_pos_hist[this->obj_pos_hist.size() - 2][2]) / this->sim->dt;
		v2_z = (this->obj_pos_hist[this->obj_pos_hist.size() - 2][2] - this->obj_pos_hist[this->obj_pos_hist.size() - 3][2]) / this->sim->dt;

		a_z = (v1_z - v2_z) / this->sim->dt;

		this->estimate_obj_next_pos[0] = this->obj_pos_hist.back()[0] + v1_x * this->sim->dt;
		this->estimate_obj_next_pos[1] = this->obj_pos_hist.back()[1] + v1_y * this->sim->dt;
		this->estimate_obj_next_pos[2] = this->obj_pos_hist.back()[2] + v1_z * this->sim->dt + a_z * this->sim->dt * this->sim->dt;
	}

}

/// <summary>
/// ׷��Ŀ�굯������д��idel��proportional_guidance����׷���߼�
/// </summary>
void Hitter::interseptor_tracking() {
	double kp = this->guidence_p;
	double k_cmp = this->guidence_cmp;
	double dt = this->sim->dt;
	
	double oep[3];

	double ex, ey, ez;
	double v_x, v_y, v_z;
	double d;
	double v,ratio;
	

	for (int i = 0; i <= 2; i++) {
		oep[i] = this->estimate_obj_next_pos[i];
		this->present_pos[i] = this->pos[i];
	}

	if (this->sim->time <= this->delay_time + this->searching_time) {
		for (int i = 0; i <= 2; i++)
			oep[i] = this->init_target[i];
	}

	ex = oep[0] - this->present_pos[0];
	ey = oep[1] - this->present_pos[1];
	ez = oep[2] - this->present_pos[2];

	d = sqrt(ex * ex + ey * ey + ez * ez);

	if (d < 2)
	{
		this->ac_error[0] += ex;
		this->ac_error[1] += ey;
		this->ac_error[2] += ez;
	}
	
	if (this->sim->time > this->delay_time) {

		if (this->tracking_method == "idel") {
			if (ex >= dt || ex <= -dt)
				this->pos[0] += ex / abs(ex) * dt + uniform(-0.05, 0.05);
			else if (ex > 0 || ex < 0)
				this->pos[0] += ex;

			if (ey >= dt || ey <= -dt)
				this->pos[1] += ey / abs(ey) * dt;
			else if (ey > 0 || ey < 0)
				this->pos[1] += ey;
			
			if (ez >= dt || ez <= -dt)
				this->pos[2] += ez / abs(ez) * dt;
			else if (ez > 0 || ez < 0)
				this->pos[2] += ez;

		}

		else if (tracking_method == "proportional_guidance") {
			v_x = ex * kp + this->ac_error[0] * k_cmp;
			v_y = ey * kp + this->ac_error[1] * k_cmp;
			v_z = ez * kp + this->ac_error[2] * k_cmp;

			v = sqrt(v_x * v_x + v_y * v_y + v_z * v_z);

			if (v > this->speed_limit) {
				ratio = this->speed_limit / v;

				v_x *= ratio;
				v_y *= ratio;
				v_y *= ratio;

			}

			if (ex >= dt || ex <= -dt)
				this->pos[0] += v_x * dt + uniform(-0.05, 0.05);
			else if (ex > 0 || ex < 0)
				this->pos[0] += ex;

			if (ey >= dt || ey <= -dt)
				this->pos[1] += v_y * dt;
			else if (ey > 0 || ey < 0)
				this->pos[1] += ey;

			if (ez >= dt || ez <= -dt)
				this->pos[2] += v_z * dt;
			else if (ez > 0 || ez < 0)
				this->pos[2] += ez;

			this->adjust_p3_to_angle_fixed_distance(this->last_pos, this->present_pos, this->pos, this->angle_limit);
		}
	}

	for (int i = 0; i <= 2; i++)
		this->last_pos[i] = this->present_pos[i];
}

/// <summary>
/// �������ص���������Ϣ�������־��
/// </summary>
void Hitter::save_txt() {

	ofstream outfile(outfilepath, ios::app);

	outfile << sim->time << "\t" << this->name << "\t";
	for (int i = 0; i <= 2; i++) {
		if(this->sim->coordinate == "cartesian")
			outfile << this->pos[i] << "\t";
		else if (this->sim->coordinate == "GPS")
			outfile << this->gps_pos[i] << "\t";

	}
	outfile << endl;

	outfile.close();

}

/// <summary>
/// �ж�Ŀ�굯�Ƿ����ص�����
/// </summary>
/// <param name="obj">����Ŀ�굯����obj��Ϣ</param>
/// <returns>���ز���ֵ�����з���true��δ���з���false</returns>
bool Hitter::is_hit(Obj* obj) {
	double x, y, z,d;

	x = obj->pos[0] - this->pos[0];
	y = obj->pos[1] - this->pos[1];
	z = obj->pos[2] - this->pos[2];

	d = sqrt(x * x + y * y + z * z);
	if (d < this->tolerance)
		return true;

}

/// <summary>
/// �������������ļн�
/// </summary>
/// <param name="v1">��һ������������</param>
/// <param name="v2">�ڶ�������������</param>
/// <returns>�������������ļн�</returns>
double Hitter::vector_angle(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
	// �������������ļнǣ��Ի��ȱ�ʾ��
	double dot_product = v1.dot(v2);
	double magnitude_v1 = v1.norm();
	double magnitude_v2 = v2.norm();
	double cos_theta = dot_product / (magnitude_v1 * magnitude_v2);
	cos_theta = std::clamp(cos_theta, -1.0, 1.0);  // ȷ��cos_theta��[-1, 1]֮��
	double theta = std::acos(cos_theta);
	return theta;
}

/// <summary>
///  �ж����ص��´��ƶ���λ����ǰ�����ƶ��ĵ�֮ǰ�Ĺ켣��ת���Ƿ�̫��̫�������ת�����ơ�
/// </summary>
/// <param name="a">��һ��ʱ��Ƭ���ص���λ����Ϣ</param>
/// <param name="b">��ǰʱ��Ƭ���ص���λ����Ϣ</param>
/// <param name="c">�µ�ʱ��Ƭ���ص���λ����Ϣ</param>
/// <param name="target_angle_degrees">���ת������</param>
/// <returns>�����µ����ص���λ����Ϣ���糬�����ת�ǣ�����ת��Լ�������������ꡣδ�������ת�ǣ��򷵻�ԭ����</returns>
Eigen::Vector3d Hitter::adjust_p3_to_angle_fixed_distance(double a[3], double b[3], double c[3], double target_angle_degrees) {
	// �ƶ�P3ʹ��v1��v2�ļнǵ���Ŀ��Ƕȣ�������p3��p2�ľ��벻��

	Eigen::Vector3d p1(a[0], a[1], a[2]);
	Eigen::Vector3d p2(b[0], b[1], b[2]);
	Eigen::Vector3d p3(c[0], c[1], c[2]);

	Eigen::Vector3d v1 = p2 - p1;
	if (v1.isZero()) {
		return p3;
	}
	Eigen::Vector3d v2 = p3 - p2;
	double current_angle = to_degrees(vector_angle(v1, v2));
	double target_angle_radians = to_radians(target_angle_degrees);

	if (current_angle <= target_angle_degrees) {
		return p3;  // �����ǰ�н��Ѿ�С�ڵ���Ŀ��Ƕȣ�����ԭ��
	}

	// ������ת�� (v1��v2�Ĳ������)
	Eigen::Vector3d axis = v1.cross(v2).normalized();

	// ������ת����Rodrigues' rotation formula��
	Eigen::Matrix3d K;
	K << 0, -axis.z(), axis.y(),
		axis.z(), 0, -axis.x(),
		-axis.y(), axis.x(), 0;

	Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d R = I + std::sin(target_angle_radians) * K + (1 - std::cos(target_angle_radians)) * K * K;

	// ��v2��ת��Ŀ��Ƕȣ�ͬʱ����ԭ����
	Eigen::Vector3d v2_rotated = R * v1.normalized() * v2.norm();

	// �µ�p3λ��
	Eigen::Vector3d new_p3 = p2 + v2_rotated;

	this->pos[0] = new_p3[0];
	this->pos[1] = new_p3[1];
	this->pos[2] = new_p3[2];

	return new_p3;
}

/// <summary>
/// ��ά����ת��ΪСƽ���GPS����
/// </summary>
/// <param name="sim">�������������sim��Ϣ</param>
void Hitter::change_gps(Sim sim) {
	for (int i = 0; i <= 1; i++)
		gps_pos[i] = pos[i] * 0.02 + sim.loc[i];
	gps_pos[2] = pos[2] * 1000 + sim.loc[2];
}

void Hitter::error_scan() {
	if (this->delay_time < 0) {
		cout << "Error: DELAY_TIME����С��0" << endl;
		this->sim->error_flag = true;
	}
	else if (this->delay_time > this->sim->max_sim_size) {
		cout << "Error: DELAY_TIME���ܴ��ڵ���MAX_SIM_SIZE" << endl;
		this->sim->error_flag = true;
	}

	if (this->searching_time < 0) {
		cout << "Error: SEARCHING_TIME����С��0" << endl;
		this->sim->error_flag = true;
	}

	if (this->tracking_method != "proportional_guidance" && this->tracking_method != "idel") {
		cout << "Error: TRACKING_MATHOD��������" << endl;
		this->sim->error_flag = true;
	}
	if (this->tolerance < 0) {
		cout << "Error: HIT_TOLERANCE����С��0" << endl;
		this->sim->error_flag = true;
	}
}



