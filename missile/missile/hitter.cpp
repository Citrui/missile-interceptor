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
/// 初始化拦截弹对象
/// </summary>
/// <param name="_config">导入yaml配置文件</param>
/// <param name="_init_target">拦截弹发射时初始朝向位置</param>
/// <param name="_init_pos">拦截弹发射的初始坐标</param>
/// <param name="_sim">导入仿真控制sim对象</param>
void Hitter::init(YAML::Node _config, double _init_target[3], double _init_pos[3],Sim *_sim) {
	this->name = "hitter";
	this->config = _config;
	this->sim = _sim;

	//--------坐标----------
	for (int i = 0; i <= 2; i++) {
		this->pos[i] = _init_pos[i];
		this->last_pos[i] = _init_pos[i];
		this->present_pos[i] = _init_pos[i];
		this->init_target[i] = _init_target[i];
		this->estimate_obj_next_pos[i] = _init_pos[i];
	}

	//---------控制---------------
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
/// 读取目标弹的坐标信息
/// </summary>
/// <param name="obj">导入目标弹对象obj</param>
void Hitter::pos_radar_reading(Obj obj) {

	vector<double> temp;
	for (int i = 0; i <= 2; i++)
		temp.push_back(obj.pos[i]);

	obj_pos_hist.push_back(temp);
}

/// <summary>
/// 根据目标弹的轨迹预下一时间片目标弹出现的位置
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
/// 追踪目标弹，其中写了idel和proportional_guidance两种追踪逻辑
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
/// 保存拦截弹的坐标信息到输出日志中
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
/// 判断目标弹是否被拦截弹击中
/// </summary>
/// <param name="obj">导入目标弹对象obj信息</param>
/// <returns>返回布尔值，击中返回true，未击中返回false</returns>
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
/// 计算两个向量的夹角
/// </summary>
/// <param name="v1">第一个向量的坐标</param>
/// <param name="v2">第二个向量的坐标</param>
/// <returns>返回两个向量的夹角</returns>
double Hitter::vector_angle(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
	// 计算两个向量的夹角（以弧度表示）
	double dot_product = v1.dot(v2);
	double magnitude_v1 = v1.norm();
	double magnitude_v2 = v2.norm();
	double cos_theta = dot_product / (magnitude_v1 * magnitude_v2);
	cos_theta = std::clamp(cos_theta, -1.0, 1.0);  // 确保cos_theta在[-1, 1]之间
	double theta = std::acos(cos_theta);
	return theta;
}

/// <summary>
///  判断拦截弹下次移动的位置与前两次移动的点之前的轨迹的转角是否太大，太大则进行转角限制。
/// </summary>
/// <param name="a">上一个时间片拦截弹的位置信息</param>
/// <param name="b">当前时间片拦截弹的位置信息</param>
/// <param name="c">新的时间片拦截弹的位置信息</param>
/// <param name="target_angle_degrees">最大转角限制</param>
/// <returns>返回新的拦截弹的位置信息，如超过最大转角，进行转角约束，产生新坐标。未超过最大转角，则返回原坐标</returns>
Eigen::Vector3d Hitter::adjust_p3_to_angle_fixed_distance(double a[3], double b[3], double c[3], double target_angle_degrees) {
	// 移动P3使得v1和v2的夹角等于目标角度，并保持p3到p2的距离不变

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
		return p3;  // 如果当前夹角已经小于等于目标角度，返回原点
	}

	// 计算旋转轴 (v1与v2的叉积方向)
	Eigen::Vector3d axis = v1.cross(v2).normalized();

	// 计算旋转矩阵（Rodrigues' rotation formula）
	Eigen::Matrix3d K;
	K << 0, -axis.z(), axis.y(),
		axis.z(), 0, -axis.x(),
		-axis.y(), axis.x(), 0;

	Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d R = I + std::sin(target_angle_radians) * K + (1 - std::cos(target_angle_radians)) * K * K;

	// 将v2旋转到目标角度，同时保持原长度
	Eigen::Vector3d v2_rotated = R * v1.normalized() * v2.norm();

	// 新的p3位置
	Eigen::Vector3d new_p3 = p2 + v2_rotated;

	this->pos[0] = new_p3[0];
	this->pos[1] = new_p3[1];
	this->pos[2] = new_p3[2];

	return new_p3;
}

/// <summary>
/// 三维坐标转换为小平面的GPS坐标
/// </summary>
/// <param name="sim">导入仿真对象变量sim信息</param>
void Hitter::change_gps(Sim sim) {
	for (int i = 0; i <= 1; i++)
		gps_pos[i] = pos[i] * 0.02 + sim.loc[i];
	gps_pos[2] = pos[2] * 1000 + sim.loc[2];
}

void Hitter::error_scan() {
	if (this->delay_time < 0) {
		cout << "Error: DELAY_TIME不能小于0" << endl;
		this->sim->error_flag = true;
	}
	else if (this->delay_time > this->sim->max_sim_size) {
		cout << "Error: DELAY_TIME不能大于等于MAX_SIM_SIZE" << endl;
		this->sim->error_flag = true;
	}

	if (this->searching_time < 0) {
		cout << "Error: SEARCHING_TIME不能小于0" << endl;
		this->sim->error_flag = true;
	}

	if (this->tracking_method != "proportional_guidance" && this->tracking_method != "idel") {
		cout << "Error: TRACKING_MATHOD配置有误" << endl;
		this->sim->error_flag = true;
	}
	if (this->tolerance < 0) {
		cout << "Error: HIT_TOLERANCE不能小于0" << endl;
		this->sim->error_flag = true;
	}
}



