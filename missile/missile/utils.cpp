#include <ctime>
#include <cstdlib>
#include <random>
#include <iostream>
#include <corecrt_math_defines.h>
#include <cmath>
#include <Eigen/Dense>
#include <numbers>
#include "utils.h"
#include "hitter.h"
#include "simulation.h"

using namespace std;

/// <summary>
/// 随机数，均匀分布
/// </summary>
/// <param name="min">随机数最小值</param>
/// <param name="max">最技术最大值</param>
/// <returns>返回随机数</returns>
double uniform(double min, double max) {
    std::random_device rd;  // 用于生成种子
    std::mt19937 gen(rd()); // 使用Mersenne Twister引擎初始化生成器

    //double max_sim_size = 10.0;  // 设定max_sim_size的值
    std::uniform_real_distribution<> dist(min, max);  // 设定均匀分布的范围

    double number = dist(gen);  // 生成范围内的随机数

    //std::cout << "Random value: " << number << std::endl;
	return number;

}

/// <summary>
/// 随机选择，在两个数中间随机挑选一个数，各有50%的概率
/// </summary>
/// <param name="a">随机数a</param>
/// <param name="b">随机数b</param>
/// <returns>返回随机数</returns>
double random_choice(double a, double b) {
    std::random_device rd;  // 用于生成种子
    std::mt19937 gen(rd()); // 使用Mersenne Twister引擎初始化生成器

    // 生成一个均匀分布的0或1的整数
    std::uniform_int_distribution<> dist(0, 1);

    // 随机选择a或b
    double result = (dist(gen) == 0) ? a : b;
    return result;
}

/// <summary>
/// 弧度制转换为角度
/// </summary>
/// <param name="radians">弧度角</param>
/// <returns>角度角</returns>
double to_degrees(double radians) {
    return radians * 180.0 / M_PI;
}

/// <summary>
/// 角度值转换为弧度制
/// </summary>
/// <param name="degrees">角度角</param>
/// <returns>弧度角</returns>
double to_radians(double degrees) {
    return degrees * M_PI / 180.0;
}

