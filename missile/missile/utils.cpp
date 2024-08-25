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
/// ����������ȷֲ�
/// </summary>
/// <param name="min">�������Сֵ</param>
/// <param name="max">������ֵ</param>
/// <returns>���������</returns>
double uniform(double min, double max) {
    std::random_device rd;  // ������������
    std::mt19937 gen(rd()); // ʹ��Mersenne Twister�����ʼ��������

    //double max_sim_size = 10.0;  // �趨max_sim_size��ֵ
    std::uniform_real_distribution<> dist(min, max);  // �趨���ȷֲ��ķ�Χ

    double number = dist(gen);  // ���ɷ�Χ�ڵ������

    //std::cout << "Random value: " << number << std::endl;
	return number;

}

/// <summary>
/// ���ѡ�����������м������ѡһ����������50%�ĸ���
/// </summary>
/// <param name="a">�����a</param>
/// <param name="b">�����b</param>
/// <returns>���������</returns>
double random_choice(double a, double b) {
    std::random_device rd;  // ������������
    std::mt19937 gen(rd()); // ʹ��Mersenne Twister�����ʼ��������

    // ����һ�����ȷֲ���0��1������
    std::uniform_int_distribution<> dist(0, 1);

    // ���ѡ��a��b
    double result = (dist(gen) == 0) ? a : b;
    return result;
}

/// <summary>
/// ������ת��Ϊ�Ƕ�
/// </summary>
/// <param name="radians">���Ƚ�</param>
/// <returns>�ǶȽ�</returns>
double to_degrees(double radians) {
    return radians * 180.0 / M_PI;
}

/// <summary>
/// �Ƕ�ֵת��Ϊ������
/// </summary>
/// <param name="degrees">�ǶȽ�</param>
/// <returns>���Ƚ�</returns>
double to_radians(double degrees) {
    return degrees * M_PI / 180.0;
}

