#pragma once

#include <Eigen/Dense>
#include "simulation.h"


double uniform(double min, double max);
double random_choice(double a, double b);
double to_degrees(double radians);
double to_radians(double degrees);
//double vector_angle(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);
//Eigen::Vector3d adjust_p3_to_angle_fixed_distance(double a[3], double b[3], double c[3], double target_angle_degrees);
//void change_gps(double pos[3],Sim sim);


