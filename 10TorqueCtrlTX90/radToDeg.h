#pragma once
#ifndef RADTODEG_H
#define RADTODEG_H

#include <vector>

using namespace std;

std::tuple <std::vector<std::vector<double>>, std::vector<std::vector<double>>, std::vector<std::vector<double>>, std::vector<std::vector<double>>, std::vector<std::vector<double>>, std::vector<std::vector<double>>, std::vector<double>, std::vector<std::vector<double>>, std::vector<std::vector<double>> >RadToDeg(std::vector<std::vector<double>> qc, std::vector<std::vector<double>> dqc, std::vector<std::vector<double>> qr, std::vector<std::vector<double>> dqr, std::vector<std::vector<double>> errorPos, std::vector<std::vector<double>> errorVel, std::vector<double> errorInt, std::vector<std::vector<double>> PIDOUT, std::vector<std::vector<double>> velRobotcalc, int trajlength);




#endif // !RADTODEG_H