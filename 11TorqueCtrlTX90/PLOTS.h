#pragma once

#ifndef PLOTS_H
#define PLOTS_H

#include <iostream>
#include <fstream>
#include <cstdio>
#include <sstream>
#include <string>
#include <queue>


void plotData(std::vector<std::vector<double>> qc, std::vector<std::vector<double>> dqc, std::vector<std::vector<double>> qr, std::vector<std::vector<double>> dqr, std::vector<std::vector<double>> ddqc, std::vector<std::vector<double>> errorPos, std::vector<std::vector<double>> errorVel, double plotN);

#endif // !PLOTS_H


