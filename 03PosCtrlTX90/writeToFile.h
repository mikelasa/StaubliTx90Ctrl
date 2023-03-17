#pragma once
#ifndef WRITETOFILE_H
#define WRITETOFILE_H

#include <iostream>
#include <fstream>
#include <cstdio>
#include <sstream>
#include <string>
#include <queue>


void writeToFile(std::ofstream& myfile, std::vector<std::vector<double>> qc, std::vector<std::vector<double>> dqc, std::vector<std::vector<double>> qr, std::vector<std::vector<double>>dqr, std::vector<std::vector<double>> errorPos, std::vector<std::vector<double>> errorVel, std::vector<std::vector<double>> PIDOUT, double lengthTrack);

#endif // !WRITETOFILE_H
