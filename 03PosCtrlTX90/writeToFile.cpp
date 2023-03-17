#include <iostream>
#include <fstream>
#include <cstdio>
#include <sstream>
#include <string>
#include <queue>
#include "writeToFile.h"
#include "Eigen/Dense"

using namespace std;
using namespace Eigen;

void writeToFile(ofstream& myfile, std::vector<std::vector<double>> qc, std::vector<std::vector<double>> dqc, std::vector<std::vector<double>> qr, std::vector<std::vector<double>>dqr, std::vector<std::vector<double>> errorPos, std::vector<std::vector<double>> errorVel, std::vector<std::vector<double>> PIDOUT, double lengthTrack)
{

	for (int count = 0; count < lengthTrack; count++) {
		//qc
		for (int count2 = 0; count2 < 6; count2++) {
			myfile << qc[count2].at(count) << ';';
		}
		//qdc
		for (int count3 = 0; count3 < 6; count3++) {
			myfile << dqc[count3].at(count) << ';';
		}
		//qr
		for (int count4 = 0; count4 < 6; count4++) {
			myfile << qr[count4].at(count) << ';';
		}
		//qdr
		for (int count5 = 0; count5 < 6; count5++) {
			myfile << dqr[count5].at(count) << ';';
		}
		//error pos
		for (int count6 = 0; count6 < 6; count6++) {
			myfile << errorPos[count6].at(count) << ';';
		}
		//error vel
		for (int count7 = 0; count7 < 6; count7++) {
			myfile << errorVel[count7].at(count) << ';';
		}
		//PIDOUT
		for (int count9 = 0; count9 < 6; count9++) {
			myfile << PIDOUT[count9].at(count) << ';';
		}
		myfile << endl;
	}
}