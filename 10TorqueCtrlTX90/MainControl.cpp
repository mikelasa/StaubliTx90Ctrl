#include <iostream>
#include <fstream>
#include <cstdio>
#include <sstream>
#include <string>
#include <queue>
#include <chrono>
#include <thread>

#include "Eigen/Dense"
#include <vector>

#include "radToDeg.h"
#include "StaubliUdpPort.h"
#include "writeToFile.h"
#include "PLOTS.h"
#include "TX90ProxyByTorque.h"
#include "TrajQuinticPVA.h"
#include "PIDCtrlPVA.h"
#include "TrajLSPBPVA.h"
#include "TX90Dynamics.h"
#include <math.h>
#include "matplotlibcpp.h"

# define M_PI           3.14159265358979323846

using namespace std;
using namespace Eigen;
namespace plt = matplotlibcpp;

float const deltaT = 0.004;
std::vector<float> const Kp = { 1200,	600,	10,  15,	15,	 15 };
std::vector<float> const Kv = {20,	10,	0.075,	0.075,	0.075,	0.075 };
std::vector<float> const Ki = { 0.1,	0.1,	0,	0,	0,	0 };

std::vector<float> const posI = { 0,-1.57,0,0,0,0 };
std::vector<float> const posF = { 0, 0,0 , 0, 0, 0 };
std::vector<float> velConsts = { 0.19,0,0,0,0,0 };
std::vector<std::vector<float>> commandedTorqueToFile(6);
std::vector<std::vector<float>> qrToFile(6);
std::vector<std::vector<float>> dqrToFile(6);
std::vector<std::vector<float>> qcToFile(6);
std::vector<std::vector<float>> dqcToFile(6);
std::vector<std::vector<float>> ddqcToFile(6);

void fillVectorOfControllerWithQuinticTraj(PIDCtrlPVA* pCtrls[]);
void fillVectorOfControllerWithLSPBTraj(PIDCtrlPVA* pCtrls[]);
void WriteFile(ofstream& outputFile, std::vector<std::vector<float>> tauValue, std::vector<std::vector<float>> qrValue, std::vector<std::vector<float>> dqrValue, float lengthTrack, TrajPVA& trj);

int main()
{
	TX90ProxyByTorque robot;
	PIDCtrlPVA* pCtrls[6];

	fillVectorOfControllerWithQuinticTraj(pCtrls);

	for (int i = 0; i < pCtrls[1]->getTrajSize(); i++)
	{
		std::vector<float> actualPos(6);
		std::vector<float> actualVel(6);
		VectorXd robotCommand(6);
		std::vector<float> tau(6);
		std::vector<float> dqidqj(15);

		
		MatrixXd fric;
		MatrixXd massMatrix;
		MatrixXd dynamicComp;
		MatrixXd CompensatedTorque;

		actualPos = robot.getPositions();
		actualVel = robot.getSpeeds();

		massMatrix = compute_fregMM(actualPos[1], actualPos[2], actualPos[3], actualPos[4], actualPos[5]);
		dynamicComp = compute_fregCentrCorG(actualVel[0], actualVel[1], actualVel[2], actualVel[3], actualVel[4], actualVel[5], actualPos[1], actualPos[2], actualPos[3], actualPos[4], actualPos[5]);
		fric = compute_fregFric(actualVel[0], actualVel[1], actualVel[2], actualVel[3], actualVel[4], actualVel[5]);
		
		// PID
		for (int j = 0; j < 6; j++)
		{
			robotCommand[j] = pCtrls[j]->getPidCommand(i, actualPos[j], actualVel[j]);
		}
		
		if (i <= 20)
		{
			CompensatedTorque = (massMatrix * robotCommand) + dynamicComp;// +fric * i * 0.04;
		}
		else
		{
			CompensatedTorque = (massMatrix * robotCommand) + dynamicComp;// +fric;
		}

		//std::cout << "Here is the matrix CompensatedTorque:\n" << dynamicComp << std::endl;
		//std::cout << "Here is the matrix massMatrix:\n" << massMatrix << std::endl;
		//std::cout << "Here is the matrix robotCommand:\n" << robotCommand << std::endl;
		//std::cout << "Here is the matrix fric:\n" << fric << std::endl;



		std::cout << "iteration" << i <<std::endl;

		for (int j = 0; j < 6; j++)
		{

			tau[j] = CompensatedTorque(j);
			//tau[j] = vec2[j].at(i);// +10;
			commandedTorqueToFile[j].push_back(tau[j]);
			qrToFile[j].push_back(actualPos[j]);
			dqrToFile[j].push_back(actualVel[j]);
		
		}

		robot.sendTorqueCommand(tau);

		robot.refresh();
	}


	ofstream outputFile;
	outputFile.open("record.txt");
	WriteFile(outputFile, commandedTorqueToFile, qrToFile, dqrToFile, pCtrls[1]->getTrajSize(), pCtrls[1]->getTrajPVA());
	outputFile.close();

	//plt::figure();
	//plt::title("Trajectory");
	//plt::named_plot("Torque", commandedTorqueToFile[0], "k--");
	//plt::figure();
	//plt::named_plot("qr", qrToFile[0], "b");
	//plt::figure();
	//plt::named_plot("dqr", dqrToFile[0], "b");
	//plt::show();

	for (int i = 0; i < 6; i++)
	{
		delete pCtrls[i];
	}

	return 0;
}

void fillVectorOfControllerWithQuinticTraj(PIDCtrlPVA* pCtrls[])
{
	for (int i = 0; i < 6; i++)
	{
		TrajPVA* pt = new TrajQuinticPVA(posI[i], posF[i], 10, deltaT);
		pCtrls[i] = new PIDCtrlPVA(pt, Kp[i], Ki[i], Kv[i]);

	}
}

void fillVectorOfControllerWithLSPBTraj(PIDCtrlPVA* pCtrls[])
{
	for (int i = 0; i < 6; i++)
	{
		TrajPVA* pt = new TrajLSPBPVA(posI[i], posF[i], velConsts[i], 5, deltaT);
		pCtrls[i] = new PIDCtrlPVA(pt, Kp[i], Ki[i], Kv[i]);
	}
}

void WriteFile(ofstream& outputFile, std::vector<std::vector<float>> tauValue, std::vector<std::vector<float>> qrValue, std::vector<std::vector<float>> dqrValue, float lengthTrack, TrajPVA& trj)
{

	for (int count = 0; count < lengthTrack; count++) {
		//commandedTorqueToFile
		for (int count1 = 0; count1 < 6; count1++) {
			outputFile << tauValue[count1].at(count) << ';';
		}
		//qreToFile
		for (int count2 = 0; count2 < 6; count2++) {
			outputFile << qrValue[count2].at(count) << ';';
		}
		//dqrToFile
		for (int count3 = 0; count3 < 6; count3++) {
			outputFile << dqrValue[count3].at(count) << ';';
		}

		outputFile << trj.getqc(count) << ';';
		outputFile << trj.getdqc(count) << ';';
		outputFile << trj.getddqc(count) << ';';

		outputFile << endl;

		
	}
}


//std::vector<float> prodDqiDqj(std::vector<float> dq) {
//	std::vector<float> dqidqj(15, 0.0);
//	int cont = 0;
//
//	for (int i = 0; i < 6; i++) {
//		for (int j = i + 1; j < 6; j++) {
//			dqidqj[cont] = dq[i] * dq[j];
//			cont++;
//		}
//	}
//
//	return dqidqj;
//}