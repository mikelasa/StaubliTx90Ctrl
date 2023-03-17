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
#include "TX90ProxyByPosition.h"
#include "TrajQuinticPVA.h"
#include "PIDCtrlPV.h"
#include "TrajCubicPV.h"
#include "TrajLSPBPVA.h"

using namespace std;
using namespace Eigen;

float const deltaT = 0.004;
std::vector<float> const Kp = { 30,	10,		15,  15,	15,	 15 };
std::vector<float> const Kv = { 0.075,	0.075,	0.075,	0.075,	0.075,	0.075 };
std::vector<float> const Ki = { 0,	0,	0,	0,	0,	0 };

std::vector<float> const posI = { 0,0,0,0,0,0 };
std::vector<float> const posF = { 1.5708, 0, 0, 0, 0, 0 };
std::vector<float> velConsts = { 0.19,0,0,0,0,0 };

void fillVectorOfControllerWithQuinticTraj(PIDCtrlPV* pCtrls[]);
void fillVectorOfControllerWithLSPBTraj(PIDCtrlPV* pCtrls[]);
void fillVectorOfControllerWithCubicTraj(PIDCtrlPV* pCtrls[]);

int main()
{
	TX90ProxyByPosition robot;
	PIDCtrlPV* pCtrls[6];

	fillVectorOfControllerWithQuinticTraj(pCtrls);
	for (int i = 0; i < pCtrls[1]->getTrajSize(); i++)
	{
		std::vector<float> actualpos;
		std::vector<float> actualvel;
		std::vector<std::tuple <float, float>> robotCommand(6);

		actualpos = robot.getPositions();
		actualvel = robot.getSpeeds();

		for (int j = 0; j < 6; j++)
		{
			robotCommand[j] = pCtrls[j]->getPidCommand(i, actualpos[j], actualvel[j]);
		}

		robot.sendCommand(actualvel, robotCommand);

		robot.refresh();
	}

	for (int i = 0; i < 6; i++)
	{
		delete pCtrls[i];
	}


	std::this_thread::sleep_for(std::chrono::milliseconds(8));


	return 0;
}

void fillVectorOfControllerWithQuinticTraj(PIDCtrlPV* pCtrls[])
{
	for (int i = 0; i < 6; i++)
	{
		TrajPV* pt = new TrajQuinticPVA(posI[i], posF[i], 5, deltaT);
		pCtrls[i] = new PIDCtrlPV(pt, Kp[i], Ki[i], Kv[i]);

	}
}

void fillVectorOfControllerWithCubicTraj(PIDCtrlPV* pCtrls[])
{
	for (int i = 0; i < 6; i++)
	{
		TrajPV* pt = new TrajCubicPV(posI[i], posF[i], 5, deltaT);
		pCtrls[i] = new PIDCtrlPV(pt, Kp[i], Ki[i], Kv[i]);
	}
}

void fillVectorOfControllerWithLSPBTraj(PIDCtrlPV* pCtrls[])
{
	for (int i = 0; i < 6; i++)
	{
		TrajPV* pt = new TrajLSPBPVA(posI[i], posF[i], velConsts[i], 5, deltaT);
		pCtrls[i] = new PIDCtrlPV(pt, Kp[i], Ki[i], Kv[i]);
	}
}

// PLOT DATA 
	// from radians to degrees
	/*
	float rad2deg = (180.0 / 3.141592653589793238463);
	for (int i = 0; i < qc[0].size(); i++) {
		for (int j = 0; j < 6; j++) {
			qc[j][i] = qc[j][i] * rad2deg;
			dqc[j][i] = dqc[j][i] * rad2deg;
			plotPos[j][i] = plotPos[j][i] * rad2deg;
			plotVel[j][i] = plotVel[j][i] * rad2deg;
		}
	}
	*/
	//?
