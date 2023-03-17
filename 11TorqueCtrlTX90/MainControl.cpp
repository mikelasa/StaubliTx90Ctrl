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
#include "PLOTS.h"
#include "TX90ProxyByTorque.h"
#include "TrajQuinticPVA.h"
#include "PIDCtrlPVA.h"
#include "TrajLSPBPVA.h"
#include "TX90Dynamics.h"
#include <math.h>
#include "matplotlibcpp.h"
#include "DynamicCompesator.h"
#include "HystoricData.h"

# define M_PI           3.14159265358979323846

using namespace std;
using namespace Eigen;
namespace plt = matplotlibcpp;

//Time step
float const deltaT = 0.004;
float const TrajTime = 10;

//PID GAINS 
std::vector<float> const Kp = { 1200,		600,		60,		600,		700,		900 };
std::vector<float> const Kv = { 20,			10,			12,			10,			13,			17 };
std::vector<float> const Ki = { 0.1,		0.1,		0.35,		0.4,		0.4,		0.7 };

// Initial and final position for robot axis
std::vector<float> const posI = { 0,		0,			0,			0,		0,		0 };
std::vector<float> const posF = { 1.57,		-0.78 ,			1.57,			1.57	,		1.57,		1.57 };

// LSPB trayectory constant speed
std::vector<float> velConsts = { 0.19,0,0,0,0,0 };

std::vector<std::vector<float>> commandedTorqueToFile(6);
std::vector<std::vector<float>> qrToFile(6);
std::vector<std::vector<float>> dqrToFile(6);
std::vector<std::vector<float>> qcToFile(6);
std::vector<std::vector<float>> dqcToFile(6);
std::vector<std::vector<float>> ddqcToFile(6);

//fills the array of PID controllers for both trayectories
void fillVectorOfControllerWithQuinticTraj(PIDCtrlPVA* pCtrls[]);
void fillVectorOfControllerWithLSPBTraj(PIDCtrlPVA* pCtrls[]);

int main()
{
	TX90ProxyByTorque robot;
	PIDCtrlPVA* pCtrls[6];
	DynamicCompesator compensator;
	HystoricData hystoricData;

	// fills the PID pointers with the needed trayectory
	fillVectorOfControllerWithQuinticTraj(pCtrls);

	//main loop
	for (int i = 0; i < pCtrls[1]->getTrajSize(); i++)
	{
		VectorXd robotCommand(6);
		std::vector<float> tau(6);
		std::vector<float> dqidqj(15);

		MatrixXd compensatedTorque;
		
		// PID
		for (int j = 0; j < 6; j++)
		{
			robotCommand[j] = pCtrls[j]->getPidCommand(i, robot.getPosition(j), robot.getSpeed(j));
		}

		// position and speed are used by the Dynamic compesator class to calculate the compensation terms C(q,q) and g(q)
		compensator.setPosAndVel(robot.getPositions(), robot.getSpeeds());

		// compute compensation 
		compensatedTorque = (compensator.getMassMatrix() * robotCommand) + compensator.getDynamicComp();// + compensator.getFric() * i * 0.04;
	
		//std::cout << "Here is the matrix compensatedTorque:\n" << compensator.getDynamicComp() << std::endl;
		/*std::cout << "Here is the matrix massMatrix:\n" << massMatrix << std::endl;
		std::cout << "Here is the matrix robotCommand:\n" << robotCommand << std::endl;
		std::cout << "Here is the matrix fric:\n" << fric << std::endl;
		std::cout << "Here is the matrix centrCor:\n" << centrCor << std::endl;
		std::cout << "Here is the matrix g:\n" << g << std::endl;*/

		std::cout << "iteration" << i <<std::endl;

		//save data to export 
		hystoricData.addQr(robot.getPositions());
		hystoricData.addDqr(robot.getSpeeds());
		hystoricData.addTorque(compensatedTorque);

		for (int j = 0; j < 6; j++)
		{
			tau[j] = compensatedTorque(j);
			//tau[j] = vec2[j].at(i);// +10;
		}

		robot.sendTorqueCommand(tau);

		robot.refresh();
	}

	hystoricData.writeToFile("record.txt", pCtrls[3]->getTrajPVA());
	
	//as dynamic variables are not self deleted, they need to be deleted to save memory
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
		TrajPVA* pt = new TrajQuinticPVA(posI[i], posF[i], TrajTime, deltaT);
		pCtrls[i] = new PIDCtrlPVA(pt, Kp[i], Ki[i], Kv[i]);

	}
}

void fillVectorOfControllerWithLSPBTraj(PIDCtrlPVA* pCtrls[])
{
	for (int i = 0; i < 6; i++)
	{
		TrajPVA* pt = new TrajLSPBPVA(posI[i], posF[i], velConsts[i], TrajTime, deltaT);
		pCtrls[i] = new PIDCtrlPVA(pt, Kp[i], Ki[i], Kv[i]);
	}
}

