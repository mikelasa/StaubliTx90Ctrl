#include "HystoricData.h"
#include <fstream>

HystoricData::HystoricData()
{
	commandedTorque = std::vector<std::vector<float>>(6);
	qr = std::vector<std::vector<float>>(6);
	dqr = std::vector<std::vector<float>>(6);
	qc = std::vector<std::vector<float>>(6);
	dqc = std::vector<std::vector<float>>(6);
	ddqc = std::vector<std::vector<float>>(6);
}

void HystoricData::addQr(std::vector<float> posRobot)
{
	for (int j = 0; j < 6; j++)
	{
		qr[j].push_back(posRobot[j]);
	}
}

void HystoricData::addDqr(std::vector<float> velRobot)
{
	for (int j = 0; j < 6; j++)
	{
		dqr[j].push_back(velRobot[j]);
	}
}

void HystoricData::addTorque(Eigen::MatrixXd torque)
{
	for (int j = 0; j < 6; j++)
	{
		commandedTorque[j].push_back(torque(j));
	}
}

void HystoricData::writeToFile(const char fileName[],TrajPVA& trj)
{
	std::ofstream outputFile;
	outputFile.open("record.txt");

	for (int count = 0; count < qr[0].size(); count++) 
	{
		//commandedTorqueToFile
		for (int count1 = 0; count1 < 6; count1++) {
			outputFile << commandedTorque[count1].at(count) << ';';
		}
		//qreToFile
		for (int count2 = 0; count2 < 6; count2++) {
			outputFile << qr[count2].at(count) << ';';
		}
		//dqrToFile
		for (int count3 = 0; count3 < 6; count3++) {
			outputFile << dqr[count3].at(count) << ';';
		}

		outputFile << trj.getqc(count) << ';';
		outputFile << trj.getdqc(count) << ';';
		outputFile << trj.getddqc(count) << ';';

		outputFile << std::endl;

	}
	outputFile.close();
}