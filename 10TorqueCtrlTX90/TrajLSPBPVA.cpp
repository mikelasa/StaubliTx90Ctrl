#include "TrajLSPBPVA.h"
#include <iostream>
#include "Eigen/Dense"
#include <vector>

using namespace std;
using namespace Eigen;

std::tuple <std::vector<float>, std::vector<float>, std::vector<float>> solveTrajLSPB(float q_0, float q_f, float t_0, float t_f, float consV, float sampling_time, bool DEBUG);

TrajLSPBPVA::TrajLSPBPVA(float posI, float posF, float consV, float timeF, float samplingTime): TrajPVA(samplingTime)
{
	std::tuple <std::vector<float>, std::vector<float>, std::vector<float>>  traj = solveTrajLSPB( posI, posF, 0, 10, consV, samplingTime, 0);
	qc = get<0>(traj);
	dqc = get<1>(traj);
	ddqc = get<2>(traj);
}


std::tuple <std::vector<float>, std::vector<float>, std::vector<float>> solveTrajLSPB(float q_0, float q_f, float t_0, float t_f, float consV, float sampling_time, bool DEBUG)
{
	int n_time = ceil((t_f - t_0) / sampling_time);
	float time = n_time;

	// LIMITS FOR VELOCITY
	float lim1V = (q_f - q_0) / t_f;
	float lim2V = 2 * (q_f - q_0) / t_f;

	//float V = abs(V) * sgn(qf - q0);

	// CALC TIME FOR BLENDS
	float tb;
	if (consV == 0) {
		tb = 0.00001;
	}
	else {
		tb = (q_0 - q_f + consV * t_f) / consV;
	}

	float tftb = t_f - tb;
	//ACCELERATION
	float acc = consV / tb;

	// INITIALIZATION

	RowVectorXd  q_traj(n_time);
	RowVectorXd  dq_traj(n_time);
	RowVectorXd  ddq_traj(n_time);

	std::cout << "lim1 " << lim1V << std::endl;
	std::cout << "lim2 " << lim2V << std::endl;

	if (consV < lim1V || consV > lim2V)
	{
		std::cout << "Speed " << consV << std::endl;
		std::cout << "Constant speed out of boundaries" << std::endl;
		exit(EXIT_SUCCESS);
	}
	else {
		std::cout << "OK " << std::endl;
		for (int i = 0; i < n_time; i++) {
			time = t_0 + i * sampling_time;

			//FIRST TRAJECTORY ACCELERATION
			if (time <= tb) {
				q_traj(i) = q_0 + acc * pow(time, 2) / 2;
				dq_traj(i) = acc * time;
				ddq_traj(i) = acc;
			}
			// SECOND TRAJECTORY V CONSTANT
			else if (time <= t_f - tb) {
				q_traj(i) = (q_f + q_0 - consV * t_f) / 2 + consV * time;
				dq_traj(i) = consV;
				ddq_traj(i) = 0;
			}
			// THIRD TRAJECTORY DECCELERATION
			else {
				q_traj(i) = q_f - (acc * pow(t_f, 2) / 2) + acc * t_f * time - (acc / 2) * pow(time, 2);
				dq_traj(i) = acc * t_f - acc * time;
				ddq_traj(i) = -acc;
			}
		}

		std::vector<float> q_tr(q_traj.data(), q_traj.data() + q_traj.size());
		std::vector<float> dq_tr(dq_traj.data(), dq_traj.data() + dq_traj.size());
		std::vector<float> ddq_tr(ddq_traj.data(), ddq_traj.data() + ddq_traj.size());

		if (DEBUG)
		{

			std::cout << "The traj is:\n" << q_traj << std::endl;
			std::cout << "The dtraj is:\n" << dq_traj << std::endl;
			std::cout << "The ddtraj is:\n" << ddq_traj << std::endl;
		}

		return std::make_tuple(q_tr, dq_tr, ddq_tr);
	}

}