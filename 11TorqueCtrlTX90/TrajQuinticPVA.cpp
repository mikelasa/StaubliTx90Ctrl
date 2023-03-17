#include "TrajQuinticPVA.h"

//#define _USE_MATH_DEFINES

#include <iostream>
#include "Eigen/Dense"

using namespace std;
using namespace Eigen;

std::tuple <std::vector<float>, std::vector<float>, std::vector<float>> solveTrajQuinticPTP(float q_0, float q_f, float dq_0, float dq_f, float ddq_0, float ddq_f, float t_0, float t_f, float samplingTime, bool DEBUG);

TrajQuinticPVA::TrajQuinticPVA(float posI, float posF, float timeF, float samplingTime): TrajPVA(samplingTime)
{
	std::tuple <std::vector<float>, std::vector<float>, std::vector<float>>  traj = solveTrajQuinticPTP(posI, posF, 0, 0, 0, 0, 0, timeF, samplingTime, 0);
	qc = get<0>(traj);
	dqc = get<1>(traj);
	ddqc = get<2>(traj);
}

std::tuple <std::vector<float>, std::vector<float>, std::vector<float>> solveTrajQuinticPTP(float q_0, float q_f, float dq_0, float dq_f, float ddq_0, float ddq_f, float t_0, float t_f, float samplingTime, bool DEBUG)

{
	MatrixXd A(6, 6);
	VectorXd b(6);

	int n_time = ceil((t_f - t_0) / samplingTime);
	float time = n_time;


	A << 1, t_0, pow(t_0, 2), pow(t_0, 3), pow(t_0, 4), pow(t_0, 5),
		0, 1, 2 * t_0, 3 * pow(t_0, 2), 4 * pow(t_0, 3), 5 * pow(t_0, 4),
		0, 0, 2, 6 * t_0, 12 * pow(t_0, 2), 20 * pow(t_0, 3),
		1, t_f, pow(t_f, 2), pow(t_f, 3), pow(t_f, 4), pow(t_f, 5),
		0, 1, 2 * t_f, 3 * pow(t_f, 2), 4 * pow(t_f, 3), 5 * pow(t_f, 4),
		0, 0, 2, 6 * t_f, 12 * pow(t_f, 2), 20 * pow(t_f, 3),

		b << q_0, dq_0, ddq_0, q_f, dq_f, ddq_f;

	VectorXd a = A.colPivHouseholderQr().solve(b);


	RowVectorXd  q_traj(n_time);
	RowVectorXd  dq_traj(n_time);
	RowVectorXd  ddq_traj(n_time);

	int cont = 0;
	for (int i = 0; i < n_time; i++)
	{
		cont = cont + 1;
		time = t_0 + i * samplingTime;

		q_traj(i) = a(0) + a(1) * time + a(2) * pow(time, 2) + a(3) * pow(time, 3) + a(4) * pow(time, 4) + a(5) * pow(time, 5);
		dq_traj(i) = a(1) + 2 * a(2) * time + 3 * a(3) * pow(time, 2) + 4 * a(4) * pow(time, 3) + 5 * a(5) * pow(time, 4);
		ddq_traj(i) = 2 * a(2) + 6 * a(3) * time + 12 * a(4) * pow(time, 2) + 20 * a(5) * pow(time, 3);

	}

	std::vector<float> q_tr(q_traj.data(), q_traj.data() + q_traj.size());
	std::vector<float> dq_tr(dq_traj.data(), dq_traj.data() + dq_traj.size());
	std::vector<float> ddq_tr(ddq_traj.data(), ddq_traj.data() + ddq_traj.size());

	if (DEBUG)
	{
		std::cout << "Here is the matrix A:\n" << A << std::endl;
		std::cout << "Here is the vector b:\n" << b << std::endl;
		std::cout << "The solution is:\n" << a << std::endl;

		std::cout << "The traj is:\n" << q_traj << std::endl;
		std::cout << "The dtraj is:\n" << dq_traj << std::endl;
	}

	return std::make_tuple(q_tr, dq_tr, ddq_tr);
}

