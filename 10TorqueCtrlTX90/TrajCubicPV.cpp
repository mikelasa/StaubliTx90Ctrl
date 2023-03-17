#include "TrajCubicPV.h"
#include <iostream>
#include "Eigen/Dense"

using namespace std;
using namespace Eigen;


std::tuple <std::vector<float>, std::vector<float>> solveTrajCubic(float q_0, float q_f, float dq_0, float dq_f, float t_0, float t_f, float sampling_time, bool DEBUG);


TrajCubicPV::TrajCubicPV(float posI, float posF, float timeF, float samplingTime): TrajPV(samplingTime)
{
	std::tuple <std::vector<float>, std::vector<float>> traj = solveTrajCubic(posI, posF, 0, 0, 0, timeF, samplingTime, 0);
	qc = get<0>(traj);
	dqc = get<1>(traj);
}


std::tuple <std::vector<float>, std::vector<float>> solveTrajCubic(float q_0, float q_f, float dq_0, float dq_f, float t_0, float t_f, float sampling_time, bool DEBUG)
{
	Matrix4d A;
	Vector4d b;

	int n_time = ceil((t_f - t_0) / sampling_time);
	float time = n_time;


	A << 1, t_0, pow(t_0, 2), pow(t_0, 3),
		0, 1, 2 * t_0, 3 * pow(t_0, 2),
		1, t_f, pow(t_f, 2), pow(t_f, 3),
		0, 1, 2 * t_f, 3 * pow(t_f, 2),

		b << q_0, dq_0, q_f, dq_f;

	VectorXd a = A.colPivHouseholderQr().solve(b);


	RowVectorXd  q_traj(n_time);
	RowVectorXd  dq_traj(n_time);
	//q_traj(1) < 10;
	int cont = 0;
	for (int i = 0; i < n_time; i++)
	{
		cont = cont + 1;
		time = t_0 + i * sampling_time;
		//std::cout << "a:\n" << a(0) + a(1) * time + a(2) * pow(time, 2) + a(3) * pow(time, 3) << std::endl;
		q_traj(i) = a(0) + a(1) * time + a(2) * pow(time, 2) + a(3) * pow(time, 3);
		//q_traj(0)  = a(0) + a(1) * time + a(2) * pow(time, 2) + a(3) * pow(time, 3);
		dq_traj(i) = a(1) + 2 * a(2) * time + 3 * a(3) * pow(time, 2);


	}
	std::vector<float> q_tr(q_traj.data(), q_traj.data() + q_traj.size());
	std::vector<float> dq_tr(dq_traj.data(), dq_traj.data() + dq_traj.size());

	if (DEBUG)
	{
		std::cout << "Here is the matrix A:\n" << A << std::endl;
		std::cout << "Here is the vector b:\n" << b << std::endl;
		std::cout << "The solution is:\n" << a << std::endl;

		std::cout << "The traj is:\n" << q_traj << std::endl;
		std::cout << "The dtraj is:\n" << dq_traj << std::endl;
	}

	return std::make_tuple(q_tr, dq_tr);
}