#include "PIDCtrlPV.h"

#include "Eigen/Dense"

using namespace std;
using namespace Eigen;

std::tuple <float, float> PIDCtrlPV::getPidCommand(int nIter, float ActualPosition, float ActualVelocity)
{

	float PIDOUT;
	float errorPos, errorVel, P, D, I;

	errorPos = pTraj->getqc(nIter) - ActualPosition;
	errorVel = pTraj->getdqc(nIter) - ActualVelocity;

	// proportional gain
	P = errorPos * Kp;

	// derivative gain
	D = ((errorPos - ErrorAnt) / pTraj->getSampligTime()) * Kv;

	//integral gain
	ErrorInt += errorPos;
	I = ErrorInt * Ki;

	// PID OUTPUT
	PIDOUT = P + D + I;

	// update variables
	ErrorAnt = errorPos;

	return std::make_tuple(PIDOUT, errorVel);
}
