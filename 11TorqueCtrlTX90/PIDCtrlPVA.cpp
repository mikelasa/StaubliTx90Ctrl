#include "PIDCtrlPVA.h"

#include "Eigen/Dense"

using namespace std;
using namespace Eigen;

float PIDCtrlPVA::getPidCommand(int nIter, float ActualPosition, float ActualVelocity)
{

	float PIDOUT;
	float errorPos, errorVel, trajAcc, P, D, I;


	errorPos = pTraj->getqc(nIter) - ActualPosition;
	errorVel = pTraj->getdqc(nIter) - ActualVelocity;
	trajAcc = pTraj->getddqc(nIter);

	// proportional gain
	P = errorPos * Kp;

	// derivative gain
	D = ((errorPos - ErrorAnt) / pTraj->getSampligTime()) * Kv;

	//integral gain
	ErrorInt += errorPos;
	I = ErrorInt * Ki;

	// PID OUTPUT
	PIDOUT = trajAcc + P + D + I;

	// update variables
	ErrorAnt = errorPos;

	return PIDOUT;
}
