#include "DynamicCompesator.h"
#include "TX90Dynamics.h"


MatrixXd DynamicCompesator::getMassMatrix()
{
  return compute_fregMM(actualPos[1], actualPos[2], actualPos[3], actualPos[4], actualPos[5]);
}

MatrixXd  DynamicCompesator::getDynamicComp()
{
 return compute_fregCentrCorG(actualVel[1], actualVel[1], actualVel[2], actualVel[3], actualVel[4], actualVel[5], actualPos[1], actualPos[2], actualPos[3], actualPos[4], actualPos[5]);
}

MatrixXd DynamicCompesator::getFric()
{
  return compute_fregFric(actualVel[1], actualVel[1], actualVel[2], actualVel[3], actualVel[4], actualVel[5]);
}
