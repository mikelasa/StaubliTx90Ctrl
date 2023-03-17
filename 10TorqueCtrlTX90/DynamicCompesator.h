#pragma once

#include <vector>
#include "Eigen/Dense"

using namespace Eigen;

class DynamicCompesator
{
protected:
  std::vector<float> actualPos(6);
  std::vector<float> actualVel(6);
public:
  DynamicCompesator(std::vector<float> pos, std::vector<float> vel) : actualPos(pos), actualVel(vel) { };
  MatrixXd getMassMatrix();
  MatrixXd getDynamicComp();
  MatrixXd getFric();
};

