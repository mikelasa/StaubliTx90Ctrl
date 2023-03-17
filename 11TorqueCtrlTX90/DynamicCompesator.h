#pragma once

#include <vector>
#include "Eigen/Dense"

using namespace Eigen;

class DynamicCompesator
{
protected:
  std::vector<float> actualPos;
  std::vector<float> actualVel;
public:
  void setPosAndVel(std::vector<float> pos, std::vector<float> vel) { actualPos = pos; actualVel = vel; }
  MatrixXd getMassMatrix();
  MatrixXd getDynamicComp();
  MatrixXd getFric();
};

