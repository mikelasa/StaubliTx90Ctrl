#pragma once

#include "Eigen/Dense"
#include <vector>
#include "TrajQuinticPVA.h"


class HystoricData
{
protected:
  std::vector<std::vector<float>> commandedTorque = std::vector<std::vector<float>>(6);
  std::vector<std::vector<float>> qr = std::vector<std::vector<float>>(6);
  std::vector<std::vector<float>> dqr = std::vector<std::vector<float>>(6);
  std::vector<std::vector<float>> qc = std::vector<std::vector<float>>(6);
  std::vector<std::vector<float>> dqc = std::vector<std::vector<float>>(6);
  std::vector<std::vector<float>> ddqc = std::vector<std::vector<float>>(6);


public: 
  HystoricData();
  void addQr(std::vector<float> posRobot);
  void addDqr(std::vector<float> velRobot);
  void addTorque(Eigen::MatrixXd torque);
  void writeToFile(const char fileName[], TrajPVA& trj);
};

