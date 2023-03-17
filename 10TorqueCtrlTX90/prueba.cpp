#include "TX90Dynamics.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <math.h>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <sstream>
#include <string>
#include <queue>
#include <chrono>
#include <thread>
#include <vector>


using namespace std;
using namespace Eigen;

int main() {

  std::vector<double> robotpos = { 0.0,	0.0,	0.0,	0.0,	0.0,	0.0 };
  std::vector<double> robotvel = { 0.0,	0.0,	0.0,	0.0,	0.0,	0.0 };

  MatrixXd centr = MatrixXd::Zero(6, 6);
  MatrixXd cor = MatrixXd::Zero(6, 15);
  MatrixXd fric = MatrixXd::Zero(6, 1);
  MatrixXd g = MatrixXd::Zero(6, 1);
  MatrixXd jacobian = MatrixXd::Zero(6, 6);
  MatrixXd centrCor = MatrixXd::Zero(6, 1);
  MatrixXd centrCorG = MatrixXd::Zero(6, 1);
  MatrixXd massMatrix = MatrixXd::Zero(6, 6);


  fric = compute_fregFric(robotvel[0], robotvel[1], robotvel[2], robotvel[3], robotvel[4], robotvel[5]);
  centr = compute_fregCentr(robotpos[1], robotpos[2], robotpos[3], robotpos[4], robotpos[5]);
  cor = compute_fregCor(robotpos[1], robotpos[2], robotpos[3], robotpos[4], robotpos[5]);
  g = compute_fregG(robotpos[1], robotpos[2], robotpos[3], robotpos[4], robotpos[5]);
  //jacobian = compute_Jacobian
  centrCor = compute_fregCentrCor(robotvel[0], robotvel[1], robotvel[2], robotvel[3], robotvel[4], robotvel[5], robotpos[1], robotpos[2], robotpos[3], robotpos[4], robotpos[5]);
  centrCorG = compute_fregCentrCorG(robotvel[0], robotvel[1], robotvel[2], robotvel[3], robotvel[4], robotvel[5], robotpos[1], robotpos[2], robotpos[3], robotpos[4], robotpos[5]);
  massMatrix = compute_fregMM(robotpos[1], robotpos[2], robotpos[3], robotpos[4], robotpos[5]);


  //std::cout << "Here is the matrix Fric:\n" << centrCorG << std::endl;
  std::cout << "Here is the matrix Fric:\n" << massMatrix << std::endl;


}