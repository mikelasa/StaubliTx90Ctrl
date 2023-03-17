#pragma once
#include <vector>
#include <tuple>
#include "StaubliUdpPort.h"

class TX90ProxyByTorque
{
protected:
  StaubliUdpPort port;

  std::vector<float> positions{ 0,0,0,0,0,0 };
  std::vector<float> speeds{ 0,0,0,0,0,0 };
  std::vector<float> torques{ 0,0,0,0,0,0 };
  std::vector<float> commandedTorques{ 0,0,0,0,0,0 };



  float deltaT = 0.004;
  int nIteration = 1;


public:
  TX90ProxyByTorque() { port.open(); };
  ~TX90ProxyByTorque();
  std::vector<float>  getPositions() { return positions; }
  float  getPosition(int i) { return positions[i]; }
  std::vector<float>  getSpeeds() { return speeds; }
  float  getSpeed(int i) { return positions[i]; }
  std::vector<float>  getTorques() { return torques; }
  float  getTorque(int i) { return torques[i]; }
  void sendTorqueCommand(std::vector<float> torqueValue);
  void refresh();
  float sigmoid(float x);

};


