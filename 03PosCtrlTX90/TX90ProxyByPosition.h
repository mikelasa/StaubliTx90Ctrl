#pragma once
#include <vector>
#include <tuple>
#include "StaubliUdpPort.h"

class TX90ProxyByPosition
{
protected:
  StaubliUdpPort port;

  std::vector<float> positions{ 0,0,0,0,0,0 };
  std::vector<float> speeds{ 0,0,0,0,0,0 };
  std::vector<float> commandedPositions{ 0,0,0,0,0,0 };
  std::vector<float> commandedSpeeds{ 0,0,0,0,0,0 };

  float deltaT = 0.004;
  int nIteration = 1;

public:
  TX90ProxyByPosition() { port.open();};
  ~TX90ProxyByPosition();
  std::vector<float>  getPositions() { return positions; }
  float  getPosition(int i) { return positions[i]; }
  std::vector<float>  getSpeeds() { return speeds; }
  float  getSpeed(int i) { return positions[i]; }
  void sendCommand(std::vector<float> speed, std::vector<std::tuple <float, float>> robotCommand);
  void refresh();

};


