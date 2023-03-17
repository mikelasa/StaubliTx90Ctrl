#pragma once
#include "VirtualTrajTypes.h"


class PIDCtrlPV
{
protected:
  TrajPV* pTraj;
  float Kp, Ki, Kv;
  float ErrorAnt, ErrorInt;

public:
  PIDCtrlPV(TrajPV* pT , float kp, float ki, float kv): pTraj(pT), ErrorAnt(0), ErrorInt(0), Kp(kp), Ki(ki), Kv(kv) {}
  ~PIDCtrlPV() { delete  pTraj; }
  std::tuple <float, float>  getPidCommand(int nIter, float ActualPosition, float ActualVelocity);
  int getTrajSize() { return pTraj->getNumSamples(); }
};

