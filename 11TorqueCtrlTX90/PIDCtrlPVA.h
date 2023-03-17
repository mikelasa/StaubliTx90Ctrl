#pragma once
#include "VirtualTrajTypes.h"
#include <ostream>


class PIDCtrlPVA
{
protected:
  TrajPVA* pTraj;
  float Kp, Ki, Kv;
  float ErrorAnt, ErrorInt;
 

public:
  PIDCtrlPVA(TrajPVA* pT , float kp, float ki, float kv): pTraj(pT), ErrorAnt(0), ErrorInt(0), Kp(kp), Ki(ki), Kv(kv) {}
  ~PIDCtrlPVA() { delete  pTraj; }
  float getPidCommand(int nIter, float ActualPosition, float ActualVelocity);
  int getTrajSize() { return pTraj->getNumSamples(); }
  TrajPVA& getTrajPVA() { return *pTraj; }
};

