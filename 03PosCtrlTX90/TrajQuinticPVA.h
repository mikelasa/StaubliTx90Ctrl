#pragma once
#include "VirtualTrajTypes.h"


class TrajQuinticPVA: public TrajPVA
{
protected: 
  std::vector<float> qc;
  std::vector<float> dqc;
  std::vector<float> ddqc;

public:
  TrajQuinticPVA(float posI, float posF, float timeF, float samplingTime);
  float getSampligTime() { return samplingTime; }
  float getqc(int i) { return qc[i]; }
  float  getdqc(int i) { return dqc[i]; }
  float  getddqc(int i) { return ddqc[i]; }
  int getNumSamples() { return qc.size(); }
  
};

