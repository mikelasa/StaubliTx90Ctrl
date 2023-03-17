#pragma once

#include "VirtualTrajTypes.h"


  class TrajCubicPV: public TrajPV
  {
  protected:
    std::vector<float> qc;
    std::vector<float> dqc;

  public:
    TrajCubicPV(float posI, float posF, float timeF, float samplingTime);
    float getqc(int i) { return qc[i]; }
    float  getdqc(int i) { return dqc[i]; }
    int getNumSamples() { return qc.size(); }

  };

