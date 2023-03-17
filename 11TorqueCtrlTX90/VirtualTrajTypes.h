#pragma once
#include <vector>

class TrajP
{
protected:
  float samplingTime;

public:
  TrajP(float samplingTime) { this->samplingTime = samplingTime; };
  float getSampligTime() { return samplingTime; }
  virtual float getqc(int i) = 0;
  virtual int getNumSamples() = 0;
};


class TrajPV: public TrajP
{

public:
  TrajPV(float samplingTime): TrajP(samplingTime){}
  virtual float getdqc(int i) = 0;
};


class TrajPVA : public TrajPV
{

public:
  TrajPVA(float samplingTime) : TrajPV(samplingTime) {}
  virtual float getddqc(int i) = 0;
};


