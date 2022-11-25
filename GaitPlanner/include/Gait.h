#ifndef PROJECT_GAIT_H
#define PROJECT_GAIT_H

#include <string>
#include <queue>
#include "MathTypes.h"

class Gait {
public:
  virtual ~Gait() = default;
  virtual Vec4<double> getContactState() = 0;
  virtual Vec4<double> getSwingState() = 0;
  virtual Eigen::MatrixXi &getMpcTable() = 0;
  virtual void setIterations(int iterationsBetweenMPC, unsigned long long int currentIteration) = 0;
  virtual double getCurrentStanceTime(double dtMPC, int leg) = 0;
  virtual double getCurrentSwingTime(double dtMPC, int leg) = 0;
  virtual int getCurrentGaitPhase() = 0;

protected:
  std::string _name;
};

using Eigen::Array4d;
using Eigen::Array4i;

class OffsetDurationGait : public Gait {
public:
  OffsetDurationGait(int nSegment, Vec4<int> offset, Vec4<int> durations, const std::string& name);
  Vec4<double> getContactState() override;
  Vec4<double> getSwingState() override;
  Eigen::MatrixXi &getMpcTable() override;
  void setIterations(int iterationsBetweenMPC, unsigned long long int currentIteration) override;
  double getCurrentStanceTime(double dtMPC, int leg) override;
  double getCurrentSwingTime(double dtMPC, int leg) override;
  int getCurrentGaitPhase() override;

private:
  Eigen::MatrixXi _mpc_table;
  Array4i _offsets; // offset in mpc segments
  Array4i _durations; // duration of step in mpc segments
  Array4d _offsetsFloat; // offsets in phase (0 to 1)
  Array4d _durationsFloat; // durations in phase (0 to 1)
  int _stance;
  int _swing;
  int _iteration;
  int _nIterations;
  double _phase;
};

#endif //PROJECT_GAIT_H
