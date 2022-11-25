#include "Gait.h"

OffsetDurationGait::OffsetDurationGait(int nSegment, Vec4<int> offsets, Vec4<int> durations, const std::string &name) :
        _offsets(offsets.array()),
        _durations(durations.array()),
        _nIterations(nSegment) {
    _name = name;
    _mpc_table = Eigen::MatrixXi::Zero(nSegment, 4);
    _offsetsFloat = offsets.cast<double>() / (double) nSegment;
    _durationsFloat = durations.cast<double>() / (double) nSegment;
    _stance = durations[0];
    _swing = nSegment - durations[0];
}

Vec4<double> OffsetDurationGait::getContactState() {
    Array4d progress = _phase - _offsetsFloat;
    for (int i = 0; i < 4; i++) {
        if (progress[i] < 0) progress[i] += 1.;
        if (progress[i] > _durationsFloat[i]) {
            progress[i] = 0.;
        } else {
            progress[i] = progress[i] / _durationsFloat[i];
        }
    }
    return progress.matrix();
}

Vec4<double> OffsetDurationGait::getSwingState() {
    Array4d swing_offset = _offsetsFloat + _durationsFloat;
    for (int i = 0; i < 4; i++)
        if (swing_offset[i] > 1) swing_offset[i] -= 1.;
    Array4d swing_duration = 1. - _durationsFloat;

    Array4d progress = _phase - swing_offset;

    for (int i = 0; i < 4; i++) {
        if (progress[i] < 0) progress[i] += 1.f;
        if (progress[i] > swing_duration[i]) {
            progress[i] = 0.;
        } else {
            progress[i] = progress[i] / swing_duration[i];
        }
    }
    return progress.matrix();
}
#include "iostream"
Eigen::MatrixXi &OffsetDurationGait::getMpcTable() {
    for (int i = 0; i < _nIterations; i++) {
        int iter = (i + _iteration + 1) % _nIterations;
        Array4i progress = iter - _offsets;
        for (int j = 0; j < 4; j++) {
            if (progress[j] < 0) progress[j] += _nIterations;
            if (progress[j] < _durations[j])
                _mpc_table(i, j) = 1;
            else
                _mpc_table(i, j) = 0;
        }
    }
//    std::cout << "mpc:\n" << _mpc_table << std::endl;
    return _mpc_table;
}

void OffsetDurationGait::setIterations(int iterationsPerMPC, unsigned long long int currentIteration) {
    _iteration = (currentIteration / iterationsPerMPC) % _nIterations;
    _phase = (double) (currentIteration % (iterationsPerMPC * _nIterations)) / (double) (iterationsPerMPC * _nIterations);
}

int OffsetDurationGait::getCurrentGaitPhase() {
    return _iteration;
}

double OffsetDurationGait::getCurrentSwingTime(double dtMPC, int leg) {
    (void) leg;
    return dtMPC * _swing;
}

double OffsetDurationGait::getCurrentStanceTime(double dtMPC, int leg) {
    (void) leg;
    return dtMPC * _stance;
}
