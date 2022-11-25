/*!
 * @file FootSwingTrajectory.cpp
 * @brief Utility to generate foot swing trajectories.
 *
 * We don't uses Bezier curves here but cycloid curves here
 */

#include "Interpolation.h"
#include "FootSwingTrajectory.h"

/*!
 * Compute foot swing trajectory with a bezier curve
 * @param phase : How far along we are in the swing (0 to 1)
 * @param swingTime : How long the swing should take (seconds)
 */
void FootSwingTrajectory::computeSwingTrajectoryBezier(double phase, double swingTime) {
    _p = Interpolate::cubicBezier<Vec3<double>>(_p0, _pf, phase);
    _v = Interpolate::cubicBezierFirstDerivative<Vec3<double>>(_p0, _pf, phase) / swingTime;
    _a = Interpolate::cubicBezierSecondDerivative<Vec3<double>>(_p0, _pf, phase) / (swingTime * swingTime);

    double zp, zv, za;

    if (phase < 0.5) {
        zp = Interpolate::cubicBezier<double>(_p0[2], _p0[2] + _height, phase * 2);
        zv = Interpolate::cubicBezierFirstDerivative<double>(_p0[2], _p0[2] + _height, phase * 2) * 2 / swingTime;
        za = Interpolate::cubicBezierSecondDerivative<double>(_p0[2], _p0[2] + _height, phase * 2) * 4 / (swingTime * swingTime);
    } else {
        zp = Interpolate::cubicBezier<double>(_p0[2] + _height, _pf[2], phase * 2 - 1);
        zv = Interpolate::cubicBezierFirstDerivative<double>(_p0[2] + _height, _pf[2], phase * 2 - 1) * 2 / swingTime;
        za = Interpolate::cubicBezierSecondDerivative<double>(_p0[2] + _height, _pf[2], phase * 2 - 1) * 4 / (swingTime * swingTime);
    }

    _p[2] = zp;
    _v[2] = zv;
    _a[2] = za;
}

void FootSwingTrajectory::computeSwingTrajectoryCycloid(double phase, double swingTime) {
    _pf[2] = _p0[2] + _height;
    _p = Interpolate::cycloid(_p0, _pf, phase);
    _v = Interpolate::cycloidFirstDerivative(_p0, _pf, phase) / swingTime;
    _a = Interpolate::cycloidSecondDerivative(_p0, _pf, phase) / (swingTime * swingTime);
};
