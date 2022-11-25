/*! origin from Cheetah-Software
 * @modified by HuNing-He
 * @date 2022-5-12
 * @file FootSwingTrajectory.h
 * @brief Utility to generate foot swing trajectories.Support Bezier curves and Composite Cycloid curves
 */

#ifndef CHEETAH_SOFTWARE_FOOT_SWING_TRAJECTORY_H
#define CHEETAH_SOFTWARE_FOOT_SWING_TRAJECTORY_H

#include "MathTypes.h"

/*!
 * A foot swing trajectory for a single foot
 */
class FootSwingTrajectory {
public:

    /*!
     * Construct a new foot swing trajectory with everything set to zero
     */
    FootSwingTrajectory() {
        _p0.setZero();
        _pf.setZero();
        _p.setZero();
        _v.setZero();
        _a.setZero();
        _height = 0;
    }

    /*!
     * Set the starting location of the foot
     * @param p0 : the initial foot position
     */
    void setInitialPosition(Vec3<double> p0) {
        _p0 = p0;
    }

    /*!
     * Set the desired final position of the foot
     * @param pf : the final foot position
     */
    void setFinalPosition(Vec3<double> pf) {
        _pf = pf;
    }

    /*!
     * Set the maximum height of the swing
     * attention that cycloid curve doesn't need to set height, the _pf[2] should be desire height
     * @param h : the maximum height of the swing, achieved halfway through the swing
     */
    void setHeight(double h) {
        _height = h;
    }

    void computeSwingTrajectoryBezier(double phase, double swingTime);

    void computeSwingTrajectoryCycloid(double phase, double swingTime);

    /*!
     * Get the foot position at the current point along the swing
     * @return : the foot position
     */
    Vec3<double> getPosition() {
        return _p;
    }

    /*!
     * Get the foot velocity at the current point along the swing
     * @return : the foot velocity
     */
    Vec3<double> getVelocity() {
        return _v;
    }

    /*!
     * Get the foot acceleration at the current point along the swing
     * @return : the foot acceleration
     */
    Vec3<double> getAcceleration() {
        return _a;
    }

private:
    Vec3<double> _p0, _pf, _p, _v, _a;
    double _height;
};


#endif //CHEETAH_SOFTWARE_FOOT_SWING_TRAJECTORY_H
