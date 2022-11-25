/*! @file LegController.h
 *  @brief Common Leg Control Interface and Leg Control Algorithms
 */

#ifndef PROJECT_LEG_CONTROLLER_H
#define PROJECT_LEG_CONTROLLER_H
#include "MathTypes.h"

struct LegControllerCommand {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LegControllerCommand() { zero(); }
    void zero(){
        tauFeedForward = Vec3<double>::Zero();
        forceFeedForward = Vec3<double>::Zero();
        qDes = Vec3<double>::Zero();
        qdDes = Vec3<double>::Zero();
        pDes = Vec3<double>::Zero();
        vDes = Vec3<double>::Zero();
        kpCartesian = Mat3<double>::Zero();
        kdCartesian = Mat3<double>::Zero();
        kpJoint = Mat3<double>::Zero();
        kdJoint = Mat3<double>::Zero();
    }
    Vec3<double> tauFeedForward, forceFeedForward, qDes, qdDes, pDes, vDes, tauApplied;
    Mat3<double> kpCartesian, kdCartesian, kpJoint, kdJoint;
};

struct LegControllerData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LegControllerData() { zero(); }

    void zero(){
        q = Vec3<double>::Zero();
        qd = Vec3<double>::Zero();
        p = Vec3<double>::Zero();
        v = Vec3<double>::Zero();
        J = Mat3<double>::Zero();
    }
    Vec3<double> q, qd, p, v;
    Mat3<double> J;
};

class LegController {
public:
    LegController(){
        zeroCommand();
        zeroData();
    }

    void zeroCommand(){
        for (auto& cmd : commands) {
            cmd.zero();
        }
    }

    void zeroData(){
        for (auto& data : datas) {
            data.zero();
        }
    }
    LegControllerCommand commands[4];
    LegControllerData datas[4];
};

#endif  // PROJECT_LEG_CONTROLLER_H
