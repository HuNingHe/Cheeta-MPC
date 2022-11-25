#ifndef CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H
#define CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H

#include <FootSwingTrajectory.h>
#include <ControlFSMData.h>
#include "MathTypes.h"
#include "Gait.h"
#include <cstdio>
#include "MPC/include/ConvexMPC.h"
#include "MPC/include/RF_MPC.h"

class MPCLocomotion {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MPCLocomotion(double _dt, int _iterations_between_mpc, ControlFSMData *data);
    ~MPCLocomotion(){
        delete rf_mpc;
        delete convex_mpc;
    }
    void initialize();
    void run(ControlFSMData &data);
    Vec3<double> pBody_des;
    Vec3<double> vBody_des;
    Vec3<double> aBody_des;
    Vec3<double> pBody_RPY_des;
    Vec3<double> vBody_Ori_des;
    Vec3<double> pFoot_des[4];
    Vec3<double> vFoot_des[4];
    Vec3<double> aFoot_des[4];
    Vec3<double> Fr_des[4];
    Vec4<double> contact_state;

private:
    void SetupCommand(ControlFSMData &data);
    LinearMPC *mpc_solver;
    RF_MPC *rf_mpc;
    ConvexMPC *convex_mpc;

    double _yaw_turn_rate;
    double _yaw_des;
    double _roll_des;
    double _pitch_des;
    double _x_vel_des = 0.;
    double _y_vel_des = 0.;
    double _body_height = 0.29;

    void recompute_timing(int iterations_per_mpc);
    void updateMPCIfNeeded(Eigen::MatrixXi &mpcTable, ControlFSMData &data);

    int iterationsBetweenMPC;
    int horizonLength;
    int default_iterations_between_mpc;
    double dt;
    double dtMPC;
    unsigned long long int iterationCounter = 0;
    Vec3<double> f_ff[4];
    Vec4<double> swingTimes;
    FootSwingTrajectory footSwingTrajectories[4];
    OffsetDurationGait trotting, bounding, pronking, galloping, standing, trotRunning, walking, pacing;
    Mat3<double> Kp, Kd, Kp_stance, Kd_stance;
    bool firstRun = true;
    bool firstSwing[4];
    double swingTimeRemaining[4];
    int current_gait;
    int gaitNumber;
    double stand_traj[6];

    Vec3<double> world_position_desired;
    Vec3<double> rpy_int;
    Vec3<double> rpy_comp;
    double x_comp_integral = 0;
    Vec3<double> pFoot[4];
};


#endif //CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H
