#ifndef MAIN_CPP_ROBOT_CONTROL_PARAMETERS_H
#define MAIN_CPP_ROBOT_CONTROL_PARAMETERS_H
#include "MathTypes.h"

struct RobotControlParameters{
    unsigned int control_mode;
    /* begin of wbc task parameters */
    Vec3<double> Kp_body;
    Vec3<double> Kd_body;
    Vec3<double> Kp_ori;
    Vec3<double> Kd_ori;
    Vec3<double> Kp_foot;
    Vec3<double> Kd_foot;
    Vec3<double> Kp_joint;  // 关节PD控制
    Vec3<double> Kd_joint;
    /* end of wbc task parameters */

    /* only for FSM_Stand_Up */
    Vec3<double> kp_stand_up;
    Vec3<double> kd_stand_up;
    double body_height;
    /* only for FSM_Stand_Up */

    bool use_wbc;
    bool use_rf_mpc;
    // for ConvexMPC
    double cmpc_x_drag;
    Vec3<double> kp_swing_mpc;
    Vec3<double> kd_swing_mpc;
    Vec3<double> kp_stand_mpc;
    Vec3<double> kd_stand_mpc;

    Eigen::Matrix<double, 12, 1> mpc_weights;
    Eigen::Matrix<double, 12, 1> rf_mpc_weights;
    Eigen::Matrix<double, 12, 1> rf_mpc_terminal_weights;
    Eigen::Vector3d rf_mpc_input_weights;
    Eigen::Vector3d rf_mpc_foot_pos_weights;

    Eigen::Matrix<double, 4, 3> des_foot_hold;

    RobotControlParameters() {
        cmpc_x_drag = 3;
        body_height = 0.28;
        use_wbc = false;
        use_rf_mpc = false;
        control_mode = 0;

        Kp_body << 100, 100, 100;
        Kd_body << 10, 10, 20;
        Kp_ori << 120, 100, 100;
        Kd_ori << 10, 12, 16;

        Kp_foot << 500, 500, 500;
        Kd_foot << 100, 100, 60;
        Kp_joint << 5, 5, 5;
        Kd_joint << 1.5, 1.5, 1.5;

        kp_stand_up << 850, 850, 1050;
        kd_stand_up << 60, 60, 80;

        kp_stand_mpc << 0, 0, 0;
        kd_stand_mpc << 7, 7, 7;

        kp_swing_mpc << 700, 700, 150;
        kd_swing_mpc << 7, 7, 7;

        mpc_weights << 2, 2, 10, 2, 2, 80, 0, 0, 0.3, 0.2, 0.2, 0.1;

        rf_mpc_input_weights << 0.1, 0.1, 0.2;
        rf_mpc_terminal_weights << 1e5, 2e5, 4e6, 5e2, 1e3, 1e3, 8e5, 1e3, 800, 140, 40, 10;
        rf_mpc_foot_pos_weights << 4e6, 4e6, 20;
        rf_mpc_weights = rf_mpc_terminal_weights;

        des_foot_hold.setZero();
    };
};

#endif //MAIN_CPP_ROBOT_CONTROL_PARAMETERS_H
