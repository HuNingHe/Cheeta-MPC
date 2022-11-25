#include "MPC/include/StepAdaption.h"
#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/KroneckerProduct>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <vector>
#include "Orientation.h"

using Eigen::VectorXi;
using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::MatrixXi;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix;
using Eigen::DiagonalMatrix;
using Eigen::kroneckerProduct;
using Eigen::Map;

SARF_MPC::SARF_MPC(double mass, Matrix3d inertia, int num_legs, int planning_horizon, double time_step, VectorXd &friction_coeff,
                   Matrix<double, 15, 1> &state_weights, Matrix<double, 15, 1> &terminal_weights,
                   Vector3d &input_weights, Vector3d& delta_foot_weights, double decay_rate)
        : LinearMPC(planning_horizon, time_step, decay_rate),
          mass_(mass),
          inertia_(std::move(inertia)),
          inv_inertia_(inertia.inverse()),
          num_legs_(num_legs) {
    assert(mass > 0 && num_legs > 0 && planning_horizon > 0);
    assert(planning_horizon < 10 && planning_horizon >= 4); // not suggest predict too long for RF-MPC
    assert(time_step > 0);
    assert(friction_coeff.rows() == num_legs);

    // fixed size for system states matrices
    A_ = MatrixXd::Zero(12 + num_legs * 3, 12 + num_legs * 3);
    B_ = MatrixXd::Zero(12 + num_legs * 3, num_legs * 3);
    d_ = VectorXd::Zero(12 + num_legs * 3);

    init_inputs_ = Eigen::VectorXd::Zero(num_legs * 3);
    init_states_ = Eigen::VectorXd::Zero(18 + num_legs * 3);

    states_weights_ = state_weights;
    terminal_weights_ = terminal_weights;
    input_weights_ = input_weights;
    delta_foot_weights_ = delta_foot_weights;

    // two step-attempts at most
    H_ = RowMajorMatrixXd::Zero((num_legs * 6 + 12) * planning_horizon + num_legs * 6, (num_legs * 6 + 12) * planning_horizon + num_legs * 6);
    g_ = VectorXd::Zero((num_legs * 6 + 12) * planning_horizon + num_legs * 6);

    Aeq_ = RowMajorMatrixXd::Zero((num_legs * 3 + 12) * planning_horizon, (num_legs * 6 + 12) * planning_horizon + num_legs * 6);
    beq_ = VectorXd::Zero((num_legs * 3 + 12) * planning_horizon);

    Aineq_ = RowMajorMatrixXd::Zero(num_legs * 18 * planning_horizon, (num_legs * 6 + 12) * planning_horizon + num_legs * 6);
    lbA_ = VectorXd::Zero(num_legs * 18 * planning_horizon);
    ubA_ = VectorXd::Zero(num_legs * 18 * planning_horizon);

    friction_coeff_ = friction_coeff;
    step_attempts_ = MatrixXi::Zero(predict_horizon_, num_legs_);

    N_ << 0, 0, 0,
          0, 0, 1,
          0, -1, 0,
          0, 0, -1,
          0, 0, 0,
          1, 0, 0,
          0, 1, 0,
          -1, 0, 0,
          0, 0, 0;

    inv_N_ << 0, 0, 0, 0, 0, 0.5, 0, -0.5, 0,
              0, 0, -0.5, 0, 0, 0, 0.5, 0, 0,
              0, 0.5, 0, -0.5, 0, 0, 0, 0, 0;
}

void SARF_MPC::checkStepAttempt(const MatrixXi &mpcTable) {
    assert(num_legs_ == mpcTable.cols());
    assert(predict_horizon_ == mpcTable.rows());

    for (int i = 0; i < num_legs_; ++i) {
        for (int j = 1; j < predict_horizon_; ++j) {
            if (mpcTable(j, i) == 1 && mpcTable(j - 1, i) == 0) {
                int count = 0;
                for (int k = j; k < predict_horizon_; ++k) {
                    if (mpcTable(k, i) == 0){
                        break;
                    }
                    count++;
                }
                step_attempts_(j, i) = count;
            }
        }
    }

    for (int i = 0; i < num_legs_; ++i) {
        int count = 0;
        for (int j = 1; j < predict_horizon_; ++j) {
            if (step_attempts_(j, i) != 0){
                count++;
            }
        }
        step_attempts_(0, i) = count;
    }
}

void SARF_MPC::UpdateSystemMatrices() {
    // system variable is [p \dot{p} \xi w]
    // Note that the xi means \xi in latex. if you are familiar with Doctor Ding's paper, it's easy to understand.
    // All state with op as subscripts represent the operating point state.
    Vector3d body_pos_op_ = init_states_.segment(0, 3);               // body position at operate point
    Vector3d body_vel_op_ = init_states_.segment(3, 3);               // body velocity at operate point
    Matrix<double, 9, 1> r_op_vec = init_states_.segment(6, 9);
    Matrix3d R_op_ = Map<Matrix3d>(r_op_vec.data());                  // rotation matrix at operate point
    Vector3d w_op_ = init_states_.segment(15, 3);                     // angular velocity in body frame at operate point
    VectorXd foot_pos_op_ = init_states_.segment(18, num_legs_ * 3);  // foot position in world frame at operate point
    VectorXd f_op_ = init_inputs_;                                    // contact force at operate point

    Matrix<double, 9, 3> kronIRopN = kroneckerProduct(Matrix3d::Identity(), R_op_) * N_;
    Matrix<double, 9, 3> Cxi_xi = kroneckerProduct(Matrix3d::Identity(), R_op_ * Ori::crossMatrix(w_op_)) * N_ - kronIRopN * Ori::crossMatrix(w_op_);
    Matrix<double, 3, 9> pre_cal = inv_N_ * time_step_ * kroneckerProduct(Matrix3d::Identity(), R_op_.transpose()); // It is reused later to avoid double calculation

    Matrix3d tmp = R_op_ * Ori::crossMatrix(w_op_);
    Matrix<double, 9, 1> stacked_vec = Map<Matrix<double, 9, 1>>(tmp.data());
    Matrix<double, 9, 1> Cxi_c =  stacked_vec - kronIRopN * w_op_; // actually this is always zeros

    MatrixXd foot_to_com_cross = MatrixXd::Zero(3, 3 * num_legs_);
    MatrixXd Cv_u = MatrixXd::Zero(3, 3 * num_legs_);
    MatrixXd force_op_cross = MatrixXd::Zero(3, 3 * num_legs_);
    Vector3d sum_force_vec = Vector3d::Zero();
    Vector3d pf;
    Vector3d single_force = Vector3d::Zero();
    for (int i = 0; i < num_legs_; ++i) {
        single_force << f_op_[i * 3], f_op_[i * 3 + 1], f_op_[i * 3 + 2];
        sum_force_vec += single_force;
        pf << foot_pos_op_(i * 3) - body_pos_op_(0), foot_pos_op_(i * 3 + 1) - body_pos_op_(1), foot_pos_op_(i * 3 + 2) - body_pos_op_(2);
        foot_to_com_cross.block(0, i * 3, 3, 3) = Ori::crossMatrix(pf);
        Cv_u.block(0, i * 3, 3, 3) = (time_step_ / mass_) * Matrix3d::Identity();
        force_op_cross.block(0, i * 3, 3, 3) = Ori::crossMatrix(single_force);
    }

    Matrix3d Cw_p = R_op_.transpose() * Ori::crossMatrix(sum_force_vec);
    MatrixXd Cw_pf = MatrixXd::Zero(3, 3 * num_legs_);
    Cw_pf = R_op_.transpose() * force_op_cross;
    Vector3d total_tau_op = foot_to_com_cross * f_op_;
    Matrix3d tmp_J_w = Ori::crossMatrix(inertia_ * w_op_) - Ori::crossMatrix(w_op_) * inertia_;

    Matrix3d Cw_xi = Ori::crossMatrix(total_tau_op) - tmp_J_w * Ori::crossMatrix(w_op_);
    Vector3d Cw_c = R_op_.transpose() * total_tau_op - Ori::crossMatrix(inertia_ * w_op_) * w_op_ - Cw_p * body_pos_op_ + Cw_pf * foot_pos_op_;

    A_.block(0, 0, 3, 3) = Matrix3d::Identity();
    A_.block(0, 3, 3, 3) = Matrix3d::Identity() * time_step_;
    A_.block(3, 3, 3, 3) = Matrix3d::Identity();
    A_.block(6, 6, 3, 3) = Matrix3d::Identity() + pre_cal * Cxi_xi;
    A_.block(6, 9, 3, 3) = pre_cal * kronIRopN;
    A_.block(9, 0, 3, 3) = time_step_ * inv_inertia_ * Cw_p;
    A_.block(9, 6, 3, 3) = time_step_ * inv_inertia_ * Cw_xi;
    A_.block(9, 9, 3, 3) = Matrix3d::Identity() + time_step_ * inv_inertia_ * tmp_J_w;
    A_.block(9, 12, 3, num_legs_ * 3) = -time_step_ * inv_inertia_ * Cw_pf;
    A_.block(12, 12, num_legs_ * 3, num_legs_ * 3) = MatrixXd::Identity(num_legs_ * 3, num_legs_ * 3);

    MatrixXd Cw_u = MatrixXd::Zero(3, num_legs_ * 3);
    Cw_u = time_step_ * inv_inertia_ * R_op_.transpose() * foot_to_com_cross;
    B_.block(3, 0, 3, num_legs_ * 3) = Cv_u;
    B_.block(9, 0, 3, num_legs_ * 3) = Cw_u;

    d_.segment(3, 3) = Cv_u * f_op_ + Vector3d(0, 0, -9.81 * time_step_);
    d_.segment(6, 3) = pre_cal * Cxi_c;
    d_.segment(9, 3) = time_step_ * inv_inertia_ * Cw_c;
}

void SARF_MPC::UpdateQPMatrices() {
    Vector3d body_pos_op_ = init_states_.segment(0, 3);               // body position at operate point
    Vector3d body_vel_op_ = init_states_.segment(3, 3);               // body velocity at operate point
    Matrix<double, 9, 1> r_op_vec = init_states_.segment(6, 9);
    Matrix3d R_op_ = Eigen::Map<Matrix3d>(r_op_vec.data());           // rotation matrix at operate point
    Vector3d w_op_ = init_states_.segment(15, 3);                     // angular velocity in body frame at operate point
    VectorXd foot_pos_op_ = init_states_.segment(18, num_legs_ * 3);  // foot position in world frame at operate point
    VectorXd f_op_ = init_inputs_;                                    // contact force at operate point

    DiagonalMatrix<double, 3> Qx, Qv, Qxi, Qw, Qpf, Qtx, Qtv, Qtxi, Qtw, Qtpf;// note that t means terminal here
    DiagonalMatrix<double, Eigen::Dynamic> Q, Qt, Qu;
    Q.resize(12 + num_legs_ * 3);
    Qt.resize(12 + num_legs_ * 3);
    Qu.resize(num_legs_ * 3);

    Eigen::VectorXd weights = VectorXd::Zero(12 + num_legs_ * 3);
    weights << states_weights_, states_weights_.segment(12, 3).replicate(3, 1);
    Eigen::VectorXd t_weights = VectorXd::Zero(12 + num_legs_ * 3);
    t_weights << terminal_weights_, terminal_weights_.segment(12, 3).replicate(3, 1);

    Q.diagonal() = weights;
    Qx.diagonal() = states_weights_.segment(0, 3);
    Qv.diagonal() = states_weights_.segment(3, 3);
    Qxi.diagonal() = states_weights_.segment(6, 3);
    Qw.diagonal() = states_weights_.segment(9, 3);
    Qtpf.diagonal() = states_weights_.segment(12, 3);

    Qt.diagonal() = t_weights;
    Qtx.diagonal() = terminal_weights_.segment(0, 3);
    Qtv.diagonal() = terminal_weights_.segment(3, 3);
    Qtxi.diagonal() = terminal_weights_.segment(6, 3);
    Qtw.diagonal() = terminal_weights_.segment(9, 3);
    Qtpf.diagonal() = terminal_weights_.segment(12, 3);

    Qu.diagonal() = input_weights_.replicate(num_legs_, 1);
    // Note that MatrixLogarithmReturnValue can't convert to MatrixBase in release mode(debug mode can).
    // So, we don't pass a MatrixLogarithmReturnValue value to ori::skewMatToVec(),
    // convert to MatrixBase derived type first for safety.
    Matrix3d Re, Rel;
    VectorXd x0 = VectorXd::Zero(12 + num_legs_ * 3);
    Matrix3d Rl = R_op_.log();
    Vector3d vecRl = Ori::skewMatToVec(Rl);
    x0 << body_pos_op_, body_vel_op_, vecRl, w_op_, foot_pos_op_;
    // decision variable for qp is [u0, x1, u1, x2 ... u(N-1), xN, pf1, pf2, pf3, pf4]
    for (int i = 0; i < predict_horizon_; ++i) {
        Vector3d des_body_pos_k = des_states_.segment(i * (18 + num_legs_ * 3), 3);               // desire body position at k-th horizon
        Vector3d des_body_vel_k = des_states_.segment(i * (18 + num_legs_ * 3) + 3, 3);           // desire body velocity at k-th horizon
        Matrix<double, 9, 1> des_r_vec = des_states_.segment(i * (18 + num_legs_ * 3) + 6, 9);
        Matrix3d Rdk = Eigen::Map<Matrix3d>(des_r_vec.data());                                    // desire rotation matrix at k-th horizon
        Vector3d wdk = des_states_.segment(i * (18 + num_legs_ * 3) + 15, 3);                     // desire angular velocity in body frame at k-th horizon
        VectorXd des_pfk = des_states_.segment(i * (18 + num_legs_ * 3) + 18, num_legs_ * 3);     // desire foot position in world frame at k-th horizon

        Re = Rdk.transpose() * R_op_;
        Rel = Re.log();

        if (i != predict_horizon_ - 1) {
            H_.block((6 * num_legs_ + 12) * i + num_legs_ * 3,
                     (6 * num_legs_ + 12) * i + num_legs_ * 3,
                     num_legs_ * 3 + 12, num_legs_ * 3 + 12) = Q * pow(decay_rate_, i);

            g_.segment((6 * num_legs_ + 12) * i + num_legs_ * 3, 3) = Qx * (-des_body_pos_k) * pow(decay_rate_, i);
            g_.segment((6 * num_legs_ + 12) * i + num_legs_ * 3 + 3, 3) = Qv * (-des_body_vel_k) * pow(decay_rate_, i);
            g_.segment((6 * num_legs_ + 12) * i + num_legs_ * 3 + 6, 3) = Qxi * Ori::skewMatToVec(Rel) * pow(decay_rate_, i);
            g_.segment((6 * num_legs_ + 12) * i + num_legs_ * 3 + 9, 3) = Qw * (-wdk) * pow(decay_rate_, i);
            g_.segment((6 * num_legs_ + 12) * i + num_legs_ * 3 + 12, 3 * num_legs_) = Qpf * (-des_pfk) * pow(decay_rate_, i);
        } else {
            H_.block((6 * num_legs_ + 12) * i + num_legs_ * 3,
                     (6 * num_legs_ + 12) * i + num_legs_ * 3,
                     num_legs_ * 3 + 12, num_legs_ * 3 + 12) = Qt * pow(decay_rate_, i);

            g_.segment((6 * num_legs_ + 12) * i + num_legs_ * 3, 3) = Qtx * (-des_body_pos_k) * pow(decay_rate_, i);
            g_.segment((6 * num_legs_ + 12) * i + num_legs_ * 3 + 3, 3) = Qtv * (-des_body_vel_k) * pow(decay_rate_, i);
            g_.segment((6 * num_legs_ + 12) * i + num_legs_ * 3 + 6, 3) = Qtxi * Ori::skewMatToVec(Rel) * pow(decay_rate_, i);
            g_.segment((6 * num_legs_ + 12) * i + num_legs_ * 3 + 9, 3) = Qtw * (-wdk) * pow(decay_rate_, i);
            g_.segment((6 * num_legs_ + 12) * i + num_legs_ * 3 + 12, 3 * num_legs_) = Qtpf * (-des_pfk) * pow(decay_rate_, i);
        } // test multiply wdk with R_op_.transpose() * Rdk *
        H_.block((6 * num_legs_ + 12) * i, (6 * num_legs_ + 12) * i, num_legs_ * 3, num_legs_ * 3) = Qu * pow(decay_rate_, i);
        g_.segment((6 * num_legs_ + 12) * i, num_legs_ * 3) = Qu * (f_op_ - des_inputs_.segment(i * num_legs_ * 3, num_legs_ * 3)) * pow(decay_rate_, i);

        if (i == 0) {
            beq_.segment(0, 12 + num_legs_ * 3) = A_ * x0 + d_;
        } else {
            beq_.segment(i * (12 + num_legs_ * 3), 12 + num_legs_ * 3) = d_;
            Aeq_.block(i * (12 + num_legs_ * 3), (i - 1) * (12 + num_legs_ * 6) + num_legs_ * 3, 12 + num_legs_ * 3, 12 + num_legs_ * 3) = -A_;
        }
        Aeq_.block(i * (12 + num_legs_ * 3), (12 + num_legs_ * 6) * i, (12 + num_legs_ * 3), num_legs_ * 3) = -B_;
        Aeq_.block(i * (12 + num_legs_ * 3), (12 + num_legs_ * 6) * i + num_legs_ * 3,
                   12 + num_legs_ * 3, 12 + num_legs_ * 3) = MatrixXd::Identity(12 + num_legs_ * 3, 12 + num_legs_ * 3);

        MatrixXd Aineq_cone = MatrixXd::Zero(num_legs_ * 6, num_legs_ * 3);
        VectorXd bineq_cone = VectorXd::Zero(num_legs_ * 6);
        MatrixXd Aineq_pf_ = MatrixXd::Zero(num_legs_ * 6, num_legs_ * 3);
        VectorXd bineq_pf = VectorXd::Zero(num_legs_ * 6);

        Matrix<double, 6, 3> Aineq_unit;
        Matrix<double, 6, 1> bineq_unit;

        double max_x_foot = 0.35;
        double max_y_foot = 0.35;

        for (int j = 0; j < num_legs_; ++j) {
            Aineq_unit << 1, 0, -friction_coeff_(j),
                          -1, 0, -friction_coeff_(j),
                          0, 1, -friction_coeff_(j),
                          0, -1, -friction_coeff_(j),
                          0, 0, 1,
                          0, 0, -1;

            bineq_unit << friction_coeff_(j) * f_op_(j * 3 + 2) - f_op_(j * 3),
                          friction_coeff_(j) * f_op_(j * 3 + 2) + f_op_(j * 3),
                          friction_coeff_(j) * f_op_(j * 3 + 2) - f_op_(j * 3 + 1),
                          friction_coeff_(j) * f_op_(j * 3 + 2) + f_op_(j * 3 + 1),
                          -f_op_(j * 3 + 2) + 3 * des_inputs_(num_legs_ * 3 * i + j * 3 + 2),
                          f_op_(j * 3 + 2);

            Aineq_cone.block(j * 6, j * 3, 6, 3) = Aineq_unit;
            bineq_cone.segment(j * 6, 6) = bineq_unit;

            Aineq_unit.block(0, 0, 3, 3) = Matrix3d::Identity();
            Aineq_unit.block(3, 0, 3, 3) = -Matrix3d::Identity();
//            bineq_unit << max_x_foot + foot_pos_op_(j * 3), max_y_foot + foot_pos_op_(j * 3 + 1), foot_pos_op_(j * 3 + 2),
//                          max_x_foot - foot_pos_op_(j * 3), max_y_foot - foot_pos_op_(j * 3 + 1), -foot_pos_op_(j * 3 + 2);

            bineq_unit << max_x_foot, max_y_foot, 0,
                          max_x_foot, max_y_foot, 0;

            Aineq_pf_.block(j * 6, j * 3, 6, 3) = Aineq_unit;
            bineq_pf.segment(j * 6, 6) = bineq_unit;

            if (step_attempts_(0, j) != 1) {
                throw std::runtime_error("Too many step attempts in predict horizon");
            } else {

            }
        }
        Aineq_.block(i * 18 * num_legs_, (12 + num_legs_ * 6) * i, 6 * num_legs_, 3 * num_legs_) = Aineq_cone;
        ubA_.segment(i * 18 * num_legs_, num_legs_ * 6) = bineq_cone;
//        Aineq_.block(i * 12 * num_legs_ + 6 * num_legs_,
//                     (12 + num_legs_ * 6) * i + 12 + num_legs_ * 3, 6 * num_legs_, 3 * num_legs_) = Aineq_pf_;
//        ubA_.segment(i * 12 * num_legs_ + 6 * num_legs_, 6 * num_legs_) = bineq_pf;
        Aineq_.block(i * 18 * num_legs_ + 6 * num_legs_,
                     (12 + num_legs_ * 6) * i + 12 + num_legs_ * 3, 6 * num_legs_, 3 * num_legs_) = Aineq_pf_;
        ubA_.segment(i * 18 * num_legs_ + 6 * num_legs_, 6 * num_legs_) = bineq_pf;

    }

//    H_.block((3 * num_legs_ + 12) * predict_horizon_,
//             (3 * num_legs_ + 12) * predict_horizon_,
//             3 * num_legs_, 3 * num_legs_) = foot_weights_;
//    g_.segment((3 * num_legs_ + 12) * predict_horizon_, 3 * num_legs_) = foot_weights_ * (-des_delta_foot_pos_);
//    for (int i = 0; i < num_legs_; ++i) {
//        if (step_horizon_[i] != -1) {
//            for (int j = 0; j < keep_horizons_[i]; ++j) {
//                Aeq_.block(12 * (step_horizon_[i] + j) + 9,
//                              (3 * num_legs_ + 12) * predict_horizon_ + 3 * i,
//                              3, 3) = time_step_ * inv_inertia_ * R_op_.transpose() * force_op_cross.block(0, i * 3, 3, 3);
//            }
//        }
//    }
}

Eigen::VectorXd &SARF_MPC::UpdateMPC(const VectorXd &state, const VectorXd &des_state, const VectorXd &des_inputs) {
    // Attention that this rpy and des_rpy are subtly different with ConvexMPC.
    // Pitch angle here can be in range [-pi, pi]. So, acrobatic movement can be achieved. This is determined by SO3 and this is also a main feature of RF-MPC.
    // So, RF-MPC can handle singular pos.
    /*
    double default_acc = 1;
    double emergent_acc = 2.5;
    // we assume that we have constant acceleration
    double linear_acc[2] = {default_acc, default_acc};    // linear acceleration
    double w0 = sqrt(9.81 / des_com_pos_world[2]);
    double dcm[2];
    dcm[0] = com_pos_world[0] + com_vel_world[0] / w0;
    dcm[1] = com_pos_world[1] + com_vel_world[1] / w0;

    // if dcm is deviate too much, we think the robot had encountered a big disturbance
    for (int i = 0; i < 2; ++i) {
        if (dcm[i] > 2000) { // Todo need some modifications here
            linear_acc[i] = emergent_acc; // increase the acceleration to recovery from disturbance quickly
        }
    }

    w_op_ = angular_vel_body;
    body_pos_op_ = com_pos_world;
    body_vel_op_ = com_vel_world;
    R_op_ = rotation_matrix;
    foot_pos_op_ = foot_positions_world_frame;

    double time;
    int index[2] = {0, 0};
    for (int i = 0; i < 2; ++i) {
        index[i] = ceil(abs(des_com_vel_world(i) - com_vel_world(i) / linear_acc[i]));
        if (des_com_vel_world(i) > com_vel_world(i)) {
            linear_acc[i] = abs(linear_acc[i]);
        } else {
            linear_acc[i] = -abs(linear_acc[i]);
        }
    }

    for (int i = 0; i < predict_horizon_; ++i) {
        time = (i + 1) * time_step_;
        for (int j = 0; j < 2; ++j) {
            if (i <= index[j]) {
                des_body_vel_[i * 3 + j] = com_vel_world(j) + linear_acc[j] * time;
                des_body_pos_[i * 3 + j] = com_vel_world(j) * time + 0.5 * pow(time, 2) * linear_acc[j] + des_com_pos_world[j];
            } else {
                des_body_vel_[i * 3 + j] = des_com_vel_world(j);
                des_body_pos_[i * 3 + j] = (pow(des_com_vel_world(j), 2) - pow(com_vel_world(j), 2)) / (2 * linear_acc[j]) + des_com_vel_world(j) * (i - index[j]) * time_step_ + des_com_pos_world[j];
            }
        }

        des_body_pos_[i * 3 + 2] = des_com_pos_world[2];
        des_body_vel_[i * 3 + 2] = 0;

        Vector3d eu_d(des_rpy[0], des_rpy[1], des_rpy[2] + des_rpy_vel[2] * time);
        Matrix3d Rdk = ori::crossMatrix(eu_d).exp();
//        const Quaterniond des_rotation = AngleAxisd(des_rpy[2] + des_rpy_vel[2] * time, Vector3d::UnitZ()) *
//                                         AngleAxisd(des_rpy[1], Vector3d::UnitY()) *
//                                         AngleAxisd(des_rpy[0], Vector3d::UnitX());
        Rd_.block(i * 3, 0, 3, 3) = Rdk;

        wd_[i * 3] = 0.0;
        wd_[i * 3 + 1] = 0.0;
        wd_[i * 3 + 2] = des_rpy_vel[2];

        if (foot_contact_states.row(i).sum() <= 0) {
            des_input_(i * num_legs_ * 3 + 2) = 0;
            des_input_(i * num_legs_ * 3 + 5) = 0;
            des_input_(i * num_legs_ * 3 + 8) = 0;
            des_input_(i * num_legs_ * 3 + 11) = 0;
        } else {
            for (int j = 0; j < num_legs_; ++j) { // foot_contact_states 0:swing 1:stance
                des_input_(i * num_legs_ * 3 + j * 3 + 2) = foot_contact_states(i, j) * mass_ * 9.81 / foot_contact_states.row(i).sum();
            }
        }
    }

    for (int i = 0; i < num_legs_; ++i) {
        des_delta_foot_pos_.segment(i * 3, 3) = des_foot_pos.row(i) - last_stance_foot_pos.row(i);
    }

//    std::cout << "des foot:\n" << des_foot_pos << std::endl;
//    std::cout << "foot pos op:\n" << foot_pos_op_ << std::endl;
    checkStepAttempt(foot_contact_states);
    UpdateQPMats();
//    std::cout << "Aeq_qp_:\n" << Aeq_qp_ << std::endl;
//    std::cout << "keep horizon:\n" << keep_horizons_ << std::endl;

    init_inputs_ += qp_solution_.segment(0, num_legs_ * 3);
     */
}
