#include <Eigen/Geometry>
#include <eigen3/unsupported/Eigen/KroneckerProduct>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include "MPC/include/RF_MPC.h"
#include "Orientation.h"
#include "Timer.h"
#include <iostream>
#include <utility>

// Attention that qpSWIFT solve time is 1.5ms on average,
// but qpOASES takes average 5.5ms on my laptop with same predict horizon.

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

RF_MPC::RF_MPC(double mass, const Eigen::Matrix3d& inertial, int num_legs, int planning_horizon, double time_step,
               Matrix<double, 12, 1> &state_weights, Matrix<double, 12, 1> &terminal_weights,
               Vector3d &input_weights, VectorXd friction_coeff, double decay_rate)
        : LinearMPC(planning_horizon, time_step, decay_rate, QP_SOLVER::QP_SWIFT),
          mass_(mass),
          inertial_(inertial),
          inv_inertial_(inertial.inverse()),
          num_legs_(num_legs),
          mu_(std::move(friction_coeff)){
    assert(mass > 0 && num_legs > 0 && time_step > 0);
    assert(planning_horizon <= 10 && planning_horizon >= 4); // not suggest predict too long for RF-MPC
    assert(friction_coeff.rows() == num_legs);
    // check that if this matrix is definitely positive
    Vector3d eigen_values = inertial.eigenvalues().real();
    for (int i = 0; i < 3; ++i) {
        assert(eigen_values[i] > 0);
    }
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

    qp_solution_ = VectorXd::Zero(3 * num_legs);

    A_ = MatrixXd::Zero(12, 12);
    B_ = MatrixXd::Zero(12, 3 * num_legs);
    d_ = VectorXd::Zero(12);

    init_inputs_ = VectorXd::Zero(3 * num_legs);
    init_states_ = VectorXd::Zero(18 + 3 * num_legs);
    des_inputs_ = VectorXd::Zero(planning_horizon * 3 * num_legs);
    des_states_ = VectorXd::Zero(planning_horizon * 18);

    states_weights_ = state_weights;
    input_weights_ = input_weights;
    terminal_weights_ = terminal_weights;

    H_ = RowMajorMatrixXd::Zero((num_legs * 3 + 12) * planning_horizon, (num_legs * 3 + 12) * planning_horizon);
    g_ = VectorXd::Zero((num_legs * 3 + 12) * planning_horizon);

    Aeq_ = RowMajorMatrixXd::Zero(12 * planning_horizon, (num_legs * 3 + 12) * planning_horizon);
    beq_ = VectorXd::Zero(12 * planning_horizon);
    if (qp_solver_ == QP_SOLVER::QP_SWIFT) {
        Aineq_ = RowMajorMatrixXd::Zero(6 * num_legs * planning_horizon, (num_legs * 3 + 12) * planning_horizon);
        ubA_ = VectorXd::Zero(6 * num_legs * planning_horizon);
    } else {
        Aineq_ = RowMajorMatrixXd::Zero((5 * num_legs + 12) * planning_horizon, (num_legs * 3 + 12) * planning_horizon); // this contains equality constraints
        ubA_ = VectorXd::Zero((5 * num_legs + 12) * planning_horizon);
        lbA_ = VectorXd::Zero((5 * num_legs + 12) * planning_horizon);
    }
}

void RF_MPC::UpdateSystemMatrices(){
    // note that the xi means \xi in latex. if you are familiar with Doctor Ding's paper, it's easy to understand.
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
    Matrix<double, 9, 1> stacked_vec(tmp.data());
    Matrix<double, 9, 1> Cxi_c =  stacked_vec - kronIRopN * w_op_; // actually this is always zeros

    MatrixXd foot_to_com_cross = MatrixXd::Zero(3, 3 * num_legs_);
    MatrixXd Cv_u = MatrixXd::Zero(3, 3 * num_legs_);
    Vector3d sum_force_vec = Vector3d::Zero();
    Vector3d pf;
    for (int i = 0; i < num_legs_; ++i) {
        sum_force_vec += Eigen::Vector3d(f_op_[i * 3], f_op_[i * 3 + 1], f_op_[i * 3 + 2]);
        pf << foot_pos_op_(i * 3) - body_pos_op_(0), foot_pos_op_(i * 3 + 1) - body_pos_op_(1), foot_pos_op_(i * 3 + 2) - body_pos_op_(2);
        foot_to_com_cross.block(0, i * 3, 3, 3) = Ori::crossMatrix(pf);
        Cv_u.block(0, i * 3, 3, 3) = (time_step_ / mass_) * Matrix3d::Identity();
    }

    Matrix3d Cw_p = R_op_.transpose() * Ori::crossMatrix(sum_force_vec);
    Vector3d total_tau_op = foot_to_com_cross * f_op_;
    Matrix3d tmp_J_w = Ori::crossMatrix(inertial_ * w_op_) - Ori::crossMatrix(w_op_) * inertial_;

    Matrix3d Cw_xi = Ori::crossMatrix(total_tau_op) - tmp_J_w * Ori::crossMatrix(w_op_);
    Vector3d Cw_c = R_op_.transpose() * total_tau_op - Ori::crossMatrix(inertial_ * w_op_) * w_op_ - Cw_p * body_pos_op_;

    // 1. Calculate the linearized state space matrix
    A_.block(0, 0, 3, 3) = Matrix3d::Identity();
    A_.block(0, 3, 3, 3) = Matrix3d::Identity() * time_step_;
    A_.block(3, 3, 3, 3) = Matrix3d::Identity();
    A_.block(6, 6, 3, 3) = Matrix3d::Identity() + pre_cal * Cxi_xi;
    A_.block(6, 9, 3, 3) = pre_cal * kronIRopN;
    A_.block(9, 0, 3, 3) = time_step_ * inv_inertial_ * Cw_p;
    A_.block(9, 6, 3, 3) = time_step_ * inv_inertial_ * Cw_xi;
    A_.block(9, 9, 3, 3) = Matrix3d::Identity() + time_step_ * inv_inertial_ * tmp_J_w;

    MatrixXd Cw_u = MatrixXd::Zero(3, num_legs_ * 3);
    Cw_u = time_step_ * inv_inertial_ * R_op_.transpose() * foot_to_com_cross;
    B_.block(3, 0, 3, num_legs_ * 3) = Cv_u;
    B_.block(9, 0, 3, num_legs_ * 3) = Cw_u;

    d_.segment(3, 3) = Cv_u * f_op_ + Vector3d(0, 0, -9.81 * time_step_);
    d_.segment(6, 3) = pre_cal * Cxi_c;
    d_.segment(9, 3) = time_step_ * inv_inertial_ * Cw_c;
}

void RF_MPC::UpdateQPMatrices(){
    Vector3d body_pos_op_ = init_states_.segment(0, 3);               // body position at operate point
    Vector3d body_vel_op_ = init_states_.segment(3, 3);               // body velocity at operate point
    Matrix<double, 9, 1> r_op_vec = init_states_.segment(6, 9);
    Matrix3d R_op_ = Map<Matrix3d>(r_op_vec.data());                  // rotation matrix at operate point
    Vector3d w_op_ = init_states_.segment(15, 3);                     // angular velocity in body frame at operate point
    VectorXd f_op_ = init_inputs_;                                    // contact force at operate point

    // 2. Convert to QP format
    DiagonalMatrix<double, 3> Qx, Qv, Qxi, Qw, Qtx, Qtv, Qtxi, Qtw;// note that t means terminal here
    DiagonalMatrix<double, 12> Q, Qt;
    DiagonalMatrix<double, Eigen::Dynamic> Qu;
    Qu.diagonal() = input_weights_.replicate(num_legs_, 1);

    Q.diagonal() = states_weights_;
    Qx.diagonal() = states_weights_.segment(0, 3);
    Qv.diagonal() = states_weights_.segment(3, 3);
    Qxi.diagonal() = states_weights_.segment(6, 3);
    Qw.diagonal() = states_weights_.segment(9, 3);

    Qt.diagonal() = terminal_weights_;
    Qtx.diagonal() = terminal_weights_.segment(0, 3);
    Qtv.diagonal() = terminal_weights_.segment(3, 3);
    Qtxi.diagonal() = terminal_weights_.segment(6, 3);
    Qtw.diagonal() = terminal_weights_.segment(9, 3);

    // Note that MatrixLogarithmReturnValue can't convert to MatrixBase in release mode(debug mode can).
    // So, we don't pass a MatrixLogarithmReturnValue value to ori::skewMatToVec(),
    // convert to MatrixBase derived type first.
    Eigen::Matrix<double, 12, 1> x0;
    Matrix3d Rl = R_op_.log();
    Vector3d vecRl = Ori::skewMatToVec(Rl);
    x0 << body_pos_op_, body_vel_op_, vecRl, w_op_;
    // decision variable for qp is [u0, x1, u1, x2 ... u(N-1), xN, pf1, pf2, pf3, pf4]
    for (int i = 0; i < predict_horizon_; ++i) {
        Vector3d des_body_pos_k = des_states_.segment(i * 18, 3);               // desire body position at k-th horizon
        Vector3d des_body_vel_k = des_states_.segment(i * 18 + 3, 3);           // desire body velocity at k-th horizon
        Matrix<double, 9, 1> des_r_vec = des_states_.segment(i * 18 + 6, 9);
        Matrix3d Rdk = Eigen::Map<Matrix3d>(des_r_vec.data());                                    // desire rotation matrix at k-th horizon
        Vector3d wdk = des_states_.segment(i * 18 + 15, 3);                     // desire angular velocity in body frame at k-th horizon

        Matrix3d Re = Rdk.transpose() * R_op_;
        Matrix3d Rel = Re.log();
        if (i != predict_horizon_ - 1) {
            H_.block((3 * num_legs_ + 12) * i + num_legs_ * 3, (3 * num_legs_ + 12) * i + num_legs_ * 3, 12, 12) = Q * pow(decay_rate_, i);
            g_.segment((3 * num_legs_ + 12) * i + num_legs_ * 3, 3) = Qx * (-des_body_pos_k.segment(i * 3, 3)) * pow(decay_rate_, i);
            g_.segment((3 * num_legs_ + 12) * i + num_legs_ * 3 + 3, 3) = Qv * (-des_body_vel_k.segment(i * 3, 3)) * pow(decay_rate_, i);
            g_.segment((3 * num_legs_ + 12) * i + num_legs_ * 3 + 6, 3) = Qxi * Ori::skewMatToVec(Rel) * pow(decay_rate_, i);
            g_.segment((3 * num_legs_ + 12) * i + num_legs_ * 3 + 9, 3) = Qw * (-wdk.segment(i * 3, 3)) * pow(decay_rate_, i);
        } else {
            H_.block((3 * num_legs_ + 12) * i + num_legs_ * 3, (3 * num_legs_ + 12) * i + num_legs_ * 3, 12, 12) = Qt * pow(decay_rate_, i);
            g_.segment((3 * num_legs_ + 12) * i + num_legs_ * 3, 3) = Qtx * (-des_body_pos_k.segment(i * 3, 3)) * pow(decay_rate_, i);
            g_.segment((3 * num_legs_ + 12) * i + num_legs_ * 3 + 3, 3) = Qtv * (-des_body_vel_k.segment(i * 3, 3)) * pow(decay_rate_, i);
            g_.segment((3 * num_legs_ + 12) * i + num_legs_ * 3 + 6, 3) = Qtxi * Ori::skewMatToVec(Rel) * pow(decay_rate_, i);
            g_.segment((3 * num_legs_ + 12) * i + num_legs_ * 3 + 9, 3) = Qtw * (-wdk.segment(i * 3, 3)) * pow(decay_rate_, i);
        } // test multiply wd_ with R_op_.transpose() * Rdk *
        H_.block((3 * num_legs_ + 12) * i, (3 * num_legs_ + 12) * i, num_legs_ * 3, num_legs_ * 3) = Qu * pow(decay_rate_, i);
        g_.segment((3 * num_legs_ + 12) * i, num_legs_ * 3) = Qu * (f_op_ - des_inputs_.segment(i * num_legs_ * 3, num_legs_ * 3)) * pow(decay_rate_, i);

        if (i == 0) {
            beq_.segment(0, 12) = A_ * x0 + d_;
        } else {
            beq_.segment(i * 12, 12) = d_;
            Aeq_.block(i * 12, (i - 1) * (12 + num_legs_ * 3) + num_legs_ * 3, 12, 12) = -A_;
        }
        Aeq_.block(i * 12, (12 + num_legs_ * 3) * i, 12, num_legs_ * 3) = -B_;
        Aeq_.block(i * 12, (12 + num_legs_ * 3) * i + num_legs_ * 3, 12, 12) = Matrix<double, 12, 12>::Identity();

        if (qp_solver_ == QP_SOLVER::QP_SWIFT) {
            MatrixXd Aineq_four_leg = MatrixXd::Zero(num_legs_ * 6, num_legs_ * 3);
            VectorXd bineq_four_leg = VectorXd::Zero(num_legs_ * 6);
            Matrix<double, 6, 3> Aineq_unit;
            Matrix<double, 6, 1> bineq_unit;

            for (int j = 0; j < num_legs_; ++j) {
                Aineq_unit << 1, 0, -mu_(j),
                              -1, 0, -mu_(j),
                              0, 1, -mu_(j),
                              0, -1, -mu_(j),
                              0, 0, 1,
                              0, 0, -1;

                bineq_unit << mu_(j) * f_op_(j * 3 + 2) - f_op_(j * 3),
                              mu_(j) * f_op_(j * 3 + 2) + f_op_(j * 3),
                              mu_(j) * f_op_(j * 3 + 2) - f_op_(j * 3 + 1),
                              mu_(j) * f_op_(j * 3 + 2) + f_op_(j * 3 + 1),
                              -f_op_(j * 3 + 2) + 3 * des_inputs_(num_legs_ * 3 * i + j * 3 + 2),
                              f_op_(j * 3 + 2);

                Aineq_four_leg.block(j * 6, j * 3, 6, 3) = Aineq_unit;
                bineq_four_leg.segment(j * 6, 6) = bineq_unit;
            }
            Aineq_.block(i * 6 * num_legs_, (12 + num_legs_ * 3) * i, 6 * num_legs_, 3 * num_legs_) = Aineq_four_leg;
            ubA_.segment(i * 6 * num_legs_, num_legs_ * 6) = bineq_four_leg;
        }else {
            MatrixXd Aineq_four_leg = MatrixXd::Zero(num_legs_ * 5, num_legs_ * 3);
            VectorXd blb_four_leg = VectorXd::Zero(num_legs_ * 5);
            VectorXd bub_four_leg = VectorXd::Zero(num_legs_ * 5);

            Matrix<double, 5, 3> Aineq_unit;
            Matrix<double, 5, 1> blb_unit;
            Matrix<double, 5, 1> bub_unit;

            for (int j = 0; j < num_legs_; ++j) {
                Aineq_unit << -1, 0, mu_(j),
                              1, 0, mu_(j),
                              0, -1, mu_(j),
                              0, 1, mu_(j),
                              0, 0, 1;

                blb_unit << -mu_(j) * f_op_(j * 3 + 2) + f_op_(j * 3),
                            -mu_(j) * f_op_(j * 3 + 2) - f_op_(j * 3),
                            -mu_(j) * f_op_(j * 3 + 2) + f_op_(j * 3 + 1),
                            -mu_(j) * f_op_(j * 3 + 2) - f_op_(j * 3 + 1),
                            -f_op_(j * 3 + 2);

                bub_unit << 5000, // represent infinity
                            5000,
                            5000,
                            5000,
                            -f_op_(j * 3 + 2) + 3 * des_inputs_(num_legs_ * 3 * i + j * 3 + 2);

                Aineq_four_leg.block(j * 5, j * 3, 5, 3) = Aineq_unit;
                blb_four_leg.segment(j * 5, 5) = blb_unit;
                bub_four_leg.segment(j * 5, 5) = bub_unit;
            }
            Aineq_.block(i * 5 * num_legs_, (12 + num_legs_ * 3) * i, 5 * num_legs_, 3 * num_legs_) = Aineq_four_leg;
            ubA_.segment(i * 5 * num_legs_, num_legs_ * 5) = bub_four_leg;
            lbA_.segment(i * 5 * num_legs_, num_legs_ * 5) = blb_four_leg;
        }
    }

    if (qp_solver_ == QP_SOLVER::QP_OASES) {
        Aineq_.block(5 * num_legs_ * predict_horizon_, 0, 12 * predict_horizon_, (12 + num_legs_ * 3) * predict_horizon_) = Aeq_;
        ubA_.segment(5 * num_legs_ * predict_horizon_, 12 * predict_horizon_) = beq_;
        lbA_.segment(5 * num_legs_ * predict_horizon_, 12 * predict_horizon_) = beq_;
    }
}

VectorXd &RF_MPC::UpdateMPC(const Eigen::VectorXd &state, const Eigen::VectorXd &des_state, const Eigen::VectorXd &des_inputs) {
    // Attention that this rpy and des_rpy are subtly different with ConvexMPC.
    // Pitch angle here can be in range [-pi, pi]. So, acrobatic movement can be achieved. This is determined by SO3 and this is also a main feature of RF-MPC.
    // So, RF-MPC can handle singular pos.
    Vector3d body_ori = state.segment(0, 3);
    const Quaterniond body_rotation = AngleAxisd(body_ori[2], Vector3d::UnitZ()) *
                                      AngleAxisd(body_ori[1], Vector3d::UnitY()) *
                                      AngleAxisd(body_ori[0], Vector3d::UnitX());
    Matrix3d rotation = body_rotation.matrix();
    Matrix<double, 9, 1> rotation_vec = Eigen::Map<Matrix<double, 9, 1>>(rotation.data());
    init_states_.segment(0, 3) = state.segment(3, 3); // body pos
    init_states_.segment(3, 3) = state.segment(9, 3); // body vel
    init_states_.segment(6, 9) = rotation_vec;        // body rotation
    init_states_.segment(15, 3) = state.segment(6, 3);// body angular vel
    init_states_.segment(18, 3 * num_legs_) = state.segment(12, 3 * num_legs_);
    MatrixXi foot_contact_states = MatrixXi::Zero(predict_horizon_, num_legs_);
    des_inputs_.setZero();
    for (int i = 0; i < predict_horizon_; ++i) {
        for (int j = 0; j < num_legs_; ++j) {
            foot_contact_states(i, j) = des_inputs[i * num_legs_ + j];
        }
    }

    for (int i = 0; i < predict_horizon_; ++i) {
        Vector3d pos_d(des_state[3] + des_state[9] * time_step_ * (i + 1), des_state[4] + des_state[10] * time_step_ * (i + 1), des_state[5]);
        Vector3d vel_d(des_state[9], des_state[10], 0);
        Vector3d eu_d(des_state[0], des_state[1], des_state[2] + des_state[8] * time_step_ * (i + 1));
        Matrix3d Rdk = Ori::crossMatrix(eu_d).exp();
        Vector3d wd(0, 0, des_state[8]);

        des_states_.segment(i * 18, 3) = pos_d;
        des_states_.segment(i * 18 + 3, 3) = vel_d;
        des_states_.segment(i * 18 + 6, 9) = Map<Matrix<double, 9, 1>>(Rdk.data());
        des_states_.segment(i * 18 + 15, 3) = wd;

        if (foot_contact_states.row(i).sum() > 0) {
            for (int j = 0; j < num_legs_; ++j) {
                des_inputs_(i * num_legs_ * 3 + j * 3 + 2) = foot_contact_states(i, j) * mass_ * 9.81 / foot_contact_states.row(i).sum();
            }
        }
    }

    UpdateSystemMatrices();
    UpdateQPMatrices();
    bool verbose = true;
    if (qp_solver_ == QP_SOLVER::QP_SWIFT) {
        // Note that we don't use sparse MPC here. Sparse MPC should faster than Dense MPC. You can make this conversion by eigen.
        QP *myQP = QP_SETUP_dense((num_legs_ * 3 + 12) * predict_horizon_,
                                  6 * num_legs_ * predict_horizon_, 12 * predict_horizon_,
                                  H_.data(), Aeq_.data(), Aineq_.data(), g_.data(),
                                  ubA_.data(), beq_.data(), nullptr, ROW_MAJOR_ORDERING);
        qp_int ExitCode = QP_SOLVE(myQP);

        for (int j = 0; j < 3 * num_legs_; ++j) {
            qp_solution_[j] = myQP->x[j];
        }

        if (verbose) {
            if (ExitCode == QP_OPTIMAL) {
                printf("Setup Time     : %f ms\n", myQP->stats->tsetup * 1000.0);
                printf("Solve Time     : %f ms\n", (myQP->stats->tsolve + myQP->stats->tsetup) * 1000.0);
                printf("Iterations     : %ld\n", myQP->stats->IterationCount);
                std::cout << "Optimal Solution Found by qpSWIFT:\n" << qp_solution_ << std::endl;
            }
            else {
                printf("Optimal Solution Not Found, ExitCode: %ld\n", ExitCode); // ExitCode = 2 means max iteration reached
                std::cout << g_ << std::endl;
            }
        }
        QP_CLEANUP_dense(myQP);

    } else {
        Timer qpOASES_time;
        // Test by HuNing-He(2022-7-10),It seems that qpOASES is much slower than qpSWIFT in dealing with equation constraints
        auto qp_problem = qpOASES::QProblem((num_legs_ * 3 + 12) * predict_horizon_, (num_legs_ * 5 + 12) * predict_horizon_);
        qpOASES::Options options;
        options.setToMPC();
        options.printLevel = qpOASES::PL_NONE;
        qp_problem.setOptions(options);
        int max_solver_iter = 100;
        Timer qpOASES_timer;
        qp_problem.init(H_.data(), g_.data(), Aineq_.data(), nullptr, nullptr, lbA_.data(), ubA_.data(), max_solver_iter);
        VectorXd qp_sol = VectorXd::Zero((num_legs_ * 3 + 12) * predict_horizon_);
        qp_problem.getPrimalSolution(qp_sol.data());
        for (int j = 0; j < 3 * num_legs_; ++j) {
            qp_solution_[j] = qp_sol[j];
        }
        if (verbose) {
            std::cout << "Optimal Solution Found by qpOASES:\n" << qp_sol.segment(0, num_legs_ * 3) << std::endl;
            std::cout << "qpOASES solve time:\t" << qpOASES_timer.getMs() << " ms" << std::endl;
        }
    }
    init_inputs_ += qp_solution_;
    return init_inputs_;
}
