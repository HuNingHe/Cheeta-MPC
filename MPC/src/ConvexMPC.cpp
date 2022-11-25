// Test at 2022-6-20 by HuNing-He
// It seems that qpSWIFT is much slower than qpOASES here
// I got the same optimal contact forces, but qpOASES takes 0.5ms at most and qpSWIFT takes 3.2ms at least, so I quit qpSWIFT here.
// For the convenience that you can test by your self.
#include "MPC/include/ConvexMPC.h"
#include "Orientation.h"
#include <unsupported/Eigen/MatrixFunctions>  // used for matrix exponential, this includes many matrix functions like exp()
#include <Eigen/Geometry>
#include <utility>

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

ConvexMPC::ConvexMPC(double mass, const Eigen::Matrix3d& inertial, int num_legs, int planning_horizon, double time_step,
                     Matrix<double, 12, 1> &state_weights, Vector3d &input_weights, Eigen::VectorXd friction_coeff)
    : LinearMPC(planning_horizon, time_step, 1, QP_SOLVER::QP_OASES),
      mass_(mass),
      inertial_(inertial),
      inv_inertial_(inertial.inverse()),
      num_legs_(num_legs),
      mu_(std::move(friction_coeff)),
      A_qp_(13 * planning_horizon, 13),
      B_qp_(13 * planning_horizon, num_legs * 3 * planning_horizon),
      x_drag_(0) {
    assert(mass > 0 && num_legs > 0 && time_step > 0);
    assert(planning_horizon <= 12 && planning_horizon >= 4); // not suggest predict too long for RF-MPC
    assert(friction_coeff.rows() == num_legs);
    Vector3d eigen_values = inertial.eigenvalues().real();
    for (int i = 0; i < 3; ++i) {
        assert(eigen_values[i] > 0);
    }
    // fixed size for system states matrices
    A_ = MatrixXd::Zero(13, 13);
    B_ = MatrixXd::Zero(13, num_legs * 3);

    init_states_ = VectorXd::Zero(13 + num_legs * 3);
    des_states_ = VectorXd::Zero(planning_horizon * 13);
    des_inputs_ = VectorXd::Zero(planning_horizon * 3 * num_legs);

    states_weights_ = state_weights;
    input_weights_ = input_weights;

    H_ = RowMajorMatrixXd::Zero(num_legs * planning_horizon * 3, num_legs * planning_horizon * 3);
    g_ = VectorXd::Zero(num_legs * planning_horizon * 3);

    Aineq_ = RowMajorMatrixXd::Zero(5 * num_legs * planning_horizon, num_legs * 3 * planning_horizon);
    lbA_ = VectorXd::Zero(5 * num_legs * planning_horizon);
    ubA_ = VectorXd::Zero(5 * num_legs * planning_horizon);
    qp_solution_ = VectorXd::Zero(num_legs * 3 * planning_horizon);
    A_qp_.setZero();
    B_qp_.setZero();
}

void ConvexMPC::UpdateSystemMatrices() {
    Vector3d rpy = init_states_.segment(0, 3);
    Vector3d com_pos = init_states_.segment(3, 3);
    Matrix3d angular_vel_to_rpy_rate;
    double cos_yaw = cos(rpy[2]);
    double sin_yaw = sin(rpy[2]);
    double cos_pitch = cos(rpy[1]);
    double tan_pitch = tan(rpy[1]);

    angular_vel_to_rpy_rate << cos_yaw / cos_pitch, sin_yaw / cos_pitch, 0,
                               -sin_yaw, cos_yaw, 0,
                               cos_yaw * tan_pitch, sin_yaw * tan_pitch, 1;

    const Quaterniond com_rotation = AngleAxisd(rpy[2], Vector3d::UnitZ()) *
                                     AngleAxisd(rpy[1], Vector3d::UnitY()) *
                                     AngleAxisd(rpy[0], Vector3d::UnitX());
    const Matrix3d inv_inertial_world = com_rotation * inv_inertial_ * com_rotation.inverse();

    /* A matrix of system*/
    A_.block(0, 6, 3, 3) = angular_vel_to_rpy_rate;
    A_(3, 9) = 1;
    A_(4, 10) = 1;
    A_(5, 11) = 1;
    A_(11, 12) = 1;
    // Test by HuNing-He at 2022-6-10, it seems that this can reduce the influence of forward speed on the height of the body
    A_(11, 9) = x_drag_; // The effect of x acceleration on acceleration in the z direction.

    /* B matrix of system*/
    Vector3d foot_pos;
    for (int i = 0; i < num_legs_; ++i) {
        foot_pos = init_states_.segment(13 + i * 3, 3) - com_pos;
        B_.block(6, i * 3, 3, 3) = inv_inertial_world * Ori::crossMatrix(foot_pos);
        B_(9, i * 3) = 1 / mass_;
        B_(10, i * 3 + 1) = 1 / mass_;
        B_(11, i * 3 + 2) = 1 / mass_;
    }
}

void ConvexMPC::UpdateQPMatrices() {
    /* concatenate A and B together, and solve system differential equation */
    MatrixXd AB = MatrixXd::Zero(13 + num_legs_ * 3, 13 + num_legs_ * 3);
    // Compute auxiliary matrix: [B_exp, A_exp * B_exp, ..., A_exp^(n-1) * B_exp]^T
    MatrixXd AnB_aux = MatrixXd::Zero(13 * predict_horizon_, num_legs_ * 3);

    AB.block(0, 0, 13, 13) = A_ * time_step_;
    AB.block(0, 13, 13, 3 * num_legs_) = B_ * time_step_;

    MatrixXd AB_exp = AB.exp();
    Matrix<double, 13, 13> A_exp;
    MatrixXd B_exp = MatrixXd::Zero(13, 3 * num_legs_);

    A_exp = AB_exp.block(0, 0, 13, 13);
    B_exp = AB_exp.block(0, 13, 13, 3 * num_legs_);

    A_qp_.block(0, 0, 13, 13) = A_exp;
    AnB_aux.block(0, 0, 13, num_legs_ * 3) = B_exp;
    for (int i = 1; i < predict_horizon_; ++i) {
        A_qp_.block(i * 13, 0, 13, 13) = A_exp * A_qp_.block((i - 1) * 13, 0, 13, 13);
        AnB_aux.block(i * 13, 0, 13, 3 * num_legs_) = A_exp * AnB_aux.block((i - 1) * 13, 0, 13, 3 * num_legs_);
    }
    /* Refer to [R2] */
    for (int i = 0; i < predict_horizon_; ++i) {
        // Diagonal block.
        B_qp_.block(i * 13, i * 3 * num_legs_, 13, 3 * num_legs_) = B_exp;
        // Off diagonal Diagonal block = A^(i - j - 1) * B_exp.
        for (int j = 0; j < i; ++j) {
            int power = i - j;
            B_qp_.block(i * 13, j * 3 * num_legs_, 13, 3 * num_legs_) = AnB_aux.block(power * 13, 0, 13, 3 * num_legs_);
        }
    }

    // We construct the H matrix by filling in h x h sub-matrices, each with size
    // 3 * num_legs_ x 3 * num_legs_.
    // The r_th (r in [1, h]) diagonal sub-matrix of H_ is:
    // 2 * sum_{i=0:h-r}(B'A'^i L A^i B) + alpha, where h is the predict-horizon.
    // The off-diagonal sub-matrix at row r and column c of H is:
    // 2 * sum_{i=0:h-c}(B'A'^{h-r-i} L A^{h-c-i} B)
    // We first compute the sub-matrices at column h.
    DiagonalMatrix<double, 13, 13> state_weights_single;
    Eigen::VectorXd state_weight = Eigen::VectorXd::Zero(13);
    state_weight << states_weights_, 0;
    state_weights_single.diagonal() = state_weight;

    for (int i = predict_horizon_ - 1; i >= 0; --i) {
        // calculate last column first
        H_.block(i * 3 * num_legs_, (predict_horizon_ - 1) * 3 * num_legs_, 3 * num_legs_, 3 * num_legs_) =
            AnB_aux.block((predict_horizon_ - i - 1) * 13, 0, 13, 3 * num_legs_).transpose() * state_weights_single * B_exp;
        // Fill the lower-triangle part by transposing the corresponding
        // upper-triangle part.
        if (i != predict_horizon_ - 1) {
            H_.block((predict_horizon_ - 1) * 3 * num_legs_, i * 3 * num_legs_, 3 * num_legs_, 3 * num_legs_) =
                H_.block(i * 3 * num_legs_, (predict_horizon_ - 1) * 3 * num_legs_, 3 * num_legs_, 3 * num_legs_).transpose();
        }
    }

    // We then fill in the sub-matrices in the middle by propagating the values
    // from lower right to upper left.
    for (int i = predict_horizon_ - 2; i >= 0; --i) {
        // Diagonal block.
        H_.block(i * 3 * num_legs_, i * 3 * num_legs_, 3 * num_legs_, 3 * num_legs_) =
            H_.block((i + 1) * 3 * num_legs_, (i + 1) * 3 * num_legs_, 3 * num_legs_, 3 * num_legs_) +
                AnB_aux.block((predict_horizon_ - i - 1) * 13, 0, 13, 3 * num_legs_).transpose() * state_weights_single *
                    AnB_aux.block((predict_horizon_ - i - 1) * 13, 0, 13, 3 * num_legs_);
        // Off diagonal block
        for (int j = i + 1; j < predict_horizon_ - 1; ++j) {
            H_.block(i * 3 * num_legs_, j * 3 * num_legs_, 3 * num_legs_, 3 * num_legs_) =
                H_.block((i + 1) * 3 * num_legs_, (j + 1) * 3 * num_legs_, 3 * num_legs_, 3 * num_legs_) +
                    AnB_aux.block((predict_horizon_ - i - 1) * 13, 0, 13, 3 * num_legs_).transpose() * state_weights_single *
                        AnB_aux.block((predict_horizon_ - j - 1) * 13, 0, 13, 3 * num_legs_);
            // Fill the lower-triangle part by transposing the corresponding
            // upper-triangle part.
            H_.block(j * 3 * num_legs_, i * 3 * num_legs_, 3 * num_legs_, 3 * num_legs_) =
                H_.block(i * 3 * num_legs_, j * 3 * num_legs_, 3 * num_legs_, 3 * num_legs_).transpose();
        }
    }

    // Add alpha and multiply by 2
    MatrixXd alpha_single = MatrixXd::Zero(3 * num_legs_, 3 * num_legs_);
    DiagonalMatrix<double, Eigen::Dynamic> alpha;
    alpha.diagonal() = input_weights_.replicate(predict_horizon_ * num_legs_, 1);

    Matrix3d input_weight;
    input_weight << input_weights_(0), 0, 0,
                    0, input_weights_(1), 0,
                    0, 0, input_weights_(2);
    for (int i = 0; i < num_legs_; ++i) {
        alpha_single.block(i * 3, i * 3, 3, 3) = input_weight;
    }
    for (int i = 0; i < predict_horizon_; ++i) {
        H_.block(i * 3 * num_legs_, i * 3 * num_legs_, 3 * num_legs_, 3 * num_legs_) += alpha_single;
    }
    H_ *= 2.0;
    // it was found that the code of the following comment is much slower than the above operation using matrix blocks, even if state_weights_diagonal is a diagonal matrix.
    // The above operations are mainly to speed up the calculation speed, which is equivalent to the following commented code
    // H_qp_ = 2 * (B_qp_.transpose() * state_weights_diagonal * B_qp_ + alpha);

    DiagonalMatrix<double, Eigen::Dynamic> state_weights_all;
    state_weights_all.diagonal() = state_weight.replicate(predict_horizon_, 1);
    // we could test for adding desire input for this MPC by minus 2 * alpha * des_inputs_
    g_ = 2 * B_qp_.transpose() * state_weights_all * (A_qp_ * init_states_.segment(0, 13) - des_states_) - 2 * alpha * des_inputs_;

    for (int i = 0; i < predict_horizon_ * num_legs_; ++i) {
        Aineq_.block(i * 5, i * 3, 5, 3) << -1, 0, mu_[i % num_legs_],
                                            1, 0, mu_[i % num_legs_],
                                            0, -1, mu_[i % num_legs_],
                                            0, 1, mu_[i % num_legs_],
                                            0, 0, 1;
    }

    for (unsigned int i = 0; i < predict_horizon_; ++i) {
        for (unsigned int j = 0; j < num_legs_; ++j) {
            unsigned int row = (i * num_legs_ + j) * 5;
            int contact_state = des_inputs_(i * num_legs_ * 3 + j * 3 + 2) > 0 ? 1 : 0;
            lbA_(row) = 0;
            lbA_(row + 1) = 0;
            lbA_(row + 2) = 0;
            lbA_(row + 3) = 0;
            lbA_(row + 4) = 0.5 * mass_ * 9.8 * contact_state / num_legs_;

            ubA_(row) = mass_ * 9.8 * 100; // just need a large number to replace infinity
            ubA_(row + 1) = mass_ * 9.8 * 100;
            ubA_(row + 2) = mass_ * 9.8 * 100;
            ubA_(row + 3) = mass_ * 9.8 * 100;
            ubA_(row + 4) = 6 * mass_ * 9.8 * contact_state / num_legs_;
        }
    }
}

VectorXd &ConvexMPC::UpdateMPC(const Eigen::VectorXd &state, const Eigen::VectorXd &des_state, const Eigen::VectorXd &des_inputs){
    init_states_ << state.segment(0, 12), -9.8, state.segment(12, 3 * num_legs_);
    Vector3d des_rpy = des_state.segment(0, 3);
    Vector3d des_com_pos_world = des_state.segment(3, 3);
    Vector3d des_rpy_vel = des_state.segment(6, 3);
    Vector3d des_com_vel_world = des_state.segment(9, 3);
    des_inputs_.setZero();
    MatrixXi foot_contact_states = MatrixXi::Zero(predict_horizon_, num_legs_);
    for (int i = 0; i < predict_horizon_; ++i) {
        for (int j = 0; j < num_legs_; ++j) {
            foot_contact_states(i, j) = des_inputs[i * num_legs_ + j];
        }
    }

    for (int i = 0; i < predict_horizon_; ++i) {
        des_states_[i * 13 + 0] = des_rpy[0];
        des_states_[i * 13 + 1] = des_rpy[1];
        des_states_[i * 13 + 2] = des_rpy[2] + time_step_ * (i + 1) * des_rpy_vel[2];
        des_states_[i * 13 + 3] = des_com_pos_world[0] + des_com_vel_world[0] * time_step_ * (i + 1);
        des_states_[i * 13 + 4] = des_com_pos_world[1] + des_com_vel_world[1] * time_step_ * (i + 1);
        des_states_[i * 13 + 5] = des_com_pos_world[2];

        des_states_[i * 13 + 6] = 0.0;
        des_states_[i * 13 + 7] = 0.0;
        des_states_[i * 13 + 8] = des_rpy_vel[2];

        des_states_[i * 13 + 9] = des_com_vel_world[0];
        des_states_[i * 13 + 10] = des_com_vel_world[1];
        des_states_[i * 13 + 11] = 0;
        des_states_[i * 13 + 12] = -9.8;

        for (int j = 0; j < num_legs_; ++j) { // foot_contact_states 0:swing 1:stance
            if (foot_contact_states.row(i).sum() > 0){
                des_inputs_(i * num_legs_ * 3 + j * 3 + 2) = foot_contact_states(i, j) * mass_ * 9.81 / foot_contact_states.row(i).sum();
            }
        }
    }
    UpdateSystemMatrices();
    UpdateQPMatrices();

    int constraint_dim = 5 * num_legs_ * predict_horizon_;
    int qp_dim = 3 * num_legs_ * predict_horizon_;

    RowMajorMatrixXd H_red, C_red; // reduced problem
    VectorXd lb_red, ub_red, g_red;
    H_red = RowMajorMatrixXd::Zero(qp_dim, qp_dim);
    C_red = RowMajorMatrixXd::Zero(constraint_dim, qp_dim);
    g_red = VectorXd::Zero(qp_dim);
    lb_red = VectorXd::Zero(constraint_dim);
    ub_red = VectorXd::Zero(constraint_dim);

    int stand_count = 0;
    int stand_cols;
    // In order to reduce the size of the QP problem, a lot of assignments and operations are added, which seems to be worthwhile at present.
    // And it is still a lot faster for the bi-foot support gait in general! For swing legs related to the state removed
    for (unsigned int i = 0; i < predict_horizon_; ++i) {
        for (unsigned int j = 0; j < num_legs_; ++j) {
            if (foot_contact_states(i, j) <= 0) { // one more swing leg in predict horizon to reduce the size of the QP problem once
                qp_dim -= 3;
                constraint_dim -= 5;
                continue;
            }

            stand_cols = 0;
            for (int m = 0; m < predict_horizon_; ++m) {
                for (int n = 0; n < num_legs_; ++n) {
                    if (foot_contact_states(m, n) <= 0){
                        continue;
                    }
                    H_red.block(stand_count * 3, stand_cols * 3, 3, 3) = H_.block(3 * num_legs_ * i + 3 * j, 3 * num_legs_ * m + 3 * n, 3, 3);
                    ++stand_cols;
                }
            }
            C_red.block(stand_count * 5, stand_count * 3, 5, 3) = Aineq_.block(i * 5 * num_legs_ + 5 * j, i * 3 * num_legs_ + 3 * j, 5, 3);
            g_red.segment(stand_count * 3, 3) = g_.segment(i * 3 * num_legs_ + j * 3, 3);
            lb_red.segment(stand_count * 5, 5) = lbA_.segment(i * 5 * num_legs_ + j * 5, 5);
            ub_red.segment(stand_count * 5, 5) = ubA_.segment(i * 5 * num_legs_ + j * 5, 5);
            ++stand_count;
        }
    }

    if (qp_dim < 3 * num_legs_ * predict_horizon_) { // All legs contact in the future, no need to reduce problem
        H_red.conservativeResize(qp_dim, qp_dim);   // The matrix stored with row priority facilitates calculations in qpOASES
        g_red.conservativeResize(qp_dim);
        C_red.conservativeResize(constraint_dim, qp_dim);
        lb_red.conservativeResize(constraint_dim);
        ub_red.conservativeResize(constraint_dim);
    }
    VectorXd qp_sol = VectorXd::Zero(qp_dim);

    if (qp_solver_ == QP_SOLVER::QP_OASES) {
        auto qp_problem = qpOASES::QProblem(qp_dim, constraint_dim);
        qpOASES::Options options;
        options.setToMPC();
        options.printLevel = qpOASES::PL_NONE;
        qp_problem.setOptions(options);
        int max_solver_iter = 100;
//        Timer qpOASES_time;
        qp_problem.init(H_red.data(), g_red.data(), C_red.data(), nullptr, nullptr, lb_red.data(), ub_red.data(), max_solver_iter);
        qp_problem.getPrimalSolution(qp_sol.data());
//        std::cout << "qpOASES solve time:" << qpOASES_time.getMs() <<std::endl;
//        for (int i = 0; i < 3; ++i) {
//            std::cout << "qpOASES solution:" << qp_sol[i] << std::endl;
//        }
    } else if(qp_solver_ == QP_SOLVER::QP_SWIFT) {
        RowMajorMatrixXd C_red_tmp = RowMajorMatrixXd::Zero(constraint_dim * 2, qp_dim);
        C_red_tmp.block(0, 0, constraint_dim, qp_dim) = C_red;
        C_red_tmp.block(constraint_dim, 0, constraint_dim, qp_dim) = -C_red;
        VectorXd up_red_swift = VectorXd::Zero(constraint_dim * 2);
        up_red_swift.segment(0, constraint_dim) = ub_red;
        up_red_swift.segment(constraint_dim, constraint_dim) = -lb_red;

//        Timer qpSWIFT_time;
        QP *myQP = QP_SETUP_dense(qp_dim, 2*constraint_dim, 0, H_red.data(), NULL, C_red_tmp.data(), g_red.data(), up_red_swift.data(), NULL, NULL, ROW_MAJOR_ORDERING);
        QP_SOLVE(myQP);
        for (int i = 0; i < qp_dim; ++i) {
            qp_sol[i] = myQP->x[i];
        }
//        std::cout << "qpSWIFT solve time:" << qpSWIFT_time.getMs() <<std::endl;
//        for (int i = 0; i < 3; ++i) {
//            std::cout << "qpSWIFT solution:" << myQP->x[i] << std::endl;
//        }
    }

    int buffer_index = 0;
    for (int i = 0; i < predict_horizon_; ++i) {
        for (int j = 0; j < num_legs_; ++j) {
            if (foot_contact_states(i, j) > 0) {
                qp_solution_[3 * i * num_legs_ + 3 * j] = -qp_sol[buffer_index * 3];
                qp_solution_[3 * i * num_legs_ + 3 * j + 1] = -qp_sol[buffer_index * 3 + 1];
                qp_solution_[3 * i * num_legs_ + 3 * j + 2] = -qp_sol[buffer_index * 3 + 2];
                ++buffer_index;
            } else {
                qp_solution_[3 * i * num_legs_ + 3 * j] = 0;
                qp_solution_[3 * i * num_legs_ + 3 * j + 1] = 0;
                qp_solution_[3 * i * num_legs_ + 3 * j + 2] = 0;
            }
        }
    }

    //Test for model accuracy i.e. test prediction's accuracy
    /*
    for (int i = 0; i < 3; ++i) {
        std::cout << i << "-th horizon's predictive states:\n"  << (A_qp_ * init_states_.segment(0, 13) - B_qp_ * qp_solution_).segment(i * 13, 13) << std::endl;
    }*/

    return qp_solution_;
}