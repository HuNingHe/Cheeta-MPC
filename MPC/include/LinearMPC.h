/*!
 * @file basicMPC.h
 * @author HuNing-He
 * @date 2022-7-26
 * @email 2689112371@qq.com
 * @brief Standard Linear MPC formulation is presented here.
 * Users should override UpdateSystemMatrices() , UpdateQPMatrices() and UpdateMPC() functions.
 * Refer to file "StepAdaption.h" and "StepAdaption.cpp".
 * Note that we don't use sparse MPC here. Sparse MPC should faster than Dense MPC. You can make this conversion by eigen.
 * qpSWIFT is recommended here
 */

#ifndef MAIN_CPP_BASICMPC_H
#define MAIN_CPP_BASICMPC_H
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cassert>
#include "Timer.h"
#include <qpOASES.hpp>
#include <qpSWIFT.h>

enum class QP_SOLVER : unsigned int {
    QP_OASES = 0,
    QP_SWIFT = 1
};

using RowMajorMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

class LinearMPC {
protected:
    int predict_horizon_;
    double decay_rate_;
    double time_step_;

    /*!
     * System state space matrix:
     * x_(k+1) = A_ * x_k + B_ * u_k + d_
     * For non-linearized system d_sys_ is usually constant zero vector
     */
    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
    Eigen::VectorXd d_;

    /*!
     * standard qpSWIFT quadratic programing as follows:
     * min 0.5 * x^T * H_ * x + g_^T * x
     * s.t.  Aeq_ * x = beq_
     *     Aineq_ * x <= ubA_
     *
     * standard qpOASES quadratic programing as follows:
     * min 0.5 * x^T * H_ * x + x^T * g_
     * s.t.  lbA_ <= Aineq_ * x <= ubA_
     *           lb_ <= x <= ub_
     * because qpOASES only support row-major order for matrix, so we use row-major for qpSWIFT
     */
    RowMajorMatrixXd H_;
    Eigen::VectorXd g_;
    RowMajorMatrixXd Aeq_;
    Eigen::VectorXd beq_;
    RowMajorMatrixXd Aineq_;
    Eigen::VectorXd lbA_;
    Eigen::VectorXd ubA_;
    Eigen::VectorXd lb_;
    Eigen::VectorXd ub_;

    Eigen::VectorXd qp_solution_;

    Eigen::VectorXd states_weights_;
    Eigen::VectorXd terminal_weights_;
    Eigen::VectorXd input_weights_;

    Eigen::VectorXd init_states_;
    Eigen::VectorXd des_states_;
    Eigen::VectorXd init_inputs_;
    Eigen::VectorXd des_inputs_;
    QP_SOLVER qp_solver_;

public:
    LinearMPC(int predict_horizon, double time_step, double decay_rate = 1, QP_SOLVER solver = QP_SOLVER::QP_SWIFT) {
        predict_horizon_ = predict_horizon;
        time_step_ = time_step;
        decay_rate_ = decay_rate;
        qp_solver_ = solver;
        qp_solution_ = Eigen::VectorXd::Zero(12);
    }
    LinearMPC() = delete;
    LinearMPC(const LinearMPC &mpc) = delete;
    LinearMPC& operator=(const LinearMPC &mpc) = delete;

    virtual ~LinearMPC() = default;

    void UpdateStatesWeights(const Eigen::VectorXd &states_weights) {
        states_weights_ = states_weights;
    }

    void UpdateTerminalWeights(const Eigen::VectorXd &terminal_weights) {
        terminal_weights_ = terminal_weights;
    }

    void UpdateInputWeights(const Eigen::VectorXd &input_weights) {
        input_weights_ = input_weights;
    }

    /*!
     * init input is just the qp solution of first horizon
     */
    virtual Eigen::VectorXd &UpdateMPC(const Eigen::VectorXd &state, const Eigen::VectorXd &des_state, const Eigen::VectorXd &des_inputs) = 0;

protected:
    /*!
     * update system states matrix A_, B_ and d_
     */
    virtual void UpdateSystemMatrices() = 0;
    /*!
     * update quadratic programing matrix H_, g_, Aeq_, beq_, Aineq_, lb_, ub_, lbA and ubA
     */
    virtual void UpdateQPMatrices() = 0;
};

#endif //MAIN_CPP_BASICMPC_H
