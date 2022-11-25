/*！
 * @file: RF_MPC.h
 * @author: HuNing-He
 * @date:2022-6-28
 * @email: 2689112371@qq.com
 * @refer to "Representation-Free Model Predictive Control for Dynamic Motions in Quadrupeds"(TRO 2021) by YanRan-Ding
 * and his open source matlab code https://github.com/YanranDing/RF-MPC
 * I convert it to c++ and add foot location as decision variable in optimal problem
 * My paper may be published soon, before that you can refer to "Dynamic Walking with Footstep Adaptation on the MIT
 * Humanoid via Linear Model Predictive Control" by YanRan-Ding(pre-print, 2022), this pre-print version may exist some typos.
 */

#ifndef MAIN_CPP_RF_MPC_H
#define MAIN_CPP_RF_MPC_H
#include <MPC/include/LinearMPC.h>
#include <utility>
#include <vector>
#include "filters.h"

class RF_MPC : public LinearMPC {
private:
    const double mass_;
    const int num_legs_;
    const Eigen::Matrix3d inertial_;
    const Eigen::Matrix3d inv_inertial_;

    Eigen::Matrix<double, 9, 3> N_;    // Constant matrix, refer to the paper.Convert so3 to a vector form
    Eigen::Matrix<double, 3, 9> inv_N_;// pseudo inverse of matrix N_
    Eigen::VectorXd mu_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @param mass body mass
     * @param inertia body inertial
     * @param num_legs
     * @param planning_horizon predictive horizon
     * @param time_step mpc sample period
     * @param state_weights 12 x 1 array of state qp weights
     * @param terminal_weights 12 x 1 array of terminal qp weights
     * @param input_weights system input weights
     * @param friction_coeff friction coefficient
     * @param decay_rate decay rate for cost function in mpc, because of SO3 linearization error and model uncertainty, we give less trust on future state
     */
    RF_MPC(double mass, const Eigen::Matrix3d& inertial, int num_legs, int planning_horizon, double time_step,
           Eigen::Matrix<double, 12, 1> &state_weights, Eigen::Matrix<double, 12, 1> &terminal_weights,
           Eigen::Vector3d &input_weights, Eigen::VectorXd friction_coeff, double decay_rate = 1);
    ~RF_MPC() override = default;

    void UpdateFriction(Eigen::VectorXd friction_coeff) {
        assert(friction_coeff.rows() == num_legs_);
        mu_ = std::move(friction_coeff);
    }

    Eigen::VectorXd &UpdateMPC(const Eigen::VectorXd &state, const Eigen::VectorXd &des_state, const Eigen::VectorXd &des_inputs) final;

protected:
    void UpdateSystemMatrices() final;
    void UpdateQPMatrices() final;
};

#endif //MAIN_CPP_RF_MPC_H