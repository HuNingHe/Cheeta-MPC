#ifndef MAIN_CPP_STEPADAPTION_H
#define MAIN_CPP_STEPADAPTION_H

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

#include "MPC/include/LinearMPC.h"

class SARF_MPC : public LinearMPC {
public:
    /*!
     * @param mass body mass
     * @param inertia body inertial
     * @param num_legs
     * @param planning_horizon predictive horizon
     * @param time_step mpc sample period
     * @param state_weights 15 x 1 array of state qp weights
     * @param terminal_weights 15 x 1 array of terminal qp weights
     * @param input_weights system input weights
     * @param delta_foot_weights if adapt_foot is true, this should not be zero vector
     * @param decay_rate decay rate for cost function in mpc, because of SO3 linearization error and model uncertainty, we give less trust on future state
     */
    SARF_MPC(double mass, Eigen::Matrix3d inertia, int num_legs, int planning_horizon, double time_step, Eigen::VectorXd &friction_coeff,
             Eigen::Matrix<double, 15, 1> &state_weights, Eigen::Matrix<double, 15, 1> &terminal_weights,
             Eigen::Vector3d &input_weights, Eigen::Vector3d& delta_foot_weights,  double decay_rate = 1);
    ~SARF_MPC() override = default;

    /*!
     * @param com_pos_world
     * @param com_vel_world
     * @param rpy
     * @param angular_vel_body
     * @param foot_contact_states contact states in predict horizon
     * @param foot_positions_world_frame
     * @param foot_friction_coeffs we make this as input for those friction coefficients identification algorithms
     * @param des_com_pos_world
     * @param des_com_vel_world
     * @param des_rpy
     * @param des_rpy_vel
     * @param des_foot_pos des foot hold
     * @return optimal contact forces and optimal foot step location
     */

    Eigen::VectorXd &UpdateMPC(const Eigen::Vector3d &com_pos_world,
                               const Eigen::Vector3d &com_vel_world,
                               const Eigen::Matrix3d &rotation_matrix,
                               const Eigen::Vector3d &angular_vel_body,
                               const Eigen::MatrixXi &foot_contact_states,
                               const Eigen::MatrixXd &foot_positions_world_frame,
                               const Eigen::VectorXd &foot_friction_coeffs,
                               const Eigen::Vector3d &des_com_pos_world,
                               const Eigen::Vector3d &des_com_vel_world,
                               const Eigen::Vector3d &des_rpy,
                               const Eigen::Vector3d &des_rpy_vel,
                               const Eigen::MatrixXd &des_foot_pos,
                               Eigen::Matrix<double, 4, 3> &last_stance_foot_pos);

    Eigen::VectorXd &UpdateMPC(const Eigen::VectorXd &state, const Eigen::VectorXd &des_state, const Eigen::VectorXd &des_inputs) override;
    void UpdateFriction(const Eigen::VectorXd &friction_coeff){
        assert(friction_coeff.rows() == num_legs_);
        friction_coeff_ = friction_coeff;
    }

    void UpdateDeltaFootWeights(const Eigen::Vector3d &weights){
        delta_foot_weights_ = weights;
    }

private:
    double mass_;
    Eigen::Matrix3d inertia_;
    Eigen::Matrix3d inv_inertia_;
    int num_legs_;

    Eigen::Matrix<double, 9, 3> N_;    // Constant matrix, refer to the paper.Convert so3 to a vector form
    Eigen::Matrix<double, 3, 9> inv_N_;// pseudo inverse of matrix N_

    Eigen::Vector3d delta_foot_weights_;

    Eigen::VectorXd friction_coeff_;
    Eigen::MatrixXi step_attempts_;
    /*!
     * only one step-attempt in predict horizon, you should set your mpc/gait period and predict horizon properly
     * @param mpcTable
     */
    void checkStepAttempt(const Eigen::MatrixXi &mpcTable);
    void UpdateSystemMatrices() override;
    void UpdateQPMatrices() override;
};
#endif //MAIN_CPP_STEPADAPTION_H
