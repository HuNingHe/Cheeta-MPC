/*!
 * @file ConvexMPC.h
 * @brief origin from Google Open source code motion-imitation/mpc_controller in
 * https://github.com/erwincoumans/motion_imitation/blob/master/mpc_controller/mpc_osqp.cc
 * I just made some modifications to speed up mpc solve time by referring to CheetahSoftware
 * @modified by HuNing-He
 * @email 2689112371@qq.com
 * @date 2022-5-26
 */

/*!
References:
   [R1] Jared Di Carlo, Patrick M. Wensing, Benjamin Katz, Gerardo Bledt, Sangbae Kim.
   Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control, 2018.
   [R2] ChengYe Wu 1, Qing Wei, Cong Zhang, HongLei An.
   Balance control of quadruped robot based on model predictive control, 2020.
   [R3] 于宪元-基于稳定性的仿生四足机器人控制系统设计[D].北京航空航天大学.2021.
*/

#ifndef CONVEXMPC_H
#define CONVEXMPC_H
#include <utility>

#include "MPC/include/LinearMPC.h"

class ConvexMPC : public LinearMPC {
private:
    double mass_;
    double x_drag_;
    int num_legs_;
    Eigen::Matrix3d inertial_;
    Eigen::Matrix3d inv_inertial_;
    Eigen::VectorXd mu_;
    Eigen::MatrixXd A_qp_;
    Eigen::MatrixXd B_qp_;
public:
    /*!
     * @param mass body mass
     * @param inertia body inertial
     * @param num_legs
     * @param planning_horizon predictive horizon
     * @param time_step mpc sample period
     * @param state_weights 12 x 1 array of state qp weights
     * @param input_weights system input weights
     * @param friction_coeff friction coefficient
     */
    ConvexMPC(double mass, const Eigen::Matrix3d& inertia, int num_legs, int planning_horizon, double time_step,
              Eigen::Matrix<double, 12, 1> &state_weights, Eigen::Vector3d &input_weights, Eigen::VectorXd friction_coeff);
    ~ConvexMPC() override = default;

    void UpdateFriction(Eigen::VectorXd friction_coeff) {
        assert(friction_coeff.rows() == num_legs_);
        mu_ = std::move(friction_coeff);
    }

    void UpdateXdrag(double xdrag) {
        x_drag_ = xdrag;
    }

    Eigen::VectorXd &UpdateMPC(const Eigen::VectorXd &state, const Eigen::VectorXd &des_state, const Eigen::VectorXd &des_inputs) override;
protected:
    void UpdateSystemMatrices() override;
    void UpdateQPMatrices() override;
};

#endif //CONVEXMPC_H
