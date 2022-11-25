/*!
 * @file: CentroidalMPC.h
 * @authors: HuNing-He
 * @date: 2022-11-25
 * @copyright (c) 2022 HuNing-He. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 * @note: If you are using bipedal robot with sole contact, please refer to:
 * https://github.com/ami-iit/paper_romualdi_2022_icra_centroidal-mpc-walking.
 * This work is partially modified from this project
 */
#pragma once

#include <vector>
#include "NonlinearMPC.h"

/*!
 * Require weights as below
 * @note the following parameters are required by the class
 * |           Parameter Name          |    Type    |                             Description                          |
 * |:---------------------------------:|:----------:|:----------------------------------------------------------------:|
 * |         `omega_dot_weight`        |  `double`  |        Weight associated to the \f$\dot{omega}\f$                |
 * |       `dcm_tracking_weight`       |  `double`  |          Weight associated to the DCM tracking                   |
 * | `omega_dot_rate_of_change_weight` |  `double`  | Weight associated to the rate of change of \f$\dot{omega}\f$     |
 * |    `vrp_rate_of_change_weight`    |  `double`  |      Weight associated to the rate of change of the VRP          |
 * |    `dcm_rate_of_change_weight`    |  `double`  |      Weight associated to the rate of change of the DCM          |
 */
class CentroidalMPC : public NonlinearMPC {
    double current_time_;
    double mass_;
    int num_legs_;
    Eigen::VectorXd mu_;

public:
    CentroidalMPC(double mass, int num_legs, int predict_horizon, double time_step,
                  IPOPT_SOLVER ipopt_solver, const Eigen::VectorXd &weights, const Eigen::VectorXd& mu);
    ~CentroidalMPC() = default;

    void SetupMPC() final;

    void CreateSystemDynamic() final;

    Eigen::VectorXd UpdateMPC(const Eigen::VectorXd &state, const Eigen::VectorXd &des_state, const Eigen::VectorXd &des_inputs) final;
};
