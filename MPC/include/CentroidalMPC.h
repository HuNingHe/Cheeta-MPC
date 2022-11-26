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

#include "NonlinearMPC.h"

class CentroidalMPC : public NonlinearMPC {
private:
    double current_time_;
    double mass_;
    int num_legs_;
    Eigen::Vector3d foot_step_lb;
    Eigen::Vector3d foot_step_ub;
    Eigen::VectorXd mu_;
    void CreateSystemDynamic() final;

public:
    CentroidalMPC(double mass, int num_legs, int predict_horizon, double time_step,
                  const Eigen::VectorXd &weights, const Eigen::VectorXd& mu, IPOPT_SOLVER ipopt_solver = IPOPT_SOLVER::MA97);
    ~CentroidalMPC() = default;

    void SetupMPC() final;

    Eigen::VectorXd UpdateMPC(const Eigen::VectorXd &state, const Eigen::VectorXd &des_state, const Eigen::VectorXd &des_inputs) final;
};
