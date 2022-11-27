/*!
 * @file: CentroidalMPCTest.cpp
 * @authors: HuNing-He
 * @date: 2022-11-26
 * @copyright (c) 2022 HuNing-He. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */
#include "CentroidalMPC.h"
#include "iostream"
#include <memory>
int main(){
    double mass = 8;
    double time_step = 0.01;
    int num_legs = 4;
    int horizon = 6;
    Eigen::VectorXd mu = Eigen::VectorXd::Zero(num_legs);
    mu << 0.8, 0.8, 0.8, 0.8;
    Eigen::VectorXd weights = Eigen::VectorXd::Zero((num_legs + 1) * 9);
    weights << 1, 1, 15,        // com pos
               0.5, 0.5, 0,     // com vel
               2, 2, 8,         //angular_momentum
               0.2, 0.2, 0.2,   // foot pos
               0.3, 0.3, 0.3,   // contact force
               0.1, 0.1, 0.1,   // force rate
               0.2, 0.2, 0.2,
               0.3, 0.3, 0.3,
               0.1, 0.1, 0.1,
               0.2, 0.2, 0.2,
               0.3, 0.3, 0.3,
               0.1, 0.1, 0.1,
               0.2, 0.2, 0.2,
               0.3, 0.3, 0.3,
               0.1, 0.1, 0.1;
    std::shared_ptr<CentroidalMPC> mpc = std::make_shared<CentroidalMPC>(mass, num_legs, horizon, time_step, weights, mu);
    mpc->SetupMPC();
    Eigen::VectorXd state = Eigen::VectorXd::Zero(3* (num_legs + 3));
    Eigen::VectorXd des_state = Eigen::VectorXd::Zero(9 * (horizon + 1));
    Eigen::VectorXd des_input = Eigen::VectorXd::Zero(7 * horizon + 3);

    state << 0.3, 0, 0.15,     // com pos
             0.1, 0, 0,        // com vel
             0, 0, 0,          // angular momentum
             0.35, 0.052, 0,   // lf foot pos
             0.35, -0.054, 0,  // rf foot pos
             -0.37, -0.053, 0, // rh foot pos
             -0.36, 0.054, 0;  // lh foot pos 
    
    Eigen::MatrixXd mpc_table = Eigen::MatrixXd::Zero(horizon, num_legs);
    mpc_table << 1, 0, 1, 0,
                 1, 0, 1, 0,
                 1, 0, 1, 0,
                 0, 1, 0, 1,
                 0, 1, 0, 1,
                 0, 1, 0, 1;

    mpc->UpdateMPC(state, des_state, des_input);
    std::cout << "heihei" << std::endl;
    return 0;
}
