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
    weights << 1, 1, 100,       // com pos
               0.5, 0.5, 0,     // com vel
               2, 2, 8,         // angular_momentum
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
    Eigen::VectorXd des_input = Eigen::VectorXd::Zero(num_legs * (4 * horizon + 3));

    state << 0, 0, 0.15,            // com pos
             0.1, 0, 0,             // com vel
             0, 0, 0.1,             // angular momentum
             0.35, 0.052, 0,        // lf foot pos
             0.35, -0.054, 0,       // rf foot pos
             -0.37, -0.053, 0,      // rh foot pos
             -0.36, 0.054, 0;       // lh foot pos 

    des_state << 0.31, 0, 0.16,     // des com pos firsr horizon
                 0.32, 0, 0.168,   
                 0.33, 0, 0.172,    
                 0.33, 0, 0.18, 
                 0.34, 0, 0.19,
                 0.348, 0, 0.2,
                 0.1, 0, 0,         // des com vel first horizon
                 0.09, 0, 0,
                 0.08, 0, 0,
                 0.06, 0, 0,
                 0.04, 0, 0,
                 0, 0, 0,
                 0, 0, 0.12,        // des angular momentum first horizon
                 0, 0, 0.14,
                 0, 0, 0.16,
                 0, 0, 0.18,
                 0, 0, 0.2,
                 0, 0, 0.22;

    Eigen::MatrixXd mpc_table = Eigen::MatrixXd::Zero(horizon, num_legs);
    mpc_table << 1, 0, 1, 0,
                 1, 0, 1, 0,
                 1, 0, 1, 0,
                 0, 1, 0, 1,
                 0, 1, 0, 1,
                 0, 1, 0, 1;
    Eigen::VectorXd des_foot_pos[4];

    des_foot_pos[0] = Eigen::VectorXd::Zero((horizon + 1) * 3);
    des_foot_pos[0] << 0.35, 0.052, 0,
                       0.35, 0.052, 0,
                       0.35, 0.052, 0,
                       0.35, 0.052, 0,
                       0.38, 0.052, 0,
                       0.39, 0.052, 0,
                       0.42, 0.052, 0;
    des_foot_pos[1] = Eigen::VectorXd::Zero((horizon + 1) * 3);
    des_foot_pos[1] << 0.35, -0.054, 0,
                       0.37, -0.052, 0,
                       0.39, -0.052, 0,
                       0.43, -0.052, 0,
                       0.43, -0.052, 0,
                       0.43, -0.052, 0,
                       0.43, -0.052, 0;
    des_foot_pos[2] = Eigen::VectorXd::Zero((horizon + 1) * 3);
    des_foot_pos[2] << -0.37, -0.052, 0,
                       -0.37, -0.052, 0,
                       -0.37, -0.052, 0,
                       -0.36, -0.052, 0,
                       -0.34, -0.052, 0,
                       -0.30, -0.052, 0,
                       -0.28, -0.052, 0;
    des_foot_pos[3] = Eigen::VectorXd::Zero((horizon + 1) * 3);
    des_foot_pos[3] << -0.36, 0.053, 0,
                       -0.34, 0.053, 0,
                       -0.32, 0.053, 0,
                       -0.31, 0.053, 0,
                       -0.31, 0.052, 0,
                       -0.31, 0.052, 0,
                       -0.31, 0.052, 0;
    for (int i = 0; i < num_legs; i++){
        des_input.segment(i * (4 * horizon + 3), horizon) = mpc_table.col(i);
        des_input.segment(horizon + i * (4 * horizon + 3), 3 * (horizon + 1)) = des_foot_pos[i];
    }
    
    mpc->UpdateMPC(state, des_state, des_input);
    std::cout << "finished test by hun" << std::endl;
    return 0;
}
