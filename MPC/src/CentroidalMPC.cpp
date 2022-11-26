/*!
 * @file: CentroidalMPC.cpp
 * @authors: HuNing-He
 * @date: 2022-11-21
 * @copyright (c) 2022 HuNing-He. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */
#include <vector>
#include <string>
#include <cstring>
#include "MPC/include/CentroidalMPC.h"

std::vector<std::string> extractVariablesName(const std::vector<casadi::MX> &variables) {
    std::vector<std::string> variablesName;
    variablesName.reserve(variables.size());
    for (const auto &variable: variables) {
        variablesName.push_back(variable.name());
    }
    return variablesName;
}

template<class T>
auto extractFutureValuesFromState(T &variable) {
    using Sl = casadi::Slice;
    return variable(Sl(), Sl(1, variable.columns()));
}

CentroidalMPC::CentroidalMPC(double mass,
                             int num_legs,
                             int predict_horizon,
                             double time_step,
                             const Eigen::VectorXd &weights,
                             const Eigen::VectorXd& mu,
                             IPOPT_SOLVER ipopt_solver) :
                             NonlinearMPC(predict_horizon,
                                          time_step,
                                          ipopt_solver,
                                          weights) {
    assert(mass > 0 && num_legs > 0 && predict_horizon > 0);
    assert(mu.size() == num_legs);
    mass_ = mass;
    num_legs_ = num_legs;
    mu_ = mu;
    current_time_ = 0;
}

/*!
 * @note
 * system state x_k include {com_pos, com_vel, angular_momentum, contact_pos, contact_enable}.
 * system input u_k include {foot_vel, contact_force}.
 * dynamic_input include x_k and u_k.
 * dynamic_output is x_{k+1} exclude contact_enable.
 */
void CentroidalMPC::CreateSystemDynamic() {
    std::vector<casadi::MX> foot_pos{};
    std::vector<casadi::MX> foot_vel{};
    std::vector<casadi::MX> contact_force{};
    std::vector<casadi::MX> contact_enable{};

    for (int i = 0; i < num_legs_; ++i) {
        foot_pos.push_back(casadi::MX::sym("foot_pos_" + std::to_string(i + 1), 3));
        foot_vel.push_back(casadi::MX::sym("foot_vel_" + std::to_string(i + 1), 3));
        contact_force.push_back(casadi::MX::sym("contact_force_" + std::to_string(i + 1), 3));
        contact_enable.push_back(casadi::MX::sym("contact_enable_" + std::to_string(i + 1)));
    }

    casadi::MX com_pos = casadi::MX::sym("com_pos", 3);
    casadi::MX com_vel = casadi::MX::sym("com_vel", 3);
    casadi::MX angular_momentum = casadi::MX::sym("angular_momentum", 3);

    casadi::MX com_acc = casadi::MX::sym("com_acc", 3);
    casadi::MX angular_momentum_derivative = casadi::MX::sym("angular_momentum_derivative", 3);

    casadi::DM gravity = casadi::DM::zeros(3);
    gravity(2) = -9.81;
    angular_momentum_derivative = casadi::DM::zeros(3);
    com_acc = gravity;

    std::vector<casadi::MX> dynamic_input;
    dynamic_input.push_back(com_pos);
    dynamic_input.push_back(com_vel);
    dynamic_input.push_back(angular_momentum);

    for (int i = 0; i < num_legs_; ++i) {
        dynamic_input.push_back(foot_pos[i]);
        dynamic_input.push_back(contact_enable[i]);
        dynamic_input.push_back(foot_vel[i]);
        dynamic_input.push_back(contact_force[i]);
        com_acc += contact_enable[i] / mass_ * contact_force[i];
        angular_momentum_derivative += contact_enable[i] * casadi::MX::cross(foot_pos[i] - com_pos, contact_force[i]);
    }

    std::vector<std::string> dynamic_output_name_list{"com_pos", "com_vel", "angular_momentum"}; // at next time_step
    std::vector<casadi::MX> dynamic_output{com_pos + com_vel * time_step_,
                                           com_vel + com_acc * time_step_,
                                           angular_momentum + angular_momentum_derivative * time_step_};
    for (int i = 0; i < num_legs_; ++i) {
        dynamic_output.push_back(foot_pos[i] + (1 - contact_enable[i]) * foot_vel[i] * time_step_); // at next time_step
        dynamic_output_name_list.push_back("foot_pos_" + std::to_string(i + 1));
    }

    system_dynamic_ = casadi::Function("system_dynamic", dynamic_input, dynamic_output,
                                       extractVariablesName(dynamic_input), dynamic_output_name_list);
}

void CentroidalMPC::SetupMPC() {
    CreateSystemDynamic();
    using Sl = casadi::Slice;
    /*!
     * Note that variable in Opti create decision variable,
     * while parameter in Opti create a parameter(fixed during optimization)
     */
    casadi::MX com_pos = opti_.variable(3, predict_horizon_ + 1);
    casadi::MX com_vel = opti_.variable(3, predict_horizon_ + 1);
    casadi::MX angular_momentum = opti_.variable(3, predict_horizon_ + 1);

    casadi::MX cur_com_pos = opti_.parameter(3);
    casadi::MX cur_com_vel = opti_.parameter(3);
    casadi::MX cur_angular_momentum = opti_.parameter(3);
    casadi::MX des_com_pos = opti_.parameter(3, predict_horizon_ + 1);
    casadi::MX des_com_vel = opti_.parameter(3, predict_horizon_ + 1);
    casadi::MX des_angular_momentum = opti_.parameter(3, predict_horizon_ + 1);

    std::vector<casadi::MX> foot_pos;
    std::vector<casadi::MX> foot_vel;
    std::vector<casadi::MX> contact_force;
    std::vector<casadi::MX> contact_enable;

    std::vector<casadi::MX> des_foot_pos;
    std::vector<casadi::MX> des_contact_force;
    std::vector<casadi::MX> cur_foot_pos;
    std::vector<casadi::MX> step_lb; // lower bound of delta foot pos
    std::vector<casadi::MX> step_ub; // upper bound of delta foot pos

    for (int i = 0; i < num_legs_; ++i) {
        foot_pos.push_back(opti_.variable(3, predict_horizon_ + 1));
        foot_vel.push_back(opti_.variable(3, predict_horizon_));
        contact_force.push_back(opti_.variable(3, predict_horizon_));
        contact_enable.push_back(opti_.parameter(1, predict_horizon_));

        des_foot_pos.push_back(opti_.parameter(3, predict_horizon_ + 1));
        des_contact_force.push_back(opti_.parameter(3, predict_horizon_));

        cur_foot_pos.push_back(opti_.parameter(3));
        step_lb.push_back(opti_.parameter(3));
        step_ub.push_back(opti_.parameter(3));
    }

    // prepare the input of the system_dynamic
    std::vector<casadi::MX> dynamic_input;
    dynamic_input.push_back(com_pos(Sl(), Sl(0, -1)));          // exclude last column
    dynamic_input.push_back(com_vel(Sl(), Sl(0, -1)));          // exclude last column
    dynamic_input.push_back(angular_momentum(Sl(), Sl(0, -1))); // exclude last column
    for (int i = 0; i < num_legs_; ++i) {
        dynamic_input.push_back(foot_pos[i](Sl(), Sl(0, -1)));  // don't mess up the order
        dynamic_input.push_back(contact_enable[i]);
        dynamic_input.push_back(foot_vel[i]);
        dynamic_input.push_back(contact_force[i]);
    }

    // set the dynamics
    // map computes the multiple shooting method
    auto dynamics = system_dynamic_.map(predict_horizon_);
    auto full_trajectory = dynamics(dynamic_input);
    // set the initial values
    opti_.subject_to(cur_com_pos == com_pos(Sl(), 0));
    opti_.subject_to(cur_com_vel == com_vel(Sl(), 0));
    opti_.subject_to(cur_angular_momentum == angular_momentum(Sl(), 0));
    for (int i = 0; i < num_legs_; ++i) {
        opti_.subject_to(cur_foot_pos[i] == foot_pos[i](Sl(), 0));
    }

    opti_.subject_to(extractFutureValuesFromState(com_pos) == full_trajectory[0]);
    opti_.subject_to(extractFutureValuesFromState(com_vel) == full_trajectory[1]);
    opti_.subject_to(extractFutureValuesFromState(angular_momentum) == full_trajectory[2]);

    // footstep dynamics
    for (int i = 0; i < num_legs_; ++i) {
        opti_.subject_to(extractFutureValuesFromState(foot_pos[i]) == full_trajectory[3 + i]);
    }

    // add constraints for the contacts
    Eigen::Matrix<double, 5, 3> friction_cone_eigen = Eigen::Matrix<double, 5, 3>::Zero();
    casadi::DM friction_cone = casadi::DM::zeros(5, 3);

    casadi::DM force_lb = casadi::DM::zeros(5);
    casadi::DM force_ub{5000, 5000, 5000, 5000, mass_ * 9.81 * num_legs_};

    for (int i = 0; i < num_legs_; ++i) {
        friction_cone_eigen << -1, 0, mu_[i],
                               1, 0, mu_[i],
                               0, -1, mu_[i],
                               0, 1, mu_[i],
                               0, 0, 1;
        // convert the eigen matrix into casadi
        // please check https://github.com/casadi/casadi/issues/2563 and
        // https://groups.google.com/forum/#!topic/casadi-users/npPcKItdLN8
        // Assumption: the matrices as stored as column-major
        std::memcpy(friction_cone.ptr(), friction_cone_eigen.data(), sizeof(double) * 5 * 3);
        auto delta_foot_pos = extractFutureValuesFromState(foot_pos[i]) - extractFutureValuesFromState(des_foot_pos[i]);
        for (int j = 0; j < predict_horizon_; j++) {
            opti_.subject_to(step_lb[i] <= delta_foot_pos(Sl(), j) <= step_ub[i]);
            opti_.subject_to(force_lb <= casadi::MX::mtimes(friction_cone, contact_force[i](Sl(), j)) <= force_ub);
        }
    }

    casadi::DM weightCoMZ = casadi::DM::zeros(1, com_pos.columns());
    for (int i = 0; i < com_pos.columns(); i++) {
        weightCoMZ(Sl(), i) = (weights_(2) / 2) * std::exp(-i) + weights_(2) / 2; // give less trust on future state
    }

    casadi::MX cost = weights_(0) * casadi::MX::sumsqr(com_pos(0, Sl()) - des_com_pos(0, Sl())) +
                      weights_(1) * casadi::MX::sumsqr(com_pos(1, Sl()) - des_com_pos(1, Sl())) +
                      casadi::MX::sumsqr(weightCoMZ * (com_pos(2, Sl()) - des_com_pos(2, Sl()))) +
                      weights_(3) * casadi::MX::sumsqr(com_vel(0, Sl()) - des_com_vel(0, Sl())) +
                      weights_(4) * casadi::MX::sumsqr(com_vel(1, Sl()) - des_com_vel(1, Sl())) +
                      weights_(5) * casadi::MX::sumsqr(com_vel(2, Sl()) - des_com_vel(2, Sl())) +
                      weights_(6) * casadi::MX::sumsqr(angular_momentum(0, Sl()) - des_angular_momentum(0, Sl())) +
                      weights_(7) * casadi::MX::sumsqr(angular_momentum(1, Sl()) - des_angular_momentum(1, Sl())) +
                      weights_(8) * casadi::MX::sumsqr(angular_momentum(2, Sl()) - des_angular_momentum(2, Sl()));

    for (int i = 0; i < num_legs_; i++) {
        cost += weights_(9 + i * 3) * casadi::MX::sumsqr(foot_pos[i](0, Sl()) - des_foot_pos[i](0, Sl())) +
                weights_(10 + i * 3) * casadi::MX::sumsqr(foot_pos[i](1, Sl()) - des_foot_pos[i](1, Sl())) +
                weights_(11 + i * 3) * casadi::MX::sumsqr(foot_pos[i](2, Sl()) - des_foot_pos[i](2, Sl()));

        cost += weights_(9 + num_legs_ * 3 + i * 3) * casadi::MX::sumsqr(contact_force[i](0, Sl()) - des_contact_force[i](0, Sl())) +
                weights_(10 + num_legs_ * 3 + i * 3) * casadi::MX::sumsqr(contact_force[i](1, Sl()) - des_contact_force[i](1, Sl())) +
                weights_(11 + num_legs_ * 3 + i * 3) * casadi::MX::sumsqr(contact_force[i](2, Sl()) - des_contact_force[i](2, Sl()));

        auto force_rate_of_change = casadi::MX::diff(contact_force[i].T()).T();

        cost += weights_(9 + num_legs_ * 3 * 2 + i * 3) * casadi::MX::sumsqr(force_rate_of_change(0, Sl()));
        cost += weights_(10 + num_legs_ * 3 * 2 + i * 3) * casadi::MX::sumsqr(force_rate_of_change(1, Sl()));
        cost += weights_(11 + num_legs_ * 3 * 2 + i * 3) * casadi::MX::sumsqr(force_rate_of_change(2, Sl()));
    }

    opti_.minimize(cost);

    // prepare the casadi function
    std::vector<casadi::MX> controller_input{}, controller_output{};
    std::vector<std::string> input_name{}, output_name{};

    controller_input.push_back(cur_com_pos);
    controller_input.push_back(cur_com_vel);
    controller_input.push_back(cur_angular_momentum);
    controller_input.push_back(des_com_pos);
    controller_input.push_back(des_com_vel);
    controller_input.push_back(des_angular_momentum);

    input_name.emplace_back("cur_com_pos");
    input_name.emplace_back("cur_com_vel");
    input_name.emplace_back("cur_angular_momentum");
    input_name.emplace_back("des_com_pos");
    input_name.emplace_back("des_com_vel");
    input_name.emplace_back("des_angular_momentum");

    for (int i = 0; i < num_legs_; i++) {
        controller_input.push_back(cur_foot_pos[i]);
        controller_input.push_back(step_lb[i]);
        controller_input.push_back(step_ub[i]);
        controller_input.push_back(contact_enable[i]);
        controller_input.push_back(des_foot_pos[i]);
        controller_input.push_back(des_contact_force[i]);

        input_name.push_back("cur_foot_pos_" + std::to_string(i + 1));
        input_name.push_back("step_lb_of_foot_" + std::to_string(i + 1));
        input_name.push_back("step_ub_of_foot_" + std::to_string(i + 1));
        input_name.push_back("contact_enable_of_foot_" + std::to_string(i + 1));
        input_name.push_back("des_foot_pos_" + std::to_string(i + 1));
        input_name.push_back("des_contact_force_" + std::to_string(i + 1));

        controller_output.push_back(foot_pos[i]);
        controller_output.push_back(contact_force[i]);

        output_name.push_back("foot_pos_" + std::to_string(i + 1));
        output_name.push_back("contact_force_" + std::to_string(i + 1));
    }
    controller_ = opti_.to_function("controller", controller_input, controller_output, input_name, output_name);
}

Eigen::VectorXd CentroidalMPC::UpdateMPC(const Eigen::VectorXd &state, const Eigen::VectorXd &des_state, const Eigen::VectorXd &des_inputs) {
    casadi::DM cur_com_pos = casadi::DM::zeros(3);
    casadi::DM cur_com_vel = casadi::DM::zeros(3);
    casadi::DM cur_angular_momentum = casadi::DM::zeros(3);
    std::vector<casadi::DM> cur_foot_pos;

    toEigen(cur_com_pos) = state.segment(0, 3);
    toEigen(cur_com_vel) = state.segment(3, 3);
    toEigen(cur_angular_momentum) = state.segment(6, 3);
    for (int i = 0; i < num_legs_; ++i) {
        cur_foot_pos.push_back(casadi::DM::zeros(3));
        toEigen(cur_foot_pos[i]) = state.segment(9 + i * 3, 3);
    }

    casadi::DM des_com_pos = casadi::DM::zeros(3, predict_horizon_ + 1);
    casadi::DM des_com_vel = casadi::DM::zeros(3, predict_horizon_ + 1);
    casadi::DM des_angular_momentum = casadi::DM::zeros(3, predict_horizon_ + 1);

    std::memcpy(des_com_pos.ptr(), des_state.data(), sizeof(double) * 3 * (predict_horizon_ + 1));
    std::memcpy(des_com_vel.ptr(), des_state.data() + 3 * (predict_horizon_ + 1), sizeof(double) * 3 * (predict_horizon_ + 1));
    std::memcpy(des_angular_momentum.ptr(), des_state.data() + 6 * (predict_horizon_ + 1), sizeof(double) * 3 * (predict_horizon_ + 1));

    std::vector<casadi::DM> des_foot_pos;
    std::vector<casadi::DM> des_contact_force;
    std::vector<casadi::DM> contact_enable;
    std::vector<casadi::DM> step_lb;
    std::vector<casadi::DM> step_ub;

    for (int i = 0; i < num_legs_; ++i) {
        des_foot_pos.push_back(casadi::DM::zeros(3, predict_horizon_ + 1));
        des_contact_force.push_back(casadi::DM::zeros(3, predict_horizon_));
        contact_enable.push_back(casadi::DM::zeros(1, predict_horizon_));
        step_lb.push_back(casadi::DM::zeros(3));
        step_ub.push_back(casadi::DM::zeros(3));

        std::memcpy(contact_enable[i].ptr(), des_inputs.data(), sizeof(double) * predict_horizon_);
        std::memcpy(des_foot_pos[i].ptr(), des_inputs.data() + predict_horizon_, sizeof(double) * 3 * (predict_horizon_ + 1));
        std::memcpy(des_contact_force[i].ptr(), des_inputs.data() + 4 * predict_horizon_ + 3, sizeof(double) * 3 * predict_horizon_);

        std::memcpy(step_lb[i].ptr(), foot_step_lb.data(), sizeof(double) * 3);
        std::memcpy(step_ub[i].ptr(), foot_step_ub.data(), sizeof(double) * 3);
    }

    std::vector<casadi::DM> inputs;
    inputs.emplace_back(cur_com_pos);
    inputs.emplace_back(cur_com_vel);
    inputs.emplace_back(cur_angular_momentum);
    inputs.emplace_back(des_com_pos);
    inputs.emplace_back(des_com_vel);
    inputs.emplace_back(des_angular_momentum);

    for (int i = 0; i < num_legs_; i++) {
        inputs.emplace_back(cur_foot_pos[i]);
        inputs.emplace_back(step_lb[i]);
        inputs.emplace_back(step_ub[i]);
        inputs.emplace_back(contact_enable[i]);
        inputs.emplace_back(des_foot_pos[i]);
        inputs.emplace_back(des_contact_force[i]);
    }

    auto output = controller_(inputs);

//    std::memcpy(output[0])
    std::cout << output << std::endl;
    current_time_ += time_step_;
    return {};
}
