/*!
 * @file: CentroidalMPC.cpp
 * @authors: HuNing-He
 * @date: 2022-11-21
 * @copyright (c) 2022 HuNing-He. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <string>
#include <unordered_map>
#include <string>
#include <iterator>
#include <utility>
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

struct CasadiContact {
    casadi::MX position;
    casadi::MX orientation;
    casadi::MX linearVelocity;
    casadi::MX isEnable;

    CasadiContact() = default;

    CasadiContact &operator=(const ContactWithCorners &other) {
        corners.resize(other.corners.size());

        for (int i = 0; i < other.corners.size(); i++) {
            this->corners[i] = other.corners[i];
        }

        this->position = casadi::MX::sym("position", 3);
        this->linearVelocity = casadi::MX::sym("linear_velocity", 3);
        this->isEnable = casadi::MX::sym("is_enable");

        return *this;
    }

    CasadiContact(const ContactWithCorners &other) {
        this->operator=(other);
    }
};


    struct CasadiContactWithConstraints : CasadiContact {
        casadi::MX currentPosition;
        casadi::MX nominalPosition;
        casadi::MX upperLimitPosition;
        casadi::MX lowerLimitPosition;
    };

    /*!
     * OptimizationVariables contains the optimization variables expressed as CasADi elements.
     */
    struct OptimizationVariables {
        casadi::MX com;
        casadi::MX dcom;
        casadi::MX angularMomentum;
        std::map<std::string, CasadiContactWithConstraints> contacts;

        casadi::MX comReference;
        casadi::MX comCurrent;
        casadi::MX dcomCurrent;
        casadi::MX angularMomentumCurrent;
        casadi::MX externalWrench;
    };
    OptimizationVariables optiVariables; /**< Optimization variables */

    struct ContactsInputs {
        casadi::DM currentPosition;
        casadi::DM orientation;
        casadi::DM nominalPosition;
        casadi::DM upperLimitPosition;
        casadi::DM lowerLimitPosition;
        casadi::DM isEnable;
    };

    struct ControllerInputs {
        std::map<std::string, ContactsInputs> contacts;
        casadi::DM comReference;
        casadi::DM comCurrent;
        casadi::DM dcomCurrent;
        casadi::DM angularMomentumCurrent;
        casadi::DM externalWrench;
    };
    ControllerInputs controllerInputs;

    struct ContactBoundingBox {
        Eigen::Vector3d upperLimit;
        Eigen::Vector3d lowerLimit;
    };

    std::unordered_map<std::string, ContactBoundingBox> contactBoundingBoxes;

    casadi::Function contactPositionError() {
        casadi::MX contactPosition = casadi::MX::sym("contact_position", 3);
        casadi::MX nominalContactPosition = casadi::MX::sym("nominal_contact_position", 3);
        casadi::MX contactOrientation = casadi::MX::sym("contact_orientation", 3 * 3);

        // the orientation is stored as a vectorized version of the matrix. We need to reshape it
        casadi::MX rhs = casadi::MX::mtimes(casadi::MX::reshape(contactOrientation, 3, 3).T(),
                                            contactPosition - nominalContactPosition);

        return casadi::Function("contact_position_error",
                                {contactPosition, nominalContactPosition, contactOrientation},
                                {rhs},
                                extractVariablesName({contactPosition, //
                                                      nominalContactPosition,
                                                      contactOrientation}),
                                {"error"});
    }

    void resizeControllerInputs() {
        constexpr int vector3Size = 3;
        const int stateHorizon = this->optiSettings.horizon + 1;

        // prepare the controller inputs struct
        this->controllerInputs.comCurrent = casadi::DM::zeros(vector3Size);
        this->controllerInputs.dcomCurrent = casadi::DM::zeros(vector3Size);
        this->controllerInputs.angularMomentumCurrent = casadi::DM::zeros(vector3Size);
        this->controllerInputs.comReference = casadi::DM::zeros(vector3Size, stateHorizon);
        this->controllerInputs.externalWrench = casadi::DM::zeros(vector3Size, //
                                                                  this->optiSettings.horizon);

        for (const auto &[key, contact]: this->state.contacts) {
            // The current position of the contact
            this->controllerInputs.contacts[key].currentPosition
                    = casadi::DM::zeros(vector3Size);

            // The orientation is stored as a vectorized version of the rotation matrix
            this->controllerInputs.contacts[key].orientation
                    = casadi::DM::zeros(9, this->optiSettings.horizon);

            // Upper limit of the position of the contact. It is expressed in the contact body frame
            this->controllerInputs.contacts[key].upperLimitPosition
                    = casadi::DM::zeros(vector3Size, this->optiSettings.horizon);

            // Lower limit of the position of the contact. It is expressed in the contact body frame
            this->controllerInputs.contacts[key].lowerLimitPosition
                    = casadi::DM::zeros(vector3Size, this->optiSettings.horizon);

            // Maximum admissible contact force. It is expressed in the contact body frame
            this->controllerInputs.contacts[key].isEnable
                    = casadi::DM::zeros(1, this->optiSettings.horizon);

            // The nominal contact position is a parameter that regularize the solution
            this->controllerInputs.contacts[key].nominalPosition
                    = casadi::DM::zeros(vector3Size, stateHorizon);
        }
    }

CentroidalMPC::CentroidalMPC(double mass,
                             int num_legs,
                             int predict_horizon,
                             double time_step,
                             IPOPT_SOLVER ipopt_solver,
                             const Eigen::VectorXd &weights,
                             const Eigen::VectorXd& mu) : NonlinearMPC(predict_horizon, time_step, ipopt_solver, weights) {
    assert(mass > 0 && num_legs > 0 && predict_horizon > 0);
    mass_ = mass;
    num_legs_ = num_legs;
    mu_ = mu;
    current_time_ = 0;
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

    std::vector<casadi::MX> foot_pos;
    std::vector<casadi::MX> foot_vel;
    std::vector<casadi::MX> contact_force;
    std::vector<casadi::MX> contact_enable;

    std::vector<casadi::MX> des_foot_pos;
    std::vector<casadi::MX> cur_foot_pos;
    std::vector<casadi::MX> foot_pos_lb; // lower bound of foot pos
    std::vector<casadi::MX> foot_pos_ub; // upper bound of foot pos

    for (int i = 0; i < num_legs_; ++i) {
        foot_pos.push_back(opti_.variable(3, predict_horizon_ + 1));
        foot_vel.push_back(opti_.variable(3, predict_horizon_));
        contact_force.push_back(opti_.variable(3, predict_horizon_));
        contact_enable.push_back(opti_.parameter(1, predict_horizon_));

        des_foot_pos.push_back(opti_.parameter(3, predict_horizon_ + 1));
        cur_foot_pos.push_back(opti_.parameter(3));
        foot_pos_lb.push_back(opti_.parameter(3, predict_horizon_));
        foot_pos_ub.push_back(opti_.parameter(3, predict_horizon_));
    }

    // prepare the input of the system_dynamic
    std::vector<casadi::MX> dynamic_input;
    dynamic_input.push_back(com_pos(Sl(), Sl(0, -1))); // exclude last column
    dynamic_input.push_back(com_vel(Sl(), Sl(0, -1))); // exclude last column
    dynamic_input.push_back(angular_momentum(Sl(), Sl(0, -1))); // exclude last column
    for (int i = 0; i < num_legs_; ++i) {
        dynamic_input.push_back(foot_pos[i](Sl(), Sl(0, -1))); // exclude last column
        dynamic_input.push_back(contact_enable[i]);
        dynamic_input.push_back(foot_vel[i]);
        dynamic_input.push_back(contact_force[i]);
    }

    // set the initial values
    opti_.subject_to(cur_com_pos == com_pos(Sl(), 0));
    opti_.subject_to(cur_com_vel == com_vel(Sl(), 0));
    opti_.subject_to(cur_angular_momentum == angular_momentum(Sl(), 0));
    for (int i = 0; i < num_legs_; ++i) {
        opti_.subject_to(cur_foot_pos[i] == foot_pos[i](Sl(), 0));
    }

    // set the dynamics
    // map computes the multiple shooting method
    auto dynamics = system_dynamic_.map(predict_horizon_);
    auto full_trajectory = dynamics(dynamic_input);
    opti_.subject_to(extractFutureValuesFromState(com_pos) == full_trajectory[0]);
    opti_.subject_to(extractFutureValuesFromState(com_vel) == full_trajectory[1]);
    opti_.subject_to(extractFutureValuesFromState(angular_momentum) == full_trajectory[2]);

    // footstep dynamics
    std::size_t contactIndex = 0;
    for (int i = 0; i < num_legs_; ++i) {
        opti_.subject_to(extractFutureValuesFromState(foot_pos[i]) == full_trajectory[3 + i]);
    }

    // add constraints for the contacts
    auto contactPositionErrorMap = this->contactPositionError().map(predict_horizon_);

    // convert the eigen matrix into casadi
    // please check https://github.com/casadi/casadi/issues/2563 and
    // https://groups.google.com/forum/#!topic/casadi-users/npPcKItdLN8
    // Assumption: the matrices as stored as column-major
    casadi::DM frictionConeMatrix = casadi::DM::zeros(frictionCone.getA().rows(), frictionCone.getA().cols());
    std::memcpy(frictionConeMatrix.ptr(), frictionCone.getA().data(), sizeof(double) * frictionCone.getA().rows() * frictionCone.getA().cols());
    const casadi::DM zero = casadi::DM::zeros(frictionCone.getA().rows(), 1);
    casadi::MX rotatedFrictionCone;

    for (int i = 0; i < num_legs_; ++i) {
        auto error = contactPositionErrorMap({extractFutureValuesFromState(contact.position),
                                              extractFutureValuesFromState(contact.nominalPosition),
                                              contact.orientation});

        opti_.subject_to(contact.lowerLimitPosition <= error[0] <= contact.upperLimitPosition);

        for (int i = 0; i < predict_horizon_; i++) {
            rotatedFrictionCone = casadi::MX::mtimes(frictionConeMatrix, casadi::MX::reshape(contact.orientation(Sl(), i), 3, 3).T());

            // TODO please if you want to add heel to toe motion you should define a
            // contact.maximumNormalForce for each corner. At this stage is too premature.
            for (const auto &corner: contact.corners) {
                opti_.subject_to(casadi::MX::mtimes(rotatedFrictionCone, corner.force(Sl(), i)) <= zero);

                // limit on the normal force
                opti_.subject_to(0 <= casadi::MX::mtimes(casadi::MX::reshape(contact.orientation(Sl(), i),3,3), corner.force(Sl(), i)(2)));
            }
        }
    }

    // create the cost function
    auto &comReference = this->optiVariables.comReference;

    casadi::DM weightCoMZ = casadi::DM::zeros(1, com.columns());
    double min = this->weights.com(2) / 2;
    for (int i = 0; i < com.columns(); i++) {
        weightCoMZ(Sl(), i) = (this->weights.com(2) - min) * std::exp(-i) + min;
    }

    std::cerr << "------------------_>>> " << weightCoMZ << std::endl;

    // (max - mix) * expo + min

    casadi::MX cost
        = this->weights.angularMomentum * 10 * casadi::MX::sumsqr(angularMomentum(0, Sl()))
            + this->weights.angularMomentum * casadi::MX::sumsqr(angularMomentum(1, Sl()))
            + this->weights.angularMomentum * casadi::MX::sumsqr(angularMomentum(2, Sl()))
            + this->weights.com(0) * casadi::MX::sumsqr(com(0, Sl()) - comReference(0, Sl()))
            + this->weights.com(1) * casadi::MX::sumsqr(com(1, Sl()) - comReference(1, Sl()))
            + casadi::MX::sumsqr(weightCoMZ * (com(2, Sl()) - comReference(2, Sl())));

    casadi::MX averageForce;
    for (const auto &[key, contact]: this->optiVariables.contacts) {
        cost += this->weights.contactPosition * casadi::MX::sumsqr(contact.nominalPosition - contact.position);

        averageForce = casadi::MX::vertcat(
            {contact.isEnable * contact.corners[0].force(0, Sl()) / contact.corners.size(),
             contact.isEnable * contact.corners[0].force(1, Sl()) / contact.corners.size(),
             contact.isEnable * contact.corners[0].force(2, Sl()) / contact.corners.size()});
        for (int i = 1; i < contact.corners.size(); i++) {
            averageForce += casadi::MX::vertcat(
                {contact.isEnable * contact.corners[i].force(0, Sl()) / contact.corners.size(),
                 contact.isEnable * contact.corners[i].force(1, Sl()) / contact.corners.size(),
                 contact.isEnable * contact.corners[i].force(2, Sl())
                     / contact.corners.size()});
        }

        for (const auto &corner: contact.corners) {
            auto forceRateOfChange = casadi::MX::diff(corner.force.T()).T();

            cost += 10 * casadi::MX::sumsqr(corner.force - averageForce);
            cost += this->weights.forceRateOfChange(0) * casadi::MX::sumsqr(forceRateOfChange(0, Sl()));
            cost += this->weights.forceRateOfChange(1) * casadi::MX::sumsqr(forceRateOfChange(1, Sl()));
            cost += this->weights.forceRateOfChange(2) * casadi::MX::sumsqr(forceRateOfChange(2, Sl()));
        }
    }

    this->opti.minimize(cost);

    // prepare the casadi function
    std::vector<casadi::MX> input;
    std::vector<casadi::MX> output;
    std::vector<std::string> inputName;
    std::vector<std::string> outputName;

    input.push_back(this->optiVariables.comCurrent);
    input.push_back(this->optiVariables.dcomCurrent);
    input.push_back(this->optiVariables.angularMomentumCurrent);
    input.push_back(this->optiVariables.comReference);

    inputName.push_back("external_wrench");
    inputName.push_back("com_current");
    inputName.push_back("dcom_current");
    inputName.push_back("angular_momentum_current");
    inputName.push_back("com_reference");

    for (const auto &[key, contact]: this->optiVariables.contacts) {
        input.push_back(contact.currentPosition);
        input.push_back(contact.nominalPosition);
        input.push_back(contact.orientation);
        input.push_back(contact.isEnable);
        input.push_back(contact.upperLimitPosition);
        input.push_back(contact.lowerLimitPosition);

        inputName.push_back("contact_" + key + "_current_position");
        inputName.push_back("contact_" + key + "_nominal_position");
        inputName.push_back("contact_" + key + "_orientation");
        inputName.push_back("contact_" + key + "_is_enable");
        inputName.push_back("contact_" + key + "_upper_limit_position");
        inputName.push_back("contact_" + key + "_lower_limit_position");

        output.push_back(contact.isEnable);
        output.push_back(contact.position);
        output.push_back(contact.orientation);

        outputName.push_back("contact_" + key + "_is_enable");
        outputName.push_back("contact_" + key + "_position");
        outputName.push_back("contact_" + key + "_orientation");

        std::size_t cornerIndex = 0;
        for (const auto &corner: contact.corners) {
            output.push_back(corner.force);
            outputName.push_back("contact_" + key + "_corner_" + std::to_string(cornerIndex) + "_force");
            cornerIndex++;
        }
    }
    controller_ = opti_.to_function("controller", input, output, inputName, outputName);
}

/*!
 * @note
 * system state x_k include {com_pos, com_vel, angular_momentum, contact_pos, contact_enable}.
 * system input u_k include {foot_vel, contact_force}.
 * dynamic_input include x_k and u_k.
 * dynamic_output is x_{k+1} exclude contact_enable.
 */
void CentroidalMPC::CreateSystemDynamic() {
    std::vector<casadi::MX> foot_pos;
    std::vector<casadi::MX> foot_vel;
    std::vector<casadi::MX> contact_force;
    std::vector<casadi::MX> contact_enable;

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

Eigen::VectorXd CentroidalMPC::UpdateMPC(const Eigen::VectorXd &state, const Eigen::VectorXd &des_state, const Eigen::VectorXd &des_inputs) {
    for (std::size_t i = 0; i < num_legs_; i++) {
        contactHandler->getParameter("bounding_box_upper_limit", this->contactBoundingBoxes[contactName].upperLimit)
        contactHandler->getParameter("bounding_box_lower_limit", this->contactBoundingBoxes[contactName].lowerLimit)
    }

    ptr->getParameter("com_weight", this->weights.com);
    ptr->getParameter("contact_position_weight", this->weights.contactPosition);
    ptr->getParameter("force_rate_of_change_weight", this->weights.forceRateOfChange);
    ptr->getParameter("angular_momentum_weight", this->weights.angularMomentum);

    if (!m_pimpl->isInitialized) {
        std::cout << "[CentroidalMPC::setContactPhaseList] The controller is not initialized please call initialize() method." << std::endl;
        return false;
    }

    auto &inputs = m_pimpl->controllerInputs;
    toEigen(inputs.comCurrent) = com;
    toEigen(inputs.dcomCurrent) = dcom;
    toEigen(inputs.angularMomentumCurrent) = angularMomentum;

    toEigen(inputs.externalWrench).setZero();
    m_pimpl->state.externalWrench = Eigen::Vector3d::Zero();

    if (externalWrench.has_value()) {
        toEigen(inputs.externalWrench).leftCols<1>() = externalWrench.value();
        m_pimpl->state.externalWrench = externalWrench.value();
    }

    std::cerr << "external wrench" << std::endl << inputs.externalWrench << std::endl;

    const int stateHorizon = m_pimpl->optiSettings.horizon + 1;

    if (!m_pimpl->isInitialized) {
        std::cout << "[CentroidalMPC::setReferenceTrajectory] The controller is not initialized please call initialize() method." << std::endl;
        return false;
    }

    if (com.rows() != 3) {
        std::cout << "[CentroidalMPC::setReferenceTrajectory] The CoM matrix should have three rows." << std::endl;
        return false;
    }

    if (com.cols() < stateHorizon) {
        std::cout << "[CentroidalMPC::setReferenceTrajectory] The CoM matrix should have at least" << m_pimpl->optiSettings.horizon
                  << " columns. The number of columns is "
                  << "equal to the horizon you set in the  initialization phase." << std::endl;
        return false;
    }

    toEigen(m_pimpl->controllerInputs.comReference) = com.leftCols(stateHorizon);


    using Sl = casadi::Slice;

    if (!m_pimpl->isInitialized) {
        std::cout << "[CentroidalMPC::step] The controller is not initialized please call initialize() method" << std::endl;
        return false;
    }

    // controller:(com_current[3], dcom_current[3], angular_momentum_current[3], com_reference[3x16],
    //            contact_left_foot_current_position[3],  contact_left_foot_nominal_position[3x16], contact_left_foot_orientation[9x15],
    //            contact_left_foot_is_enable[1x15], contact_left_foot_upper_limit_position[3x15], contact_left_foot_lower_limit_position[3x15],
    //            contact_right_foot_current_position[3], contact_right_foot_nominal_position[3x16], contact_right_foot_orientation[9x15],
    //            contact_right_foot_is_enable[1x15], contact_right_foot_upper_limit_position[3x15],  contact_right_foot_lower_limit_position[3x15])
    //             -------------------->
    //            (contact_left_foot_position[3x16], contact_left_foot_is_enable[1x15], contact_left_foot_corner_0_force[3x15],
    //                                                                                  contact_left_foot_corner_1_force[3x15],
    //                                                                                  contact_left_foot_corner_2_force[3x15],
    //                                                                                  contact_left_foot_corner_3_force[3x15],
    //             contact_right_foot_position[3x16], contact_right_foot_is_enable[1x15], contact_right_foot_corner_0_force[3x15],
    //                                                                                    contact_right_foot_corner_1_force[3x15],
    //                                                                                    contact_right_foot_corner_2_force[3x15],
    //                                                                                    contact_right_foot_corner_3_force[3x15])

    const auto &inputs = m_pimpl->controllerInputs;

    std::vector<casadi::DM> vectorizedInputs;
    vectorizedInputs.push_back(inputs.externalWrench);
    vectorizedInputs.push_back(inputs.comCurrent);
    vectorizedInputs.push_back(inputs.dcomCurrent);
    vectorizedInputs.push_back(inputs.angularMomentumCurrent);
    vectorizedInputs.push_back(inputs.comReference);

    for (const auto &[key, contact]: inputs.contacts) {
        vectorizedInputs.push_back(contact.currentPosition);
        vectorizedInputs.push_back(contact.nominalPosition);
        vectorizedInputs.push_back(contact.orientation);
        vectorizedInputs.push_back(contact.isEnable);
        vectorizedInputs.push_back(contact.upperLimitPosition);
        vectorizedInputs.push_back(contact.lowerLimitPosition);
    }

    // compute the output
    auto controllerOutput = m_pimpl->controller(vectorizedInputs);

    // get the solution
    auto it = controllerOutput.begin();
    m_pimpl->state.nextPlannedContact.clear();
    for (auto &[key, contact]: m_pimpl->state.contacts) {
        // the first output tell us if a contact is enabled
        log()->info("activation sequence {}", toEigen(*it));

        int index = toEigen(*it).size();
        const int size = toEigen(*it).size();
        for (int i = 0; i < size; i++) {
            if (toEigen(*it)(i) > 0.5) {
                if (i == 0) {
                    break;
                } else if (toEigen(*it)(i - 1) < 0.5) {
                    index = i;
                    break;
                }
            }
        }

        double isEnable = toEigen(*it)(0);
        std::advance(it, 1);
        contact.pose.translation(toEigen(*it).leftCols<1>());

        if (index < size) {
            m_pimpl->state.nextPlannedContact[key].name = key;
            m_pimpl->state.nextPlannedContact[key].pose.translation(toEigen(*it).col(index));
        }

        std::advance(it, 1);
        contact.pose.quat(Eigen::Quaterniond(
            Eigen::Map<const Eigen::Matrix3d>(toEigen(*it).leftCols<1>().data())));

        if (index < size) {
            m_pimpl->state.nextPlannedContact[key].pose.quat(Eigen::Quaterniond(
                Eigen::Map<const Eigen::Matrix3d>(toEigen(*it).leftCols<1>().data())));

            // this is wrong but if I don‘t put index + 1 I get strange values when index = 1
            const double nextPlannedContactTime
                = m_pimpl->currentTime + m_pimpl->optiSettings.samplingTime * (index + 1);
            auto nextPlannedContact = m_pimpl->contactPhaseList.lists().at(key).getPresentContact(nextPlannedContactTime);
            if (nextPlannedContact == m_pimpl->contactPhaseList.lists().at(key).end()) {
                std::cout << "[CentroidalMPC::step] Unable to get the next planned contact." << std::endl;
                return false;
            }

            log()->warn(
                "[CentroidalMPC] key {} next planned contact pose {} activation time {} deactivation time {} next planned contact time {} index {}",
                key,
                m_pimpl->state.nextPlannedContact[key].pose.translation().transpose(),
                nextPlannedContact->activateTime,
                nextPlannedContact->deactivateTime,
                nextPlannedContactTime, index);

            m_pimpl->state.nextPlannedContact[key].activateTime = nextPlannedContact->activateTime;
            m_pimpl->state.nextPlannedContact[key].deactivateTime = nextPlannedContact->deactivateTime;
            m_pimpl->state.nextPlannedContact[key].index = nextPlannedContact->index;
            m_pimpl->state.nextPlannedContact[key].type = nextPlannedContact->type;
        }

        std::advance(it, 1);

        for (auto &corner: contact.corners) {
            // isEnable == 1 means that the contact is active
            if (isEnable > 0.5) {
                corner.force = toEigen(*it).leftCols<1>();
            } else {
                corner.force.setZero();
            }
            std::advance(it, 1);
        }
    }

    // advance the time
    current_time_ += time_step_;
    return {};
}

