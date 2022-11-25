#include <iostream>
#include <Timer.h>
#include "MPC/include/MPCLocomotion.h"
#include "Gait.h"
#include "MathTypes.h"

MPCLocomotion::MPCLocomotion(double _dt, int _iterations_between_mpc, ControlFSMData *data) :
        iterationsBetweenMPC(_iterations_between_mpc),
        horizonLength(10), // 10 for convexMPC
        dt(_dt),
        trotting(horizonLength, Vec4<int>(5, 0, 5, 0), Vec4<int>(5, 5, 5, 5), "Trotting"),
        bounding(horizonLength, Vec4<int>(5, 5, 0, 0), Vec4<int>(4, 4, 4, 4), "Bounding"),
        pronking(horizonLength, Vec4<int>(0, 0, 0, 0), Vec4<int>(4, 4, 4, 4), "Pronking"),
        galloping(horizonLength, Vec4<int>(2, 0, 7, 9), Vec4<int>(4, 4, 4, 4), "Galloping"),
        standing(horizonLength, Vec4<int>(0, 0, 0, 0), Vec4<int>(10, 10, 10, 10), "Standing"),
        trotRunning(horizonLength, Vec4<int>(5, 0, 5, 0), Vec4<int>(4, 4, 4, 4), "Trot Running"),
        walking(horizonLength, Vec4<int>(3, 0, 5, 8), Vec4<int>(5, 5, 5, 5), "Walking"),
        pacing(horizonLength, Vec4<int>(0, 5, 5, 0), Vec4<int>(5, 5, 5, 5), "Pacing"){
    dtMPC = dt * iterationsBetweenMPC;
    default_iterations_between_mpc = iterationsBetweenMPC;
    printf("[MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, iterationsBetweenMPC, dtMPC);
    rpy_comp[0] = 0;
    rpy_comp[1] = 0;
    rpy_comp[2] = 0;
    rpy_int[0] = 0;
    rpy_int[1] = 0;
    rpy_int[2] = 0;
    _yaw_des = 0;

    for (bool & i : firstSwing)
        i = true;

    pBody_des.setZero();
    vBody_des.setZero();
    aBody_des.setZero();

    Eigen::VectorXd friction = Eigen::VectorXd::Zero(4);
    friction << 0.8, 0.8, 0.8, 0.8;
    Eigen::Vector3d weights_input;
    weights_input << 0.000005, 0.000005, 0.000005;
    convex_mpc = new ConvexMPC(data->_model->quadruped->body_mass, data->_model->quadruped->body_inertial, 4,
                               horizonLength, dtMPC, data->controlParameters->mpc_weights, weights_input, friction);

    rf_mpc = new RF_MPC(data->_model->quadruped->body_mass, data->_model->quadruped->body_inertial, 4,
                        horizonLength, dtMPC, data->controlParameters->rf_mpc_weights, data->controlParameters->rf_mpc_terminal_weights,
                        data->controlParameters->rf_mpc_input_weights, friction);
}

void MPCLocomotion::initialize() {
    for (bool & i : firstSwing) i = true;
    firstRun = true;
}

void MPCLocomotion::recompute_timing(int iterations_per_mpc) {
    iterationsBetweenMPC = iterations_per_mpc;
    dtMPC = dt * iterations_per_mpc;
}

void MPCLocomotion::SetupCommand(ControlFSMData &data) {
    _body_height = 0.29;
    double x_vel_cmd, y_vel_cmd;
    double filter = 0.01;
    _yaw_turn_rate = -1 * data._desiredStateCommand->rightStickAnalog[0];
    x_vel_cmd = -1.2 * data._desiredStateCommand->leftStickAnalog[1];
    y_vel_cmd = -1.0 * data._desiredStateCommand->leftStickAnalog[0];
    _x_vel_des = _x_vel_des * (1 - filter) + x_vel_cmd * filter;
    _y_vel_des = _y_vel_des * (1 - filter) + y_vel_cmd * filter;
//    std::cout << "x vel:\n" << _x_vel_des << "y vel:\n" << _y_vel_des << std::endl;
    _roll_des = 0.;
    _pitch_des = 0.;

    if (data._desiredStateCommand->a){
        gaitNumber = 0;
    } else if (data._desiredStateCommand->b){
        gaitNumber = 1;
    } else if (data._desiredStateCommand->x){
        gaitNumber = 2;
    } else if (data._desiredStateCommand->y){
        gaitNumber = 3;
    } else if (data._desiredStateCommand->LB){
        gaitNumber = 4;
    } else if (data._desiredStateCommand->RB){
        gaitNumber = 5;
    } else if (data._desiredStateCommand->logitech){
        gaitNumber = 6;
    } else {
        gaitNumber = 7;
    }
}

void MPCLocomotion::run(ControlFSMData &data) {
    SetupCommand(data);
    auto &seResult = data._stateEstimator->getResult();

    if (((gaitNumber == 4) && current_gait != 4) || firstRun) {
        world_position_desired[0] = seResult.position[0];
        world_position_desired[1] = seResult.position[1];
        stand_traj[0] = seResult.position[0];
        stand_traj[1] = seResult.position[1];
        stand_traj[2] = 0.28;
        stand_traj[3] = 0;
        stand_traj[4] = 0;
        stand_traj[5] = seResult.rpy[2];
    }

    Gait* gait = &trotting;
    if (gaitNumber == 0)
        gait = &bounding;
    else if (gaitNumber == 1)
        gait = &pronking;
    else if (gaitNumber == 3)
        gait = &trotRunning;
    else if (gaitNumber == 4)
        gait = &standing;
    else if(gaitNumber == 6)
        gait = &pacing;
    current_gait = gaitNumber;

    gait->setIterations(iterationsBetweenMPC, iterationCounter);
    recompute_timing(default_iterations_between_mpc);
    if (_body_height < 0.02) {
        _body_height = 0.28;
    }

    Vec3<double> v_des_robot(_x_vel_des, _y_vel_des, 0);
    Vec3<double> v_des_world = seResult.rBody * v_des_robot;
    Vec3<double> v_robot = seResult.vWorld;

    if (fabs(v_robot[0]) > 0.2){  // 避免后腿拖地
        rpy_int[1] += dt * (_pitch_des - seResult.rpy[1]) / v_robot[0];
    }
    if (fabs(v_robot[1]) > 0.1) {
        rpy_int[0] += dt * (_roll_des - seResult.rpy[0]) / v_robot[1];
    }

    rpy_int[0] = fmin(fmax(rpy_int[0], -0.4), 0.4);
    rpy_int[1] = fmin(fmax(rpy_int[1], -0.4), 0.4); // 积分饱和
    rpy_comp[1] = v_robot[0] * rpy_int[1];
    rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber != 1);  // pronking 没有后腿和侧面腿拖地的烦恼
//    std::cout << rpy_comp << std::endl;
    for (int i = 0; i < 4; i++) {
        pFoot[i] = seResult.position + seResult.rBody * (data._model->quadruped->hipLocationInKinematic.row(i).transpose() + data._legController->datas[i].p);
    }

    if (gait != &standing) {
        _yaw_des += dt * _yaw_turn_rate;
        if (abs(_yaw_des - M_PI) < 1e-2 || abs(_yaw_des + M_PI) < 1e-2) {
            _yaw_des = seResult.rpy[2] + dt * _yaw_turn_rate; // 原版MIT代码没有加这个判断条件，会发散，实测会发散
        }
        world_position_desired += dt * Vec3<double>(v_des_world[0], v_des_world[1], 0);
    }

    // some first time initialization
    if (firstRun) {
        world_position_desired[0] = seResult.position[0];
        world_position_desired[1] = seResult.position[1];
        world_position_desired[2] = seResult.rpy[2];

        for (int i = 0; i < 4; i++) {
            footSwingTrajectories[i].setHeight(0.05);
            footSwingTrajectories[i].setInitialPosition(pFoot[i]);
            footSwingTrajectories[i].setFinalPosition(pFoot[i]);
        }
        firstRun = false;
    }

    for (int l = 0; l < 4; l++)
        swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);

    double v_abs = std::fabs(v_des_robot[0]);
    for (int i = 0; i < 4; i++) {
        if (firstSwing[i]) {
            swingTimeRemaining[i] = swingTimes[i];
        } else {
            swingTimeRemaining[i] -= dt;
        }
        footSwingTrajectories[i].setHeight(0.05);
        Vec3<double> pRobotFrame = data._model->quadruped->hipLocationInKinematic.row(i).transpose();

        double stance_time = gait->getCurrentStanceTime(dtMPC, i);
        Vec3<double> pYawCorrected = Eigen::AngleAxisd(-_yaw_turn_rate * stance_time / 2, Vector3d::UnitZ()) * pRobotFrame;

        Vec3<double> des_vel;
        des_vel[0] = _x_vel_des;
        des_vel[1] = _y_vel_des;
        des_vel[2] = 0.0;

        Vec3<double> Pf = seResult.position + seResult.rBody * (pYawCorrected + des_vel * swingTimeRemaining[i]);

        double p_rel_max = 0.32f;

        double pfx_rel = seResult.vWorld[0] * 0.5 * stance_time +
                         0.03f * (seResult.vWorld[0] - v_des_world[0]) +
                         (0.5f * seResult.position[2] / 9.81) * (seResult.vWorld[1] * _yaw_turn_rate);

        double pfy_rel = seResult.vWorld[1] * 0.5 * stance_time * dtMPC +
                         0.03f * (seResult.vWorld[1] - v_des_world[1]) +
                         (0.5f * seResult.position[2] / 9.81) * (-seResult.vWorld[0] * _yaw_turn_rate);
        pfx_rel = fmin(fmax(pfx_rel, -p_rel_max), p_rel_max);
        pfy_rel = fmin(fmax(pfy_rel, -p_rel_max), p_rel_max);
        Pf[0] += pfx_rel;
        Pf[1] += pfy_rel;
        Pf[2] = 0;
        footSwingTrajectories[i].setFinalPosition(Pf);
    }

    iterationCounter++;

    Kp << 500, 0, 0,
            0, 500, 0,
            0, 0, 300;
    Kp_stance = 0 * Kp;
    Kd << 17, 0, 0,
          0, 17, 0,
          0, 0, 20;
    Kd_stance = Kd;

    Vec4<double> contactStates = gait->getContactState();
    Vec4<double> swingStates = gait->getSwingState();
    Eigen::MatrixXi mpcTable = gait->getMpcTable();
    updateMPCIfNeeded(mpcTable, data);
    Vec4<double> se_contactState(0, 0, 0, 0);

    for (int foot = 0; foot < 4; foot++) {
        double contactState = contactStates[foot];
        double swingState = swingStates[foot];
        if (swingState > 0){
            if (firstSwing[foot]) {
                firstSwing[foot] = false;
                footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
            }

//            footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
            footSwingTrajectories[foot].computeSwingTrajectoryCycloid(swingState, swingTimes[foot]);

            Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition();
            Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
            Vec3<double> pDesLeg = seResult.rBody.transpose() * (pDesFootWorld - seResult.position) - data._model->quadruped->hipLocationInKinematic.row(foot).transpose();
            Vec3<double> vDesLeg = seResult.rBody.transpose() * (vDesFootWorld - seResult.vWorld);

            // Update for WBC
            pFoot_des[foot] = pDesFootWorld;// + data._model->quadruped->body_com;
            vFoot_des[foot] = vDesFootWorld;
            aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();

            if (!data.controlParameters->use_wbc) {
                // Update leg control command regardless of the usage of WBIC
                data._legController->commands[foot].pDes = pDesLeg;
                data._legController->commands[foot].vDes = vDesLeg;
                data._legController->commands[foot].kpCartesian = Kp;
                data._legController->commands[foot].kdCartesian = Kd;
            }
        } else{ // foot is in stance
            firstSwing[foot] = true;

            Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition();
            Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
            Vec3<double> pDesLeg = seResult.rBody.transpose() * (pDesFootWorld - seResult.position) - data._model->quadruped->hipLocationInKinematic.row(foot).transpose();
            Vec3<double> vDesLeg = seResult.rBody.transpose() * (vDesFootWorld - seResult.vWorld);

            if (!data.controlParameters->use_wbc) {
                data._legController->commands[foot].pDes = pDesLeg;
                data._legController->commands[foot].vDes = vDesLeg;
                data._legController->commands[foot].kpCartesian = Kp_stance;
                data._legController->commands[foot].kdCartesian = Kd_stance;

                data._legController->commands[foot].forceFeedForward = f_ff[foot];
                data._legController->commands[foot].kdJoint = Mat3<double>::Identity() * 0.2;
            } else { // Stance foot damping
                data._legController->commands[foot].pDes = pDesLeg;
                data._legController->commands[foot].vDes = vDesLeg;
                data._legController->commands[foot].kpCartesian = 0 * Kp_stance;
                data._legController->commands[foot].kdCartesian = Kd_stance;
            }
            se_contactState[foot] = contactState;
        }
    }

    data._stateEstimator->setContactPhase(se_contactState);
    // Update For WBC
    pBody_des[0] = world_position_desired[0];
    pBody_des[1] = world_position_desired[1];
    pBody_des[2] = _body_height;

    vBody_des[0] = v_des_world[0];
    vBody_des[1] = v_des_world[1];
    vBody_des[2] = 0.;

    aBody_des.setZero();

    pBody_RPY_des[0] = 0.0;
    pBody_RPY_des[1] = 0.0;
    pBody_RPY_des[2] = _yaw_des;

    vBody_Ori_des[0] = 0.;
    vBody_Ori_des[1] = 0.;
    vBody_Ori_des[2] = _yaw_turn_rate;
    contact_state = gait->getContactState();
    // END of WBC Update
}

void MPCLocomotion::updateMPCIfNeeded(Eigen::MatrixXi &mpcTable, ControlFSMData &data) {
    if ((iterationCounter % iterationsBetweenMPC) == 0) {
        auto seResult = data._stateEstimator->getResult();
        Vec3<double> v_des_robot(_x_vel_des, _y_vel_des, 0);
        Vec3<double> des_rpy, des_pos, v_des_world, des_ang_vel;
        if (current_gait == 4) {
            des_rpy = Vec3<double>(_roll_des, _pitch_des, stand_traj[5]);
            des_pos = Vec3<double>(stand_traj[0], stand_traj[1], _body_height);
            des_ang_vel = Vec3<double>::Zero();
            v_des_world = Vec3<double>::Zero();
        } else {
            const double max_pos_error = 0.1;
            double xStart = world_position_desired[0];
            double yStart = world_position_desired[1];

            if (xStart - seResult.position[0] > max_pos_error) xStart = seResult.position[0] + max_pos_error;
            if (seResult.position[0] - xStart > max_pos_error) xStart = seResult.position[0] - max_pos_error;

            if (yStart - seResult.position[1] > max_pos_error) yStart = seResult.position[1] + max_pos_error;
            if (seResult.position[1] - yStart > max_pos_error) yStart = seResult.position[1] - max_pos_error;

            world_position_desired[0] = xStart;
            world_position_desired[1] = yStart;

            v_des_world = seResult.rBody * v_des_robot;
            v_des_world[2] = 0;
            des_ang_vel = Vec3<double>(0, 0, _yaw_turn_rate);
            des_pos = Vec3<double>(xStart, yStart, _body_height);
            des_rpy = Vec3<double>(rpy_comp[0], rpy_comp[1], _yaw_des);
        }

        Eigen::VectorXd friction_coeff = Eigen::VectorXd::Zero(4);
        friction_coeff << 0.8, 0.8, 0.8, 0.8;

        Vec3<double> pxy_des(world_position_desired[0], world_position_desired[1], 0);
        double pz_err = seResult.position[2] - _body_height;
        Vec3<double> vxy(seResult.vWorld[0], seResult.vWorld[1], 0);

        Eigen::VectorXd init_state = Eigen::VectorXd::Zero(24);
        init_state << seResult.rpy, seResult.position, seResult.omegaWorld, seResult.vWorld, pFoot[0], pFoot[1], pFoot[2], pFoot[3];
        Eigen::VectorXd des_state = Eigen::VectorXd::Zero(12);
        des_state << des_rpy, des_pos, des_ang_vel, v_des_world;

        Eigen::VectorXd des_input = Eigen::VectorXd::Zero(horizonLength * 4);
        for (int i = 0; i < horizonLength; ++i) {
            for (int j = 0; j < 4; ++j) {
                des_input[i * 4 + j] = mpcTable(i, j);
            }
        }

        Eigen::VectorXd f;
        if (data.controlParameters->use_rf_mpc) {
            rf_mpc->UpdateFriction(friction_coeff);
            mpc_solver = dynamic_cast<LinearMPC *>(rf_mpc);
            mpc_solver->UpdateStatesWeights(data.controlParameters->rf_mpc_weights);
            f = -mpc_solver->UpdateMPC(init_state, des_state, des_input);
        } else {
            convex_mpc->UpdateXdrag(x_comp_integral);
            convex_mpc->UpdateFriction(friction_coeff);
            mpc_solver = dynamic_cast<LinearMPC *>(convex_mpc);
            mpc_solver->UpdateStatesWeights(data.controlParameters->mpc_weights);
            f = mpc_solver->UpdateMPC(init_state, des_state, des_input);
        }

        if (vxy[0] > 0.3 || vxy[0] < -0.3) {
            x_comp_integral += data.controlParameters->cmpc_x_drag * pz_err * dtMPC / vxy[0];
        }

        for (int leg = 0; leg < 4; leg++) {
            f_ff[leg] = seResult.rBody.transpose() * f.segment(leg * 3, 3);
            // Update for WBC
            Fr_des[leg] = -f.segment(leg * 3, 3);
        }
    }
}
