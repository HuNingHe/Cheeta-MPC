#include "GaitScheduler.h"
#include <vector>

/*
void GaitScheduler::step() {
    if (new_period_count != 0) { // 有更新步态周期指令在
        if (iteration * dt <= new_period * new_period_count + 0.002 && iteration * dt >= new_period * new_period_count - 0.002){ // 更新的步态周期已经执行完成了
            new_period_count = 0;
            new_period = 0;
            updateInitPhase(gaitData.initial_leg_phase,new_init_leg_phase,
                            gaitData.initial_state_ratio_in_cycle, new_init_state_ratio_in_cycle,
                            new_init_leg_state);
            new_init_leg_state = Vector4i::Zero();
            new_init_state_ratio_in_cycle = Vector4d::Zero();
            new_init_leg_phase = Vector4d::Zero();
        } else {
            for (int leg = 0; leg < 4; ++leg) {
                double current_time = dt * iteration + new_init_leg_phase[leg] * new_period;
                gaitData.current_leg_phase[leg] = fmod(current_time, new_period) / new_period;
                if (gaitData.current_leg_phase[leg] <= new_init_state_ratio_in_cycle[leg]){
                    gaitData.current_leg_state[leg] = new_init_leg_state[leg]; // 暂未发生相位切换
                } else {
                    gaitData.current_leg_state[leg] = 1 - new_init_leg_state[leg]; // 发生相位切换
                }

                if (new_init_leg_state[leg]){
                    gaitData.swing_sub_phase[leg] = (gaitData.current_leg_phase[leg] - gaitData.duty_factor) / (1 - gaitData.duty_factor);
                    gaitData.stance_sub_phase[leg] = gaitData.current_leg_phase[leg] / gaitData.duty_factor;
                } else {
                    gaitData.swing_sub_phase[leg] = gaitData.current_leg_phase[leg] / (1 - gaitData.duty_factor);
                    gaitData.stance_sub_phase[leg] = (gaitData.current_leg_phase[leg] - (1 - gaitData.duty_factor)) / gaitData.duty_factor;
                }

                if (gaitData.current_leg_state[leg]){
                    gaitData.swing_sub_phase[leg] = 0.0;
                }
                if (!gaitData.current_leg_state[leg]){
                    gaitData.stance_sub_phase[leg] = 0.0;
                }
            }
        }
    }
    iteration++;
}

void GaitScheduler::updateGaitPeriod(unsigned int _new_period_count, double _new_period) {*/
/* 对于步态周期可能不断变化的情况, 现在有三个选项:
 * 1、改变步态周期后持续保持MPC预测时域个新的步态周期, 不要立马恢复为原来的步态周期, 暂时不考虑这样
 * 2、根据机器人状态确定, 机器人执行几个新的步态周期, 然后恢复之前的, 这是本代码的思路
 * 3、根据MPC的状态矩阵, 预测机器人状态, 以此预测未来步态周期, 这将导致求解预测时域个QP问题, 会导致机器人
 */
/*
    assert(_new_period > 0);
    if (new_period_count == 0) {
        iteration = 0; // 计时直接清零，从头开始计时
        new_period = _new_period;
        new_period_count = _new_period_count;
        new_init_leg_state = gaitData.current_leg_state;

        updateInitPhase(new_init_leg_phase, gaitData.initial_leg_phase,
                        new_init_state_ratio_in_cycle, gaitData.initial_state_ratio_in_cycle,
                        gaitData.initial_leg_state);
    }
}

Eigen::MatrixXi GaitScheduler::predictContactStates(unsigned int horizon, double time_per_horizon) {
    Eigen::MatrixXi mpc_table = Eigen::MatrixXi::Zero(horizon, 4);
    Eigen::Vector4i predict_leg_state = Eigen::Vector4i::Zero();
    Eigen::Vector4d predict_phase = Eigen::Vector4d::Zero();

    unsigned long long int cur_iteration = iteration;
    // 试着让current_time增加horizon * time_per_horizon就好了

    if (time_per_horizon * horizon < new_period * new_period_count) {
        for (int i = 0; i < horizon; ++i) {
            for (int leg = 0; leg < 4; ++leg) {
                double predict_time = dt * cur_iteration + new_init_leg_phase[leg] * new_period + i * time_per_horizon;
                predict_phase[leg] = fmod(predict_time, new_period) / new_period;
                if (predict_phase[leg] <= new_init_state_ratio_in_cycle[leg]){
                    predict_leg_state[leg] = new_init_leg_state[leg]; // 暂未发生相位切换
                } else {
                    predict_leg_state[leg] = 1 - new_init_leg_state[leg]; // 发生相位切换
                }
            }
            mpc_table.row(i) = predict_leg_state;
        }
    } else { // 这种情况比较复杂
        int new_period_horizon = floor(time_per_horizon * horizon / new_period * new_period_count);
        double time_remain = fmod(time_per_horizon * horizon, new_period * new_period_count);
        for (int i = 0; i < new_period_horizon; ++i) {
            for (int leg = 0; leg < 4; ++leg) {
                double predict_time = dt * cur_iteration + new_init_leg_phase[leg] * new_period + i * time_per_horizon;
                predict_phase[leg] = fmod(predict_time, new_period) / new_period;
                if (predict_phase[leg] <= new_init_state_ratio_in_cycle[leg]){
                    predict_leg_state[leg] = new_init_leg_state[leg]; // 暂未发生相位切换
                } else {
                    predict_leg_state[leg] = 1 - new_init_leg_state[leg]; // 发生相位切换
                }
            }
            mpc_table.row(i) = predict_leg_state;
        }

        Eigen::Vector4i tmp_leg_state(0, 0, 0, 0);
        for (int leg = 0; leg < 4; ++leg) {
            double tmp_time = dt * cur_iteration + new_init_leg_phase[leg] * new_period + time_remain + (new_period_horizon - 1) * time_per_horizon;
            predict_phase[leg] = fmod(tmp_time, new_period) / new_period;
            if (predict_phase[leg] <= new_init_state_ratio_in_cycle[leg]){
                tmp_leg_state[leg] = new_init_leg_state[leg]; // 暂未发生相位切换
            } else {
                tmp_leg_state[leg] = 1 - new_init_leg_state[leg]; // 发生相位切换
            }
        }

        std::vector<int> current_swing;
        Eigen::Vector4d new_tmp_init_phase = new_init_leg_phase;
        Eigen::Vector4i new_tmp_init_state = tmp_leg_state;
        Eigen::Vector4d new_tmp_init_ratio(0, 0, 0, 0);

        for (int i = 0; i < 4; ++i) {
            if (tmp_leg_state[i] == 0){
                current_swing.push_back(i);
            }
            if (tmp_leg_state[i] == new_init_leg_state[i]){
                new_tmp_init_ratio[i] = new_init_state_ratio_in_cycle[i];
            } else {
                new_tmp_init_ratio[i] = 1 - new_init_state_ratio_in_cycle[i];
            }
        }

        if (gaitData._currentGait == GaitType::STATIC_WALK){
            switch (current_swing[0]) {
                case 1: // 当前第二条腿摆动
                    new_tmp_init_phase[0] = 0.2;
                    new_tmp_init_phase[1] = 0.0;
                    new_tmp_init_phase[2] = 0.4;
                    new_tmp_init_phase[3] = 0.6;
                    break;
                case 2:
                    new_tmp_init_phase[0] = 0.6;
                    new_tmp_init_phase[1] = 0.4;
                    new_tmp_init_phase[2] = 0.0;
                    new_tmp_init_phase[3] = 0.2;
                    break;
                case 3:
                    new_tmp_init_phase[0] = 0.4;
                    new_tmp_init_phase[1] = 0.2;
                    new_tmp_init_phase[2] = 0.6;
                    new_tmp_init_phase[3] = 0.0;
                    break;
                default:
                    break;
            }
        }

        for (int i = new_period_horizon; i < horizon; ++i) {
            for (int leg = 0; leg < 4; ++leg) {
                double predict_time = new_tmp_init_phase[leg] * gaitData.stance_duration / gaitData.duty_factor + (i - new_period_horizon) * time_per_horizon;
                predict_phase[leg] = fmod(predict_time, gaitData.stance_duration / gaitData.duty_factor) * gaitData.duty_factor / gaitData.stance_duration;
                if (predict_phase[leg] <= new_tmp_init_ratio[leg]){
                    predict_leg_state[leg] = new_tmp_init_state[leg]; // 暂未发生相位切换
                } else {
                    predict_leg_state[leg] = 1 - new_tmp_init_state[leg]; // 发生相位切换
                }
            }
            mpc_table.row(i) = predict_leg_state;
        }
    }
    return mpc_table;
}

void GaitScheduler::updateInitPhase(Eigen::Vector4d &init_phase, Eigen::Vector4d &old_init_phase, Eigen::Vector4d &init_ratio, Eigen::Vector4d &old_init_ratio, Eigen::Vector4i &old_init_state) {
    std::vector<int> current_swing;
    init_phase = old_init_phase;
    for (int i = 0; i < 4; ++i) {
        if (gaitData.current_leg_state[i] == 0){
            current_swing.push_back(i);
        }
        if (gaitData.current_leg_state[i] == old_init_state[i]){
            init_ratio[i] = old_init_ratio[i];
        } else {
            init_ratio[i] = 1 - old_init_ratio[i];
        }
    }
}
*/


using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::Vector4d;
using Eigen::Vector4i;
using Eigen::MatrixXi;

GaitController::GaitController(double _dt) {
    current_gait = GaitType::STAND;
    next_gait = GaitType::STAND;
    dt = _dt;
    iteration = 0;
    phase = Vector4d::Zero();
    stance_sub_phase = Vector4d::Zero();
    swing_sub_phase = Vector4d::Ones();
    contact_states = Vector4i::Ones();
    almost_finished_swing = Vector4i::Zero();
    createGait();
}

GaitController::GaitController(double _dt, Vector4d _duty, Vector4d _offset, Vector4d Tst1, GaitType gait_type) {
    dt = _dt;
    iteration = 0;
    phase = Vector4d::Zero();
    stance_sub_phase = Vector4d::Zero();
    swing_sub_phase = Vector4d::Ones();
    contact_states = Vector4i::Ones();

    duty = std::move(_duty);
    offset = std::move(_offset);
    Tst = std::move(Tst1);
    current_gait = gait_type;
    next_gait = gait_type;
}

void GaitController::createGait() {
    switch(next_gait){
        case GaitType::STAND: {
            for (int i = 0; i < 4; ++i) {
                Tst(i) = 1;
                duty(i) = 1;
                offset(i) = 0;
            }
            break;
        }
        case GaitType::TROT: {
            Tst << 0.15, 0.15, 0.15, 0.15;
            duty << 0.5, 0.5, 0.5, 0.5;
            offset << 0.5, 0, 0.5, 0;
            break;
        }
        case GaitType::STATIC_WALK: {
            Tst << 0.2, 0.2, 0.2, 0.2;
            duty << 0.75, 0.75, 0.75, 0.75;
            offset << 0.75, 0.5, 0.25, 0;
            break;
        }
        case GaitType::PACE: {
            Tst << 0.2, 0.2, 0.2, 0.2;
            duty << 0.5, 0.5, 0.5, 0.5;
            offset << 0.5, 0, 0, 0.5;
            break;
        }
        case GaitType::PRONK: {
            Tst << 0.2, 0.2, 0.2, 0.2;
            duty << 0.5, 0.5, 0.5, 0.5;
            offset << 0, 0, 0, 0;
            break;
        }
        case GaitType::BOUND: {
            Tst << 0.2, 0.2, 0.2, 0.2;
            duty << 0.5, 0.5, 0.5, 0.5;
            offset << 0.5, 0.5, 0, 0;
            break;
        }
    }
    current_gait = next_gait;
}

void GaitController::changeGaitType(GaitType gait_type) {
    if (current_gait != gait_type){
        next_gait = gait_type;
        iteration = 0;
    }
    createGait();
}

void GaitController::step() {
    double near_stance = 0.1; // 摆动腿马上要切换为支撑腿了，这个时候也没必要修改迈步周期了
    double current_time = iteration * dt;
    for (int i = 0; i < 4; ++i) {
        double T = Tst(i) / duty(i);  // gait period
        phase(i) = fmod(fmod(current_time, T) / T + offset(i), 1);
        if (phase(i) > duty(i)){
            contact_states(i) = 0;
            swing_sub_phase(i) = (phase(i) - duty(i)) / (1 - duty(i));
            stance_sub_phase(i) = 0;
            if (phase(i) - duty(i) < near_stance)
                almost_finished_swing(i) = 1;
        } else {
            contact_states(i) = 1;
            swing_sub_phase(i) = 0;
            stance_sub_phase(i) = phase(i) / duty(i);
            almost_finished_swing(i) = 0;
        }
    }
    iteration++;
}

MatrixXi GaitController::predictContactStates(unsigned int horizon, double time_per_horizon) {
    MatrixXi mpc_table = MatrixXi::Zero(horizon, 4);
    Vector4i predict_leg_state = Eigen::Vector4i::Zero();
    Vector4d predict_phase = Eigen::Vector4d::Zero();
    unsigned long long int cur_iteration = iteration;

    for (unsigned int i = 0; i < horizon; ++i) {
        for (int leg = 0; leg < 4; ++leg) {
            double T = Tst(leg) / duty(leg);
            double predict_time = dt * cur_iteration + i * time_per_horizon;
            predict_phase[leg] = fmod(fmod(predict_time, T) / T + offset(leg), 1);
            if (predict_phase[leg] > duty(leg)){
                predict_leg_state[leg] = 0;
            } else {
                predict_leg_state[leg] = 1;
            }
        }
        mpc_table.row(i) = predict_leg_state;
    }
    return mpc_table;
}

Vector4d& GaitController::getSwingSubPhase() {
    return swing_sub_phase;
}

Vector4d& GaitController::getStanceSubPhase() {
    return stance_sub_phase;
}

Vector4d GaitController::getSwingDuration() {
    Vector4d Tsw = Vector4d::Zero();
    for (int i = 0; i < 4; ++i) {
        Tsw(i) = Tst(i) * (1 / duty(i) - 1);
    }
    return Tsw;
}

Vector4d &GaitController::getStanceDuration() {
    return Tst;
}

void GaitController::updateGaitPeriod(double new_Tsw, int new_Tsw_count) {
    for (int i = 0; i < 4; ++i) {
        if(contact_states(i) <= 0) {

        }
        if(new_Tsw < Tst(i) * (1 / duty(i) - 1)) {
            continue;
        }
    }
}

Vector4i &GaitController::getContactStates() {
    return contact_states;
}
