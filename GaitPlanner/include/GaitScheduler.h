#ifndef PROJECT_GAIT_SCHEDULER_H
#define PROJECT_GAIT_SCHEDULER_H

/*!
 * @author HuNing-He
 * @email 2689112371@qq.com
 * @date 2022-5-24
 * @file GaitScheduler.h
 * @origin from CheetahSoftware, but made many modifications to fit my algorithm
 */

#include <eigen3/Eigen/Dense>

/*!
 * 此处不考虑复杂的步态, 并且如何进行优雅的步态转换并不是我的研究方向,
 * 仅实现了一个简单的开环步态调度器, 并没有考虑触地检测
 */
enum class GaitType: unsigned short int{
    STAND,
    STATIC_WALK,
    TROT,
    PACE,
    BOUND,
    PRONK
};

class GaitController{
private:
    Eigen::Vector4d phase;
    Eigen::Vector4d duty;
    Eigen::Vector4d offset;
    Eigen::Vector4d Tst;
    Eigen::Vector4d swing_sub_phase;
    Eigen::Vector4d stance_sub_phase;
    Eigen::Vector4i contact_states;
    Eigen::Vector4i almost_finished_swing;

    unsigned long long int iteration = 0;
    GaitType current_gait;
    GaitType next_gait;
    void createGait();

public:
    double dt;
    explicit GaitController(double _dt);
    GaitController(double _dt, Eigen::Vector4d _duty, Eigen::Vector4d _offset, Eigen::Vector4d Tst1, GaitType gait_type);
    ~GaitController() = default;
    void changeGaitType(GaitType gait_type = GaitType::STAND);
    void step();
    Eigen::MatrixXi predictContactStates(unsigned int horizon, double time_per_horizon);
    Eigen::Vector4i &getContactStates();
    Eigen::Vector4d &getSwingSubPhase();
    Eigen::Vector4d &getStanceSubPhase();
    Eigen::Vector4d getSwingDuration();
    Eigen::Vector4d &getStanceDuration();
    void updateGaitPeriod(double new_Tsw, int new_Tsw_count);
};
#endif //PROJECT_GAIT_SCHEDULER_H
