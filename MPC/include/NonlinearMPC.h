/*!
 * @file: NonlinearMPC.h
 * @authors: HuNing-He
 * @date: 2022-11-25
 * @copyright (c) 2022 HuNing-He. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#pragma once
#include <casadi/casadi.hpp>
#include <eigen3/Eigen/Dense>

/*!
 * ma97 and ma86 are suggested for nonlinear mpc, mumps is default one
 * please check on https://www.hsl.rl.ac.uk/ipopt/
 */
enum class IPOPT_SOLVER: unsigned int {
    MUMPS = 0,
    WSMP = 1,
    PARDISO = 2,
    MA27 = 3,
    MA57 = 4,
    MA77 = 5,
    MA86 = 6,
    MA97 = 7,
};
/*!
 * @brief refer to my csdn blog https://blog.csdn.net/weixin_43989965/article/details/128027442
 * @param input
 * @return eigen map
 */
Eigen::Map<Eigen::MatrixXd> toEigen(casadi::DM &input) {
    return {input.ptr(), input.rows(), input.columns()};
}

class NonlinearMPC {
protected:
    int predict_horizon_;
    double time_step_;

    casadi::Opti opti_;                   // CasADi Opti stack
    casadi::Function controller_;         // Updated in SetupMPC
    casadi::Function system_dynamic_;     // Updated in CreateSystemDynamic

    casadi::DM weights_;
    Eigen::VectorXd controller_input_;

    /*!
     * System state space dynamic:
     * x_{k+1} = f(x_k, u_k)
     * the function f is system_dynamic_ in casadi here
     * this function must be called in SetupMPC()
     */
    virtual void CreateSystemDynamic() = 0; // create system_dynamic_ here

public:
    NonlinearMPC() = delete;
    NonlinearMPC(const NonlinearMPC &n) = delete;
    NonlinearMPC &operator=(const NonlinearMPC &n) = delete;
    ~NonlinearMPC() = default;

    NonlinearMPC(int predict_horizon, double time_step, IPOPT_SOLVER ipopt_solver, const Eigen::VectorXd &weights,
                 unsigned long verbosity = 0, double ipopt_tolerance = 1e-8) {
        predict_horizon_ = predict_horizon;
        time_step_ = time_step;

        std::string ipopt_solver_{};            // Linear solved used by ipopt
        switch (ipopt_solver) {
            case IPOPT_SOLVER::MUMPS:ipopt_solver_ = "mumps";
                break;
            case IPOPT_SOLVER::WSMP:ipopt_solver_ = "wsmp";
                break;
            case IPOPT_SOLVER::PARDISO:ipopt_solver_ = "pardiso";
                break;
            case IPOPT_SOLVER::MA27:ipopt_solver_ = "ma27";
                break;
            case IPOPT_SOLVER::MA57:ipopt_solver_ = "ma57";
                break;
            case IPOPT_SOLVER::MA77:ipopt_solver_ = "ma77";
                break;
            case IPOPT_SOLVER::MA86:ipopt_solver_ = "ma86";
                break;
            case IPOPT_SOLVER::MA97:ipopt_solver_ = "ma97";
                break;
            default:ipopt_solver_ = "mumps";
                break;
        }

        weights_ = casadi::DM::zeros(weights.rows());
        UpdateWeights(weights);
        // refer to https://coin-or.github.io/Ipopt/OPTIONS.html
        casadi::Dict ipopt_options;
        casadi::Dict casadi_options;

        if (verbosity != 0) {
            auto ipoptVerbosity = static_cast<long long>(verbosity - 1);
            ipopt_options["print_level"] = ipoptVerbosity;
            casadi_options["print_time"] = true;
        } else {
            ipopt_options["print_level"] = 0;
            casadi_options["print_time"] = false;
        }

        ipopt_options["linear_solver"] = ipopt_solver_;
        ipopt_options["tol"] = ipopt_tolerance;
        casadi_options["expand"] = true;

        opti_.solver("ipopt", casadi_options, ipopt_options);
    }

    void UpdateWeights(const Eigen::VectorXd &weights) {
        toEigen(weights_) = weights;
    }

    /*!
     * @brief create controller_
     * @note You should setup your cost function and constraints in opti_ here
     */
    virtual void SetupMPC() = 0;

    /*!
     * @brief update controller_ states here and get mpc output i.e. controller_input_
     * @param state system state
     * @param des_state desire system state
     * @param des_inputs desire system input
     * @return controller_input_
     */
    virtual Eigen::VectorXd UpdateMPC(const Eigen::VectorXd &state, const Eigen::VectorXd &des_state, const Eigen::VectorXd &des_inputs) = 0;
};
