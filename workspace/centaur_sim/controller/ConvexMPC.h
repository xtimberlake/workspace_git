/*
 * @Author: haoyun 
 * @Date: 2022-07-18 09:28:44
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-19 23:20:17
 * @FilePath: /drake/workspace/centaur_sim/controller/ConvexMPC.h
 * @Description: mpc matrix transformation and qp sovler
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once

#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/solver_interface.h>
#include <drake/solvers/mosek_solver.h>
#include <drake/solvers/solve.h>
#include <drake/workspace/centaur_sim/controller/CentaurStates.h>
#include "drake/workspace/centaur_sim/controller/CentaurParams.h"
#include "drake/workspace/centaur_sim/Utils/Utils.h"

class ConvexMPC {
public:
    ConvexMPC(int mpc_horizon, double dt, Eigen::VectorXd q_weights, Eigen::VectorXd r_weights, double mu);
    void Update_Aqp_Nqp(Eigen::Vector3d euler);
    void Update_Bd(double mass, Eigen::Matrix3d inertia,
                    Eigen::Matrix3d R, Eigen::Matrix<double, 3, 2> foot_pos);
    void FormulateQP(int* mpc_contact_table);
    
    int _state_dim;
    int _u_dim;
    int _mpc_horizon;
    double _dt;

    Eigen::Matrix<double, NUM_STATE, 1> x0;

    Eigen::Matrix<double, NUM_STATE, NUM_STATE> A_power[MPC_HORIZON + 1];
    Eigen::Matrix<double, NUM_STATE, NUM_U> A_powerB[MPC_HORIZON];
    Eigen::Matrix<double, NUM_STATE * MPC_HORIZON, NUM_STATE> A_qp;
    Eigen::Matrix<double, NUM_STATE, NUM_U> B_d;
    Eigen::Matrix<double, NUM_STATE * MPC_HORIZON, NUM_U * MPC_HORIZON> B_qp;
    Eigen::Matrix<double, NUM_STATE * MPC_HORIZON, NUM_STATE> N_qp;

    Eigen::Matrix<double, NUM_STATE * MPC_HORIZON, NUM_STATE * MPC_HORIZON> _Q_qp;
    Eigen::Matrix<double, NUM_U * MPC_HORIZON, NUM_U * MPC_HORIZON> _R_qp;

    Eigen::Matrix<double, 5 * 2 * MPC_HORIZON, NUM_U * MPC_HORIZON> ConstrianMat;// two legs
    Eigen::Matrix<double, 5 * 2 * MPC_HORIZON, 1> lb;
    Eigen::Matrix<double, 5 * 2 * MPC_HORIZON, 1> ub;
    
};