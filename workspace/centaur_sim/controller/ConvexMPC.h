/*
 * @Author: haoyun 
 * @Date: 2022-07-18 09:28:44
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-18 10:12:04
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

class ConvexMPC {
public:
    ConvexMPC(int mpc_horizon, Eigen::VectorXd q_weights, Eigen::VectorXd r_weights);
    
    int _state_dim;
    int _u_dim;
    int _mpc_horizon;
    Eigen::MatrixXd _Q_qp;
    Eigen::MatrixXd _R_qp;
};