/*
 * @Author: haoyun 
 * @Date: 2022-07-18 09:28:36
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-18 22:36:03
 * @FilePath: /drake/workspace/centaur_sim/controller/ConvexMPC.cc
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#include "drake/workspace/centaur_sim/controller/ConvexMPC.h"

ConvexMPC::ConvexMPC(int mpc_horizon,
                     double dt,
                     Eigen::VectorXd q_weights, 
                     Eigen::VectorXd r_weights)
{
    this->_mpc_horizon = mpc_horizon;
    this->_state_dim = q_weights.rows();
    this->_u_dim = r_weights.rows();
    this->_dt = dt;

    DRAKE_DEMAND(this->_mpc_horizon == MPC_HORIZON);
    DRAKE_DEMAND(this->_state_dim == NUM_STATE);
    DRAKE_DEMAND(this->_u_dim == NUM_U);



    // Step # 1:
    // construct Q_qp & R_qp for quadratic program
    Eigen::Matrix<double, NUM_STATE * MPC_HORIZON, 1> q_weights_mpc;
    for (int i = 0; i < MPC_HORIZON; i++) {
        q_weights_mpc.segment<NUM_STATE>(NUM_STATE * i) = q_weights;
    }

    Eigen::Matrix<double, NUM_U * MPC_HORIZON, 1> r_weights_mpc;
    for (int i = 0; i < MPC_HORIZON; i++) {
        r_weights_mpc.segment<NUM_U>(NUM_U * i) = r_weights;
    }

    for (int i = 0; i < NUM_STATE * MPC_HORIZON; i++) {
        _Q_qp(i, i) =  q_weights_mpc(i);
    }

    for (int i = 0; i < NUM_U * MPC_HORIZON; i++) {
        _R_qp(i, i) =  r_weights_mpc(i);
    }

    drake::log()->info("num of states = " + std::to_string(_state_dim));
    drake::log()->info("Q_qp rows = " + std::to_string(_Q_qp.rows()) + ", Q_qp cols = " + std::to_string(_Q_qp.cols()));
    drake::log()->info("R_qp rows = " + std::to_string(_R_qp.rows()) + ", R_qp cols = " + std::to_string(_R_qp.cols()));

    // Step # 2:
    // construct A_qp matrix (invarient part)
    Eigen::Matrix3d eye3; eye3.setIdentity();
    Eigen::Matrix<double, NUM_STATE, NUM_STATE> eye12; eye12.setIdentity();
    A_power[0] = eye12;
    for (int i = 0; i < MPC_HORIZON; i++)
    {
        // discrete-time
        A_power[i + 1].block<3, 3>(3, 9) = eye3 * i * _dt;
        A_power[i + 1] += eye12;

        A_qp.block<NUM_STATE, NUM_STATE>(i * NUM_STATE, 0) = A_power[i + 1];
    }

    // Step # 3:
    // construct B_qp matrix(skip because B is time invarient)
    
    // Step # 4:
    // construct N_qp matrix(skip because N is time invarient)

    
    
    


    
}

void ConvexMPC::update_A_d(Eigen::Vector3d euler)
{

    this->_state_dim = euler(2);
}