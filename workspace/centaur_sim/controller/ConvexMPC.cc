/*
 * @Author: haoyun 
 * @Date: 2022-07-18 09:28:36
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-19 11:30:24
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
    // construct B_qp matrix(skip this part because B is time-invarient)
    
    // Step # 4:
    // construct N_qp matrix
    N_qp.block<12, 12>(0, 0) = eye12;
    
    

    
}

void ConvexMPC::Update_Ad_Nd(Eigen::Vector3d euler)
{
    Eigen::Matrix3d R_yaw_T;
    double yaw = euler(2);
    R_yaw_T << cos(yaw), sin(yaw), 0,
            -sin(yaw), cos(yaw), 0,
            0, 0, 1;
    
    for (int i = 0; i < _mpc_horizon; i++)  {

        // power = 1, 2, 3, ..., 10
        int power = i + 1;
        A_power[power].block<3, 3>(i * NUM_STATE, 6) = R_yaw_T * power * _dt;
        if(i > 0) // i = 1, 2, ..., 9
        {
            int preview = i - 1;
            N_qp.block<NUM_STATE, NUM_STATE>(i * NUM_STATE, 0) =
                N_qp.block<NUM_STATE, NUM_STATE>(preview * NUM_STATE, 0) + A_power[i];
        }
    }

}

/**
 * @description: calculate B_d matrix
 * @note: foot positions are already expreesd in the world frame
 * @return {*}
 */
void ConvexMPC::Update_Bd(double mass, 
                        Eigen::Matrix3d inertia,
                        Eigen::Matrix3d R, 
                        Eigen::Matrix<double, 3, 2> foot_pos) {

    Eigen::Matrix3d inertia_in_world, I_inv;
    inertia_in_world = R * inertia * R.transpose();
    I_inv = Utils::pseudo_inverse(inertia_in_world); // improve robustness

    Eigen::Matrix<double, NUM_STATE, NUM_U> B_d;
    for (int i = 0; i < 2; i++) // two legs
    {
        B_d.block<3, 3>(6, i * NUM_U) = I_inv * Utils::skew(foot_pos.block<3, 1>(0, i)) * this->_dt;
        B_d.block<3, 3>(9, i * NUM_U) = (1 / mass) * Eigen::Matrix3d::Identity() * this->_dt;
    }
    



}