/*
 * @Author: haoyun 
 * @Date: 2022-07-18 09:28:36
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-19 23:44:41
 * @FilePath: /drake/workspace/centaur_sim/controller/ConvexMPC.cc
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#include "drake/workspace/centaur_sim/controller/ConvexMPC.h"

ConvexMPC::ConvexMPC(int mpc_horizon,
                     double dt,
                     Eigen::VectorXd q_weights, 
                     Eigen::VectorXd r_weights,
                     double mu)
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

    // Step # 5:
    // construct linear contraint matrix C
    for (int i = 0; i < 2 * MPC_HORIZON; i++) 
    {
            ConstrianMat(0 + 5 * i, 0 + 3 * i) = 1;
            ConstrianMat(1 + 5 * i, 0 + 3 * i) = 1;
            ConstrianMat(2 + 5 * i, 1 + 3 * i) = 1;
            ConstrianMat(3 + 5 * i, 1 + 3 * i) = 1;
            ConstrianMat(4 + 5 * i, 2 + 3 * i) = 1;

            ConstrianMat(0 + 5 * i, 2 + 3 * i) = mu;
            ConstrianMat(1 + 5 * i, 2 + 3 * i) = -mu;
            ConstrianMat(2 + 5 * i, 2 + 3 * i) = mu;
            ConstrianMat(3 + 5 * i, 2 + 3 * i) = -mu;
        
    }
    // drake::log()->info("C_mat rows = " + std::to_string(ConstrianMat.rows()) + ", C_qp cols = " + std::to_string(ConstrianMat.cols()));
    

    // Step # 6:
    // lower bound and upper bound
    Eigen::VectorXd lb_one_horizon(5 * 2); // two loegs
    Eigen::VectorXd ub_one_horizon(5 * 2);
    for (int i = 0; i < 2; ++i) // two loegs
    {
        lb_one_horizon.segment<5>(i * 5) << 0,
                -std::numeric_limits<double>::infinity(),
                0,
                -std::numeric_limits<double>::infinity(),
                0; /* f_min = 0 */
        ub_one_horizon.segment<5>(i * 5) << std::numeric_limits<double>::infinity(),
                0,
                std::numeric_limits<double>::infinity(),
                0,
                0; /* f_max * contact */
    }

    for (int i = 0; i < MPC_HORIZON; i++)
    {
        lb.segment<5 * 2>(i * 5 * 2) = lb_one_horizon;
        ub.segment<5 * 2>(i * 5 * 2) = ub_one_horizon;
    }

}

void ConvexMPC::Update_Aqp_Nqp(Eigen::Vector3d euler)
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

        A_qp.block<NUM_STATE, NUM_STATE>(i * NUM_STATE, i) = A_power[power];

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

    for (int i = 0; i < 2; i++) // two legs
    {
        B_d.block<3, 3>(6, i * NUM_U) = I_inv * Utils::skew(foot_pos.block<3, 1>(0, i)) * this->_dt;
        B_d.block<3, 3>(9, i * NUM_U) = (1 / mass) * Eigen::Matrix3d::Identity() * this->_dt;
    }
    
}

void ConvexMPC::FormulateQP(int* mpc_contact_table)
{
    // Step 1: A_qp(done in 'Update_Aqp_Nqp' function)

    // Step 2: B_qp
    for (int i = 0; i < MPC_HORIZON; i++) {
       A_powerB[i] = A_power[i] * B_d;
    }
    
    for (int row = 0; row < MPC_HORIZON; row++)
    {
        for (int col = 0; col <= row; col++)
        {
            int a_power = row - col;
            B_qp.block<NUM_STATE, NUM_U>(row * NUM_STATE, col * NUM_U) = A_powerB[a_power];
        }
        
    }
    
    // Step 3: N_qp(done in 'Update_Aqp_Nqp' function)

    // Step 4: lower bound & upper bound

    int index = 0;
    for (int i = 0; i < MPC_HORIZON; i++)
    {
       for (int leg = 0; leg < 2; leg++) // two legs
       {
            // does not need to change lb because f_min = 0
            ub(4 + i*10 + leg*5) = mpc_contact_table[index++];
       }
    }
    
}
