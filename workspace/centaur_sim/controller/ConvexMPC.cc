/*
 * @Author: haoyun 
 * @Date: 2022-07-18 09:28:36
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-20 19:42:44
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
    // Extern term
    this->gravity << 0, 0, -9.81;
    ExternalTerm.segment<3>(9) = this->gravity;

    // Step # 6:
    // construct linear contraint matrix C
    for (int i = 0; i < 2 * MPC_HORIZON; i++) 
    {
            ConstriantMat(0 + 5 * i, 0 + 3 * i) = 1;
            ConstriantMat(1 + 5 * i, 0 + 3 * i) = 1;
            ConstriantMat(2 + 5 * i, 1 + 3 * i) = 1;
            ConstriantMat(3 + 5 * i, 1 + 3 * i) = 1;
            ConstriantMat(4 + 5 * i, 2 + 3 * i) = 1;

            ConstriantMat(0 + 5 * i, 2 + 3 * i) = mu;
            ConstriantMat(1 + 5 * i, 2 + 3 * i) = -mu;
            ConstriantMat(2 + 5 * i, 2 + 3 * i) = mu;
            ConstriantMat(3 + 5 * i, 2 + 3 * i) = -mu;
        
    }
    // drake::log()->info("C_mat rows = " + std::to_string(ConstriantMat.rows()) + ", C_qp cols = " + std::to_string(ConstriantMat.cols()));
    

    // Step # 7:
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

    // solvers:
    U_all.setZero();
    next_result_vec.setZero();

}

void ConvexMPC::Update_Xd_Trajectory(CentaurStates& state)
{
    // velocity
    for (int i = 0; i < MPC_HORIZON; i++)
    {
        xd_trajectory.segment<3>(6 + i * NUM_STATE) << state.root_ang_vel_d_world;
        xd_trajectory.segment<3>(9 + i * NUM_STATE) << state.root_lin_vel_d_world;
    }
    
    // position
    for (int i = 0; i < MPC_HORIZON; i++)
    {
        xd_trajectory.segment<3>(0 + i * NUM_STATE) << state.root_pos_d + state.root_lin_vel_d_world * (i * _dt);
        xd_trajectory.segment<3>(0 + i * NUM_STATE) << state.root_euler_d;
        // TODO(haoyun) what's proper desired euler angle?
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
void ConvexMPC::Update_Bd_ExternTerm(double mass,
                            Eigen::Matrix3d inertia,
                            Eigen::Vector3d euler,
                            Eigen::Matrix<double, 3, 2> foot_pos,
                            Eigen::Matrix<double, 6, 1> wrench,
                            Eigen::Vector3d sphere_joint_location) {

    Eigen::Matrix3d R, inertia_in_world, I_inv;
    R << cos(euler[2]), -sin(euler[2]), 0,
            sin(euler[2]), cos(euler[2]), 0,
            0, 0, 1;
    inertia_in_world = R * inertia * R.transpose();
    I_inv = Utils::pseudo_inverse(inertia_in_world); // improve robustness

    for (int i = 0; i < 2; i++) // two legs
    {
        B_d.block<3, 3>(6, i * NUM_U) = I_inv * Utils::skew(foot_pos.block<3, 1>(0, i)) * this->_dt;
        B_d.block<3, 3>(9, i * NUM_U) = (1 / mass) * Eigen::Matrix3d::Identity() * this->_dt;
    }

    ExternalTerm.segment<3>(6) = I_inv * Utils::skew(sphere_joint_location) * wrench.tail(3) + wrench.head(3);
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

    HessianMat = B_qp.transpose() * _Q_qp * B_qp + _R_qp;
    gradientVec = B_qp.transpose() * _Q_qp * (A_qp * x0 - xd_trajectory + N_qp * ExternalTerm);
    
}

void ConvexMPC::SolveMPC()
{
    drake::solvers::MathematicalProgram prog;
    drake::solvers::MosekSolver mosek_solver;

    auto U = prog.NewContinuousVariables<NUM_U * MPC_HORIZON>();
    
    auto qp_cost = prog.AddQuadraticCost(HessianMat, gradientVec, U);
    
    // auto linear_constraint = prog.AddLinearConstraint(ConstriantMat, lb, ub, U);
    
    if(mosek_solver.available()) {
        drake::solvers::MathematicalProgramResult prog_result;
        mosek_solver.Solve(prog, {}, {}, &prog_result);
        if(qp_cost.evaluator()->is_convex()) drake::log()->info("convex!");

        if (prog_result.is_success()) {
            drake::log()->info("congras!");
            U_all = prog_result.GetSolution();
            result_mat.block<3, 1>(0, 0) = U_all.segment<3>(0);
            result_mat.block<3, 1>(0, 1) = U_all.segment<3>(3);
            next_result_vec = U_all;
            const drake::solvers::MosekSolverDetails& mosek_solver_details =
                prog_result.get_solver_details<drake::solvers::MosekSolver>();
            drake::log()->info("optimizer time: " + std::to_string(mosek_solver_details.optimizer_time));
        }
        else {
            drake::log()->warn("fail to find a result...");
        }
    }
    else{
        drake::log()->warn("mosek solver is not available !");
    }
    

}


