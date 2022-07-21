/*
 * @Author: haoyun 
 * @Date: 2022-07-16 14:31:07
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-21 21:00:03
 * @FilePath: /drake/workspace/centaur_sim/controller/CentaurControl.cc
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#include "drake/workspace/centaur_sim/controller/CentaurControl.h"



CentaurControl::CentaurControl(const control_params_constant ctrl_params) {


    DRAKE_DEMAND(ctrl_params.mpc_horizon == MPC_HORIZON);
    this->mpc_horizon = ctrl_params.mpc_horizon;

    this->mpc_dt = ctrl_params.control_dt * ctrl_params.nIterationsPerMPC;

    DRAKE_DEMAND(ctrl_params.q_weights.size() == NUM_STATE);
    for (size_t i = 0; i < ctrl_params.q_weights.size(); i++)
        this->mpc_q_weights(i) = ctrl_params.q_weights.at(i);

    DRAKE_DEMAND(ctrl_params.r_weights.size() == NUM_U);
    for (size_t i = 0; i < ctrl_params.r_weights.size(); i++)
        this->mpc_r_weights(i) = ctrl_params.r_weights.at(i);   
        
    this->mu = ctrl_params.mu;
    mpc_solver = new ConvexMPC(this->mpc_horizon, this->mpc_dt, this->mpc_q_weights, this->mpc_r_weights, this->mu);


}

Eigen::Matrix<double, 3, 2> CentaurControl::ComputeGoundReactionForce(CentaurStates& state)
{

    
    // initial state
    mpc_solver->x0 << state.root_euler[0], state.root_euler[1], state.root_euler[2],
                    state.root_pos[0], state.root_pos[1], state.root_pos[2],
                    state.root_ang_vel[0], state.root_ang_vel[1], state.root_ang_vel[2],
                    state.root_lin_vel[0], state.root_lin_vel[1], state.root_lin_vel[2];

    // drake::log()->info("x0 = ");
    // drake::log()->info(mpc_solver->x0.transpose());

    mpc_solver->Update_Xd_Trajectory(state);

    mpc_solver->Update_Aqp_Nqp(state.root_euler);
    
    mpc_solver->Update_Bd_ExternTerm(state.mass,
                                     state.inertiaMat, 
                                     state.root_euler, 
                                     state.foot_pos_abs,
                                     state.external_wrench,
                                     state.sphere_joint_location);

    mpc_solver->FormulateQP(state.mpc_contact_table);

    mpc_solver->SolveMPC();

    return mpc_solver->result_mat;
}

void CentaurControl::GenerateSwingTrajectory(CentaurStates& state)
{
    // Step 1: update foot_pos_dest_rel
    // Raibert Heuristic, calculate foothold position
    state.foothold_dest_rel = state.default_foot_pos_rel;
    Eigen::Vector3d lin_vel_rel = state.root_rot_mat_z.transpose() * state.root_lin_vel;
    
    for (int leg = 0; leg < 2; leg++) // two legs
    {
        double delta_x =
                std::sqrt(std::abs(state.default_foot_pos_rel(2)) / 9.8) * (lin_vel_rel(0) - state.root_lin_vel_d(0)) +
                (state.gait_period * (1 - state.stance_duration(leg))) / 2.0 * state.root_lin_vel_d(0);
        
        double delta_y =
                std::sqrt(std::abs(state.default_foot_pos_rel(2)) / 9.8) * (lin_vel_rel(1) - state.root_lin_vel_d(1)) +
                (state.gait_period * (1 - state.stance_duration(leg)))  / 2.0 * state.root_lin_vel_d(1);

        delta_x = (delta_x>FOOT_DELTA_X_LIMIT)?(FOOT_DELTA_X_LIMIT):((delta_x<-FOOT_DELTA_X_LIMIT)?(-FOOT_DELTA_X_LIMIT):delta_x);
        delta_y = (delta_y>FOOT_DELTA_X_LIMIT)?(FOOT_DELTA_X_LIMIT):((delta_y<-FOOT_DELTA_X_LIMIT)?(-FOOT_DELTA_X_LIMIT):delta_y);
        
        state.foothold_dest_rel(0, leg) += delta_x;
        state.foothold_dest_rel(1, leg) += delta_y;

        state.foothold_dest_abs.block<3, 1>(0, leg) = state.root_rot_mat * state.foothold_dest_rel.block<3, 1>(0, leg);
        state.foothold_dest_world.block<3, 1>(0, leg) = state.foothold_dest_abs.block<3, 1>(0, leg) + state.root_pos;

    }
    
    // Step 2: generate trajectory using Bezier curve given gait scheduler


}