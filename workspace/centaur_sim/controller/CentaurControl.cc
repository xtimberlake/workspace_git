/*
 * @Author: haoyun 
 * @Date: 2022-07-16 14:31:07
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-11-25 18:57:32
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

    FirstTimeSwing = true;

    for (int leg = 0; leg < 2; leg++) {
        swingtrajectory[leg].setHeight(FOOT_SWING_CLEARANCE2);
    }
    
    
}

Eigen::Matrix<double, 3, 2> CentaurControl::ComputeGoundReactionForce(CentaurStates& state)
{

    
    // initial state
    mpc_solver->x0 << state.root_euler[0], state.root_euler[1], state.root_euler[2],
                    state.root_pos[0], state.root_pos[1], state.root_pos[2],
                    state.root_ang_vel_world[0], state.root_ang_vel_world[1], state.root_ang_vel_world[2],
                    state.root_lin_vel_world[0], state.root_lin_vel_world[1], state.root_lin_vel_world[2];

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

    state.foot_force_cmd_world = mpc_solver->result_mat;
    state.foot_force_cmd_rel.block<3, 1>(0, 0) = state.root_rot_mat_z.transpose() * state.foot_force_cmd_world.block<3, 1>(0, 0);
    state.foot_force_cmd_rel.block<3, 1>(0, 1) = state.root_rot_mat_z.transpose() * state.foot_force_cmd_world.block<3, 1>(0, 1);


    // state.tau_ff.segment<3>(0) = -state.JacobianFoot[0].transpose() * state.foot_force_cmd_rel.block<3, 1>(0, 0);
    // state.tau_ff.segment<3>(3) = -state.JacobianFoot[1].transpose() * state.foot_force_cmd_rel.block<3, 1>(0, 1);


    return mpc_solver->result_mat;
}

void CentaurControl::GenerateSwingTrajectory(CentaurStates& state)
{
    // // Step 1: update foot_pos_dest_rel
    // // Raibert Heuristic, calculate foothold position
    // state.foothold_dest_rel = state.default_foot_pos_rel;
    // Eigen::Vector3d lin_vel_rel = state.root_rot_mat.transpose() * state.root_lin_vel_world;

    // for (int leg = 0; leg < 2; leg++) // two legs
    // {
    //     double swingTimeRemain = (1 - state.plan_swings_phase[leg]) * state.gait_period * (1 - state.stance_duration[leg]);

    //     double delta_x =
    //             8 * std::sqrt(std::abs(0.9/*state.default_foot_pos_rel(2)*/) / 9.81) * (lin_vel_rel(0) - state.root_lin_vel_d_rel(0))
    //             + (state.gait_period * state.stance_duration(leg)) / 2.0 * state.root_lin_vel_d_rel(0)
    //             + swingTimeRemain * lin_vel_rel(0);
        
    //     double delta_y =
    //             8 * std::sqrt(std::abs(0.9/*state.default_foot_pos_rel(2)*/) / 9.81) * (lin_vel_rel(1) - state.root_lin_vel_d_rel(1))
    //             + (state.gait_period * state.stance_duration(leg))  / 2.0 * state.root_lin_vel_d_rel(1)
    //             + swingTimeRemain * lin_vel_rel(1);

    //     delta_x = (delta_x>FOOT_DELTA_X_LIMIT)?(FOOT_DELTA_X_LIMIT):((delta_x<-FOOT_DELTA_X_LIMIT)?(-FOOT_DELTA_X_LIMIT):delta_x);
    //     delta_y = (delta_y>FOOT_DELTA_Y_LIMIT)?(FOOT_DELTA_Y_LIMIT):((delta_y<-FOOT_DELTA_Y_LIMIT)?(-FOOT_DELTA_Y_LIMIT):delta_y);
        
    //     state.foothold_dest_rel(0, leg) += delta_x;
    //     state.foothold_dest_rel(1, leg) += delta_y;
        
    //     state.foothold_dest_abs.block<3, 1>(0, leg) = state.root_rot_mat * state.foothold_dest_rel.block<3, 1>(0, leg);
    //     state.foothold_dest_world.block<3, 1>(0, leg) = state.foothold_dest_abs.block<3, 1>(0, leg) + state.root_pos;

    // }

    Eigen::Matrix<double, 3, 2> pShoulder_world; pShoulder_world.setZero();
    Eigen::Matrix<double, 3, 1> pSymmetry; pSymmetry.setZero();
    Eigen::Matrix<double, 3, 1> pCentrifugal; pCentrifugal.setZero();
    Eigen::Matrix<double, 3, 1> pYawCorrected; pYawCorrected.setZero();
    Eigen::Matrix<double, 3, 1> pDelta; pDelta.setZero();
    Eigen::Matrix<double, 3, 1> foot_final_pos; foot_final_pos.setZero();
    double tStance;
    double swingTimeRemain;
    int side_sign[2] = {1, -1};

    
    for (int leg = 0; leg < 2; leg++) {
        Eigen::Matrix<double, 3, 1> offset(1.0 * state.ctrl_params_const.default_foot_pos_under_hip.at(0), side_sign[leg] * state.ctrl_params_const.default_foot_pos_under_hip.at(1), 0);
        swingTimeRemain = (1 - state.plan_swings_phase[leg]) * state.gait_period * (1 - state.stance_duration[leg]);
        tStance = (state.gait_period * state.stance_duration(leg));
        pShoulder_world.block<3, 1>(0, leg) = state.root_pos + state.root_rot_mat * (state.hipLocation_local.block<3, 1>(0, leg) + offset);
        pYawCorrected = ori::coordinateRotation(ori::CoordinateAxis::Z, - state.root_ang_vel_d_world[2] * tStance / 2) * pShoulder_world.block<3, 1>(0, leg);

        pSymmetry = tStance / 2.0 * state.root_lin_vel_world + 0.03 * (state.root_lin_vel_world - state.root_lin_vel_d_world);
        pCentrifugal = std::sqrt(std::abs(/*state.root_pos(2)*/0.9) / 9.81)/2.0 * state.root_lin_vel_world.cross(state.root_ang_vel_d_world);

        pDelta = pSymmetry + pCentrifugal + swingTimeRemain * state.root_lin_vel_world;
        pDelta[0] = (pDelta[0]>FOOT_DELTA_X_LIMIT)?(FOOT_DELTA_X_LIMIT):((pDelta[0]<-FOOT_DELTA_X_LIMIT)?(-FOOT_DELTA_X_LIMIT):pDelta[0]);
        pDelta[1] = (pDelta[1]>FOOT_DELTA_Y_LIMIT)?(FOOT_DELTA_Y_LIMIT):((pDelta[1]<-FOOT_DELTA_Y_LIMIT)?(-FOOT_DELTA_Y_LIMIT):pDelta[1]);

        foot_final_pos = pYawCorrected + pDelta;
        foot_final_pos[2] = -0.003; // height in world frame

        // if (leg == 0) {
        //     std::cout << "pSymmetry = " << pSymmetry.transpose() << ", " << "pCentrifugal = " << pCentrifugal.transpose() << "," 
        //     << "swingT*vel =  " << swingTimeRemain * state.root_lin_vel_world.transpose() << std::endl;
        // }
        

        state.foothold_dest_world.block<3, 1>(0, leg) = foot_final_pos;
        state.foothold_dest_abs.block<3, 1>(0, leg) = state.foothold_dest_world.block<3, 1>(0, leg) - state.root_pos;
        state.foothold_dest_rel.block<3, 1>(0, leg) = state.root_rot_mat.transpose() * state.foothold_dest_abs.block<3, 1>(0, leg);
    }
    


    
    // Step 2: generate trajectory(in world) using Bezier curve given gait scheduler
    if(FirstTimeSwing) {
        for (int leg = 0; leg < 2; leg++) {
            swingtrajectory[leg].setInitialPosition(state.foot_pos_world.block<3, 1>(0, leg));
            swingtrajectory[leg].setFinalPosition(state.foothold_dest_world.block<3, 1>(0, leg));
            state.foot_pos_cmd_world.block<3, 1>(0, leg) = state.foot_pos_world.block<3, 1>(0, leg);
            state.foot_vel_cmd_world.block<3, 1>(0, leg).setZero();
        }
        FirstTimeSwing = false;
    }
    else {
        for (int leg = 0; leg < 2; leg++)
        {
            if (state.plan_contacts_phase(leg) > 0) {
                // stance
                swingtrajectory[leg].setInitialPosition(state.foot_pos_world.block<3, 1>(0, leg));
                state.foot_pos_cmd_world.block<3, 1>(0, leg) = state.foot_pos_world.block<3, 1>(0, leg);
                // state.foot_pos_cmd_world.block<3, 1>(0, leg) = state.foothold_dest_world.block<3, 1>(0, leg);
                state.foot_vel_cmd_world.block<3, 1>(0, leg).setZero();


            }
            else {
                // swing
                swingtrajectory[leg].setFinalPosition(state.foothold_dest_world.block<3, 1>(0, leg));
                swingtrajectory[leg].computeSwingTrajectoryBezier(state.plan_swings_phase(leg), state.gait_period * (1 - state.stance_duration(leg)));
                state.foot_pos_cmd_world.block<3, 1>(0, leg) = swingtrajectory[leg].getPosition();
                state.foot_vel_cmd_world.block<3, 1>(0, leg) = swingtrajectory[leg].getVelocity();
              
            }
        }
    } 

    // Event-based swing foot control
    for (int leg = 0; leg< 2; leg++) {
        switch (state.foot_contact_event[leg])
        { 
            case ContactEvent::SWING:
            {
                break;
            }
            case ContactEvent::EARLY_CONTACT:
            {

                state.foot_pos_cmd_world.block<3, 1>(0, leg) = state.locked_foot_pos.block<3, 1>(0, leg);
                state.foot_vel_cmd_world.block<3, 1>(0, leg).setZero();
                break;
            }
            case ContactEvent::LATE_CONTACT:
            {
                // state.foot_pos_cmd_world.block<3, 1>(0, leg) = state.locked_foot_pos.block<3, 1>(0, leg);
                break;
            }
            case ContactEvent::RESTANCE:
            {

                break;
            }
            case ContactEvent::STANCE:
            {
                break;
            }
        
            default: break;
        }
        

    }
    




    // expressed in the CoM's frame
    for (int leg = 0; leg < 2; leg++)
    {
        state.foot_pos_cmd_abs.block<3, 1>(0, leg) = state.foot_pos_cmd_world.block<3, 1>(0, leg) - state.root_pos;
        state.foot_pos_cmd_rel.block<3, 1>(0, leg) = state.root_rot_mat.transpose() * state.foot_pos_cmd_abs.block<3, 1>(0, leg);

        state.foot_vel_cmd_rel.block<3, 1>(0, leg) = state.root_rot_mat_z.transpose() * state.foot_vel_cmd_world.block<3, 1>(0, leg);
    }
    
}

void CentaurControl::InverseKinematics(CentaurStates& state)
{
    
    for (int leg = 0; leg < 2; leg++) {
        state.qdot_cmd.segment<3>(leg * 3) = Utils::pseudo_inverse(state.JacobianFoot[leg]) * state.foot_vel_cmd_rel.block<3, 1>(0, leg);
    }

    Eigen::Matrix<double, 3, 2> foot_pos_err;
    for (int leg = 0; leg < 2; leg++)
    {
        foot_pos_err.block<3, 1>(0, leg) = state.foot_pos_cmd_rel.block<3, 1>(0, leg) - state.foot_pos_rel.block<3, 1>(0, leg);
        state.q_cmd.segment<3>(leg * 3) = state.q.segment<3>(leg * 3) + state.control_dt * Utils::pseudo_inverse(state.JacobianFoot[leg]) * foot_pos_err.block<3, 1>(0, leg);
    }

}