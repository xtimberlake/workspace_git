/*
 * @Author: haoyun 
 * @Date: 2022-07-16 14:31:07
 * @LastEditors: haoyun 
 * @LastEditTime: 2023-04-13 16:06:10
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

void CentaurControl::UpdateDesiredStates(CentaurStates& state) {

    int number_of_traj = 0;
    number_of_traj =  static_cast<int>((state.k/5));
    if(number_of_traj > state.max_human_ref_index) 
        number_of_traj = state.max_human_ref_index;
    // state.root_euler_d[2] = state.human_ref_traj.yaw[number_of_traj];
    state.root_euler_d[2] = 0.0;

    state.root_pos_d = state.root_pos;
    state.root_pos_d[2] = 0.9;
    if(state.Hri_pos[0] > 2.0 + state.sphere_joint_location(0)) {
        int coordinate_x = int((state.Hri_pos[0] - 2.0 - state.sphere_joint_location(0))/0.02);
        int coordinate_y = int((state.Hri_pos[1] + 0.5)/0.02);

        state.root_pos_d[2] = 0.9 + state.map.torso(coordinate_x, coordinate_y)*0.6019989318684672/0.777187762264438;
    }
    
    Eigen::Matrix<double, 3, 1> Kang, Kpos;
    Kang.setZero(); Kpos.setZero();
    Kang << 0.0, 0.0, 0.0;
    // Kpos << 0.0, 0.0, 1.5;
    state.root_ang_vel_d_world = Kang.cwiseProduct(state.root_euler_d - state.root_euler);
    state.root_lin_vel_d_world = Kpos.cwiseProduct(state.root_pos_d - state.root_pos);

    
    // std::cout << "yaw = " << state.root_euler[2] << "/" << state.root_euler_d[2] << std::endl;
}

void CentaurControl::CalcHRITorques(CentaurStates& state) {

    Eigen::Matrix<double, 6, 1> total_torques; total_torques.setZero();
    Eigen::Vector3d prismatic_joint_q; prismatic_joint_q.setZero();
    Eigen::Vector3d prismatic_joint_qdot; prismatic_joint_qdot.setZero();

    Eigen::Vector3d prismatic_joint_q_des; prismatic_joint_q_des.setZero();
    Eigen::Vector3d prismatic_joint_qdot_des; prismatic_joint_qdot_des.setZero();

    // int number_of_traj = 0;
    // number_of_traj =  static_cast<int>((state.k/5));
    // if(number_of_traj > state.max_human_ref_index) 
    //     number_of_traj = state.max_human_ref_index;
        
    // // // std::cout << "num of traj = " << number_of_traj << std::endl;
    // prismatic_joint_q_des[0] = state.human_ref_traj.x[number_of_traj];
    // prismatic_joint_q_des[1] = state.human_ref_traj.y[number_of_traj];


    int number_of_traj = 0;
    number_of_traj =  static_cast<int>((state.k));
        
    // // std::cout << "num of traj = " << number_of_traj << std::endl;
    prismatic_joint_q_des[0] = number_of_traj*0.0002;
    prismatic_joint_q_des[1] = 0.0;
    if(state.Hri_pos[0] > 2.0) {
        int coordinate_x = int((state.Hri_pos[0] - 2.0)/0.02);
        int coordinate_y = int((state.Hri_pos[1] + 0.5)/0.02);

        prismatic_joint_q_des[2] = -0.0 + state.map.torso(coordinate_x, coordinate_y)*0.6019989318684672/0.777187762264438;
    }
    


    prismatic_joint_q = state.hri_joint_states.segment<3>(0);
    prismatic_joint_qdot = state.hri_joint_states.segment<3>(6);

    total_torques.head(3) = state.human_pos_stiff.cwiseProduct(prismatic_joint_q_des - prismatic_joint_q)
                          + state.human_pos_damp.cwiseProduct(prismatic_joint_qdot_des - prismatic_joint_qdot);

    


    state.hri_actuated_torques = total_torques;



}

// void CentaurControl::EstHRIForces(CentaurStates& state) {

//     // Inertia
//     Eigen::Matrix3d R, inertia_in_world;
//     R << cos(state.root_euler[2]), -sin(state.root_euler[2]), 0,
//             sin(state.root_euler[2]), cos(state.root_euler[2]), 0,
//             0, 0, 1;

//     inertia_in_world = R * state.inertiaMat * R.transpose();

//     // rh
//     Eigen::Vector3d rh = R * state.sphere_joint_location;

    
//     // dot_omega
//     static Eigen::Vector3d last_omega_world = Eigen::Vector3d::Zero();
//     Eigen::Vector3d dot_omega = (state.root_ang_vel_world - last_omega_world)/0.001;
//     dot_omega.setZero();

//     // dot_v
//     static Eigen::Vector3d last_v_world = Eigen::Vector3d::Zero();
//     Eigen::Vector3d dot_v = (state.root_lin_vel_world - last_v_world)/0.001;
//     dot_v.setZero();

//     int contact_state = 0;
//     if(state.plan_contacts_phase[0] > 0) contact_state += 1;
//     if(state.plan_contacts_phase[1] > 0) contact_state += 2;


//     Eigen::Vector3d ri[2], fi[2], riCrossFi, fi_total; fi_total.setZero();
//     for (int i = 0; i < 2; i++) {
//         ri[i].setZero();
//         fi[i].setZero();
//     }
    
//     riCrossFi.setZero();

//     switch (contact_state)
//     {
//     case 0: 
//         // zero already
//     break;
//     case 1: 
//         // left contact only
//         ri[0] = state.foot_pos_world.block<3, 1>(0, 0) - state.root_pos;
//         fi[0] = -state.foot_force_world.block<3, 1>(0, 0);
//         fi_total = fi[0];
//         riCrossFi = ri[0].cross(fi[0]);
//     break;
//     case 2: 
//         // right contact only
//         ri[1] = state.foot_pos_world.block<3, 1>(0, 1) - state.root_pos;
//         fi[1] = -state.foot_force_world.block<3, 1>(0, 1);
//         fi_total = fi[1];
//         riCrossFi = ri[1].cross(fi[1]);
//     break;
//     case 3: 
//         // both contact
//         for (int i = 0; i < 2; i++) {
//             ri[i] = state.foot_pos_world.block<3, 1>(0, i) - state.root_pos;
//             fi[i] = -state.foot_force_world.block<3, 1>(0, i);
//             riCrossFi += ri[i].cross(fi[i]);
//             fi_total += fi[i];
//         }
        

//     break;
    
//     default: std::cout << "error contact!" << std::endl;
//         break;
//     }

//     Eigen::Vector3d est_fh; est_fh.setZero();
//     DMat<double> leftHandSide; leftHandSide = Eigen::Matrix<double, 6, 3>::Zero();
//     DMat<double> leftHandSide_inv;
//     leftHandSide.block<3, 3>(0, 0) = Utils::skew(rh);
//     leftHandSide.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();
//     pseudoInverse(leftHandSide, 0.001, leftHandSide_inv);

//     Eigen::Matrix<double, 6, 1> rightHandSide; rightHandSide.setZero();
//     Eigen::Vector3d grav; grav << 0, 0, -9.81;
//     rightHandSide.segment<3>(0) = inertia_in_world * dot_omega - riCrossFi;
//     rightHandSide.segment<3>(3) = state.mass * dot_v - fi_total - state.mass * grav;

//     std::cout << "rh = " << rh.transpose() << ",      ri0 = " << ri[0].transpose() << std::endl;
//     std::cout << "case = " << contact_state << ".    ";
//     est_fh = leftHandSide_inv * rightHandSide;

//     std::cout << "fi_total = " << fi_total.transpose() << ",   ";
//     std::cout << "est_fh = " << est_fh.transpose() << std::endl;
// }


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

        // fix the target position after 70% of the swing phase:
        if(state.plan_swings_phase[leg] > 0.7) continue;

        Eigen::Matrix<double, 3, 1> offset(1.0 * state.ctrl_params_const.default_foot_pos_under_hip.at(0), side_sign[leg] * state.ctrl_params_const.default_foot_pos_under_hip.at(1), 0);
        swingTimeRemain = (1 - state.plan_swings_phase[leg]) * state.gait_period * (1 - state.stance_duration[leg]);
        tStance = (state.gait_period * state.stance_duration(leg));
        pShoulder_world.block<3, 1>(0, leg) = state.root_pos + state.root_rot_mat * (state.hipLocation_local.block<3, 1>(0, leg) + offset);
        pYawCorrected = ori::coordinateRotation(ori::CoordinateAxis::Z, - state.root_ang_vel_d_world[2] * tStance / 2) * pShoulder_world.block<3, 1>(0, leg);
        // pYawCorrected = pShoulder_world.block<3, 1>(0, leg);
        pSymmetry = tStance / 2.0 * state.root_lin_vel_world + 0.03 * (state.root_lin_vel_world - state.root_lin_vel_d_world);
        pCentrifugal = std::sqrt(std::abs(/*state.root_pos(2)*/0.9) / 9.81)/2.0 * state.root_lin_vel_world.cross(state.root_ang_vel_d_world);
        // pCentrifugal = 1.0 * std::sqrt(std::abs(/*state.root_pos(2)*/0.9) / 9.81)/2.0 * state.root_lin_vel_world.cross(state.root_ang_vel_world);

        pDelta = pSymmetry + pCentrifugal + swingTimeRemain * state.root_lin_vel_world;
        pDelta[0] = (pDelta[0]>FOOT_DELTA_X_LIMIT)?(FOOT_DELTA_X_LIMIT):((pDelta[0]<-FOOT_DELTA_X_LIMIT)?(-FOOT_DELTA_X_LIMIT):pDelta[0]);
        pDelta[1] = (pDelta[1]>FOOT_DELTA_Y_LIMIT)?(FOOT_DELTA_Y_LIMIT):((pDelta[1]<-FOOT_DELTA_Y_LIMIT)?(-FOOT_DELTA_Y_LIMIT):pDelta[1]);

        foot_final_pos = pYawCorrected + pDelta;
        foot_final_pos[2] = -0.000; // height in world frame

        // if (leg == 0) {
        //     std::cout << "pSymmetry = " << pSymmetry.transpose() << ", " << "pCentrifugal = " << pCentrifugal.transpose() << "," 
        //     << "swingT*vel =  " << swingTimeRemain * state.root_lin_vel_world.transpose() 
        //     << "final x = " << foot_final_pos[0] << "y = " << foot_final_pos[1] << std::endl;
        // }


        if(foot_final_pos[0] > 2.0) {   
            // if(state.k % 10 == 0)
            foot_final_pos = SpiralBinarySearch(foot_final_pos.head(2), state.map, 8);
        }

        state.foothold_dest_world.block<3, 1>(0, leg) = foot_final_pos;
        state.foothold_dest_abs.block<3, 1>(0, leg) = state.foothold_dest_world.block<3, 1>(0, leg) - state.root_pos;
        state.foothold_dest_rel.block<3, 1>(0, leg) = state.root_rot_mat.transpose() * state.foothold_dest_abs.block<3, 1>(0, leg);
    }

    // // terrain height adaption
    // for(int leg = 0; leg < 2; leg++) { 
    //     // if(state.plan_swings_phase[leg] > 0.9) {

    //     // }
    //     if(state.foot_final_pos(0, leg) > 2.0) {
    //         float x = state.foot_final_pos(0, leg);
    //         float y = state.foot_final_pos(1, leg);
    //         int coordinate_x = int((x - 2.0)/0.02);
    //         int coordinate_y = int((y + 0.5)/0.02);
    //         foot_final_pos[2] += state.map.elevation(coordinate_x, coordinate_y);
    //         std::cout << "(x,y)=(" << coordinate_x << "," << coordinate_y << ")" << " = " << foot_final_pos[2] <<std::endl;
    //     }
    
    // }
    

    // std::cout << "test height = " << state.map.elevation  << std::endl;
    
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

                if(state.foothold_dest_world(0, leg) > 2.0) { 
                double height = (state.foothold_dest_world(2, leg) - swingtrajectory[leg]._p0(2))*3 + 0.3;
                // std::cout << "height = " << height << std::endl;
                height = height>0.15?(height<0.5?height:0.5):0.15;

                // height = 0.3;
                swingtrajectory[leg].setHeight(height);
                // std::cout << "height = " << height << std::endl;
                }

                swingtrajectory[leg].computeSwingTrajectoryBezier(state.plan_swings_phase(leg), state.gait_period * (1 - state.stance_duration(leg)));
                state.foot_pos_cmd_world.block<3, 1>(0, leg) = swingtrajectory[leg].getPosition();
                state.foot_vel_cmd_world.block<3, 1>(0, leg) = swingtrajectory[leg].getVelocity();
              
            }
        }
    } 

    #ifdef USE_REACTIVE_CONTROL
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
                // state.foot_pos_cmd_world(2, leg) = state.locked_foot_pos(2, leg) + 0.05;
                state.foot_vel_cmd_world.block<3, 1>(0, leg).setZero();
                break;
            }
            case ContactEvent::LATE_CONTACT:
            {
                // // downward motion
                // double distance = 0.08;
                // // distance = 0.08 + 0.01 * state.restance_k(leg);
                // // distance = (distance>0.2)?0.2:distance;
                
                // state.foot_pos_cmd_world(2, leg) -= distance;
                // std::cout << "leg" << leg << " z des = " << state.foot_pos_cmd_world(2, leg) << std::endl;
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
    #endif




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

Eigen::Matrix<double, 3, 1> CentaurControl::SpiralBinarySearch(
     Eigen::Matrix<double, 2, 1> initial_pos,
     map_struct map,
     int layer) {
        
        Eigen::Matrix<double, 3, 1> valid_pos; valid_pos.setZero();
        valid_pos.head(2) = initial_pos;
        int coordinate_x = int((initial_pos(0) - 2.0)/0.02);
        int coordinate_y = int((initial_pos(1) + 0.5)/0.02);
        if(map.traversability(coordinate_x, coordinate_y) > 0.99) {
            valid_pos(2) = map.elevation(coordinate_x, coordinate_y);     
            // if(initial_pos(1)>0) {
            // std::cout << "choose the default foothold! ";
            // std::cout << "initial_pos = (" << initial_pos(0) << "," <<initial_pos(1) << ")";
            // std::cout << " coordinate = (" << coordinate_x << "," <<coordinate_y << ")";
            // std::cout << " trav = " << map.traversability(coordinate_x, coordinate_y) << std::endl;
            // }
            
            return valid_pos;
        }

        int left, right, top, bottom;
        for (int layer_i = 1; layer_i < layer+1; layer_i++) {
            left = layer_i;
            right = -layer_i;
            top = layer_i;
            bottom = -layer_i;

            // [1,1], [1,0]
            for (int i = left; i > right; i--) {
                int temp_x, temp_y;
                temp_x = (coordinate_x+top<0)?0:(coordinate_x+top);
                temp_y = (coordinate_y+i);
            
                if(map.traversability(temp_x, temp_y) > 0.99) {
                    valid_pos(0) = initial_pos(0) + static_cast<float>(top)*0.02;
                    valid_pos(1) = initial_pos(1) + static_cast<float>(i)*0.02;
                    valid_pos(2) = map.elevation(temp_x, temp_y);
                    // if(initial_pos(1)>0) {
                    // std::cout << "default = [" << initial_pos(0) << "," << initial_pos(1)<<"]" << std::endl;
                    // std::cout << "modified = [" << valid_pos(0) << "," << valid_pos(1)<<"]" << std::endl;
                    // }
                    return valid_pos;
                }
            }

            // [1,-1], [0,-1]
            for (int i = top; i > bottom; i--) {
                int temp_x, temp_y;
                temp_x = (coordinate_x+i<0)?0:(coordinate_x+i);
                temp_y = (coordinate_y+right);
                
                if(map.traversability(temp_x, temp_y) > 0.99) {
                    valid_pos(0) = initial_pos(0) + static_cast<float>(i)*0.02;
                    valid_pos(1) = initial_pos(1) + static_cast<float>(right)*0.02;
                    valid_pos(2) = map.elevation(temp_x, temp_y);
                    // if(initial_pos(1)>0) {
                    // std::cout << "default  = [" << initial_pos(0) << "," << initial_pos(1)<<"]" << std::endl;
                    // std::cout << "modified = [" << valid_pos(0) << "," << valid_pos(1)<<"]" << std::endl;
                    // }
                    return valid_pos;
                }
            }
            
            // [-1,-1], [-1,0]
            for (int i = right; i < left; i++) {
                int temp_x, temp_y;
                temp_x = (coordinate_x+bottom<0)?0:(coordinate_x+bottom);
                temp_y = (coordinate_y+i);   
                if(map.traversability(temp_x, temp_y) > 0.99) {

                    valid_pos(0) = initial_pos(0) + static_cast<float>(bottom)*0.02;
                    valid_pos(1) = initial_pos(1) + static_cast<float>(i)*0.02;
                    valid_pos(2) = map.elevation(temp_x, temp_y);
                    // if(initial_pos(1)>0) {
                    // std::cout << "default  = [" << initial_pos(0) << "," << initial_pos(1)<<"]" << std::endl;
                    // std::cout << "modified = [" << valid_pos(0) << "," << valid_pos(1)<<"]" << std::endl;
                    // }
                    return valid_pos;
                }
            }

            // [-1,1], [0,1]
            for (int i = bottom; i < top; i++) {
                int temp_x, temp_y;
                temp_x = (coordinate_x+i<0)?0:(coordinate_x+i);
                temp_y = (coordinate_y+left); 
                if(map.traversability(temp_x, temp_y) > 0.99) {
                    
                    valid_pos(0) = initial_pos(0) + static_cast<float>(i)*0.02;
                    valid_pos(1) = initial_pos(1) + static_cast<float>(left)*0.02;
                    valid_pos(2) = map.elevation(temp_x, temp_y);
                    // if(initial_pos(1)>0) {
                    // std::cout << "default  = [" << initial_pos(0) << "," << initial_pos(1)<<"]" << std::endl;
                    // std::cout << "modified = [" << valid_pos(0) << "," << valid_pos(1)<<"]" << std::endl;
                    // }
                    return valid_pos;
                }
            }

        }
        
        std::cout << "optimal not found!!";
        std::cout << "initial_pos = (" << initial_pos(0) << "," <<initial_pos(1) << ")" << std::endl;

        valid_pos(2) =  valid_pos(2) = map.elevation(coordinate_x, coordinate_y);
        return valid_pos;
}

// double CentaurControl::iterateTheHeight(
//      Eigen::Matrix<double, 3, 1> from,
//      Eigen::Matrix<double, 3, 1> to,
//      double init_height,
//      map_struct map) {

//         FootSwingTrajectory<double> check_trajectory;
//         double valid_height = init_height;
//         bool valid_flag = false;
        
//         while (!valid_flag)
//         {

            
//         }
        
        
//         return valid_height;

// }