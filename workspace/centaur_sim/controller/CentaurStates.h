/*
 * @Author: haoyun 
 * @Date: 2022-07-16 14:30:49
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-17 21:02:38
 * @FilePath: /drake/workspace/centaur_sim/controller/CentaurStates.h
 * @Description: define all the states that used in controller; mainly 
 *                adapted from https://github.com/ShuoYangRobotics/A1-QP-MPC-Controller
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */


#pragma once

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>



struct control_params_constant {
    // system
    double control_dt;

    // gait info
    double gait_resolution;

    // mpc
    int nMPC_per_period;
    int mpc_horizon;

    template <typename Archive>
        void Serialize(Archive* a) {
        a->Visit(DRAKE_NVP(control_dt));
        a->Visit(DRAKE_NVP(gait_resolution));
        a->Visit(DRAKE_NVP(nMPC_per_period));
        a->Visit(DRAKE_NVP(mpc_horizon));
        
    }
};


class CentaurStates {
 public:
    CentaurStates() {

        const control_params_constant ctrl_params_const = drake::yaml::LoadYamlFile<control_params_constant>(
          drake::FindResourceOrThrow("drake/workspace/centaur_sim/config/centaur_sim_control_params.yaml"));
        
        this->control_dt = ctrl_params_const.control_dt;
        this->nMPC_per_period = ctrl_params_const.nMPC_per_period;
        this->gait_resolution = ctrl_params_const.gait_resolution;
        this->mpc_horizon = ctrl_params_const.mpc_horizon;
        this->mpc_contact_table = new int[ctrl_params_const.mpc_horizon * 2];


        this->k = 0;

        // default desired states
        this->root_pos_d << 0.0, 0.0, 0.9;
        this->root_euler_d.setZero();
        this->root_lin_vel_d.setZero();
        this->root_lin_vel_d_world.setZero();
        this->root_ang_vel_d.setZero();
        this->root_ang_vel_d_world.setZero();
      

    }

    // variables

    // system
    double t;               // time in seconds
    double control_dt;
    uint64_t k;

    // gait
    int nMPC_per_period;
    double gait_resolution;

    // mpc
    int mpc_horizon;
    int* mpc_contact_table;
    
    Eigen::Vector2f plan_contacts_phase;
    Eigen::Vector2f plan_swings_phase;

    // robot's states
    Eigen::Quaterniond root_quat;
    Eigen::Vector3d root_pos;
    Eigen::Vector3d root_ang_vel;
    Eigen::Vector3d root_lin_vel;
    Eigen::Matrix3d root_rot_mat;
    Eigen::Matrix3d root_rot_mat_z;  
    Eigen::Vector3d root_euler;
    Eigen::Vector3d root_acc;

    Eigen::Vector3d root_pos_d;
    Eigen::Vector3d root_euler_d;
    Eigen::Vector3d root_lin_vel_d;
    Eigen::Vector3d root_lin_vel_d_world;
    Eigen::Vector3d root_ang_vel_d;
    Eigen::Vector3d root_ang_vel_d_world;

    

};