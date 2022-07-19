/*
 * @Author: haoyun 
 * @Date: 2022-07-16 14:30:49
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-19 22:38:37
 * @FilePath: /drake/workspace/centaur_sim/controller/CentaurStates.h
 * @Description: define all the states that used in controller; mainly 
 *                adapted from https://github.com/ShuoYangRobotics/A1-QP-MPC-Controller
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */


#pragma once

#include <iostream>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>
#include <type_traits>

#include "drake/workspace/centaur_sim/controller/CentaurParams.h"



struct control_params_constant {
    // system
    double control_dt;

    // gait info
    double gait_resolution;

    // mpc
    int nMPC_per_period;
    int mpc_horizon;
    std::vector<double> q_weights;
    std::vector<double> r_weights;
    double mu;


    template <typename Archive>
        void Serialize(Archive* a) {
        a->Visit(DRAKE_NVP(control_dt));
        a->Visit(DRAKE_NVP(gait_resolution));
        a->Visit(DRAKE_NVP(nMPC_per_period));
        a->Visit(DRAKE_NVP(mpc_horizon));
        a->Visit(DRAKE_NVP(q_weights));
        a->Visit(DRAKE_NVP(r_weights));
        a->Visit(DRAKE_NVP(mu));
        
    }
};

struct robot_params_constant {
    
    // Inertia
    double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
    double mass;


    template <typename Archive>
        void Serialize(Archive* a) {
        a->Visit(DRAKE_NVP(mass));
        a->Visit(DRAKE_NVP(Ixx));
        a->Visit(DRAKE_NVP(Iyy));
        a->Visit(DRAKE_NVP(Izz));
        a->Visit(DRAKE_NVP(Ixy));
        a->Visit(DRAKE_NVP(Ixz));
        a->Visit(DRAKE_NVP(Iyz));
        
    }
};


class CentaurStates {
 public:
    CentaurStates() {

        this->ctrl_params_const = drake::yaml::LoadYamlFile<control_params_constant>(
          drake::FindResourceOrThrow("drake/workspace/centaur_sim/config/centaur_sim_control_params.yaml"));
        this->control_dt = ctrl_params_const.control_dt;
        this->nMPC_per_period = ctrl_params_const.nMPC_per_period;
        this->gait_resolution = ctrl_params_const.gait_resolution;
        DRAKE_DEMAND(ctrl_params_const.mpc_horizon == MPC_HORIZON);
        this->mpc_horizon = ctrl_params_const.mpc_horizon;
        this->mpc_contact_table = new int[ctrl_params_const.mpc_horizon * 2];
        this->mu = ctrl_params_const.mu;


        this->robot_params_const = drake::yaml::LoadYamlFile<robot_params_constant>(
          drake::FindResourceOrThrow("drake/workspace/centaur_sim/config/centaur_sim_robot_params.yaml"));

        this->mass = robot_params_const.mass;
        this->Ixx = robot_params_const.Ixx;
        this->Iyy = robot_params_const.Iyy;
        this->Izz = robot_params_const.Izz;
        this->Ixy = robot_params_const.Ixy;
        this->Ixz = robot_params_const.Ixz;
        this->Iyz = robot_params_const.Iyz;
        
        this->inertiaMat << this->Ixx, this->Ixy, this->Ixz,
                            this->Ixy, this->Iyy, this->Iyz,
                            this->Ixz, this->Iyz, this->Izz;
        


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
    control_params_constant ctrl_params_const;
    robot_params_constant robot_params_const;

    // system
    double t;               // time in seconds
    double control_dt;
    uint64_t k;

    // gait
    int nMPC_per_period;
    double gait_resolution;
    Eigen::Vector2f plan_contacts_phase;
    Eigen::Vector2f plan_swings_phase;


    // mpc
    int mpc_horizon;
    int* mpc_contact_table;
    double mu;

    // robot's inertia
    double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
    double mass;
    Eigen::Matrix3d inertiaMat;

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

    Eigen::Matrix<double, 3, 2> foot_pos_world;
    Eigen::Matrix<double, 3, 2> foot_pos_rel;

    

};