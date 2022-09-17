/*
 * @Author: haoyun 
 * @Date: 2022-07-16 14:30:49
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-09-17 09:52:31
 * @FilePath: /drake/workspace/centaur_sim/controller/CentaurStates.h
 * @Description: define all the states that used in controller; mainly 
 *                adapted from https://github.com/ShuoYangRobotics/A1-QP-MPC-Controller
 * 
 * Copyright (c) 2022 by HARR-Lab, All Rights Reserved. 
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
    int nIterationsPerMPC;
    int mpc_horizon;
    std::vector<double> q_weights;
    std::vector<double> r_weights;
    double mu;

    // swing
    std::vector<double> default_foot_pos_under_hip;

    // ik
    int max_iter;
    double ik_eps;


    template <typename Archive>
        void Serialize(Archive* a) {
        a->Visit(DRAKE_NVP(control_dt));
        a->Visit(DRAKE_NVP(gait_resolution));
        a->Visit(DRAKE_NVP(nIterationsPerMPC));
        a->Visit(DRAKE_NVP(mpc_horizon));
        a->Visit(DRAKE_NVP(q_weights));
        a->Visit(DRAKE_NVP(r_weights));
        a->Visit(DRAKE_NVP(mu));
        a->Visit(DRAKE_NVP(default_foot_pos_under_hip));
        a->Visit(DRAKE_NVP(max_iter));
        a->Visit(DRAKE_NVP(ik_eps));
        
    }
};

struct robot_params_constant {
    
    // Inertia
    double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
    double mass;
    std::vector<double> sphere_joint_location;
    std::vector<double> left_hip_location;


    template <typename Archive>
        void Serialize(Archive* a) {
        a->Visit(DRAKE_NVP(mass));
        a->Visit(DRAKE_NVP(Ixx));
        a->Visit(DRAKE_NVP(Iyy));
        a->Visit(DRAKE_NVP(Izz));
        a->Visit(DRAKE_NVP(Ixy));
        a->Visit(DRAKE_NVP(Ixz));
        a->Visit(DRAKE_NVP(Iyz));
        a->Visit(DRAKE_NVP(sphere_joint_location));
        a->Visit(DRAKE_NVP(left_hip_location));
        
    }
};


class CentaurStates {
 public:
    CentaurStates() {

        this->ctrl_params_const = drake::yaml::LoadYamlFile<control_params_constant>(
          drake::FindResourceOrThrow("drake/workspace/centaur_sim/config/centaur_sim_control_params.yaml"));
        this->control_dt = ctrl_params_const.control_dt;
        this->nIterationsPerMPC = ctrl_params_const.nIterationsPerMPC;
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
        
        for (int i = 0; i < 3; i++) {
            this->sphere_joint_location(i) = robot_params_const.sphere_joint_location.at(i);
        }

        // swing
        this->default_foot_pos_rel.block<3, 1>(0, 0) <<
            robot_params_const.left_hip_location.at(0) + ctrl_params_const.default_foot_pos_under_hip.at(0),
            robot_params_const.left_hip_location.at(1) + ctrl_params_const.default_foot_pos_under_hip.at(1),
            robot_params_const.left_hip_location.at(2) + ctrl_params_const.default_foot_pos_under_hip.at(2);

        this->default_foot_pos_rel.block<3, 1>(0, 1) <<
            robot_params_const.left_hip_location.at(0) + ctrl_params_const.default_foot_pos_under_hip.at(0),
            FILP_DIR * robot_params_const.left_hip_location.at(1) + ctrl_params_const.default_foot_pos_under_hip.at(1),
            robot_params_const.left_hip_location.at(2) + ctrl_params_const.default_foot_pos_under_hip.at(2);

        // ik
        this->max_iter = ctrl_params_const.max_iter;
        this->ik_eps = ctrl_params_const.ik_eps;


        this->k = 0;

        // default desired states
        this->root_pos_d << 0.0, 0.0, 0.9;
        this->root_euler_d.setZero();
        this->root_lin_vel_d.setZero();
        this->root_lin_vel_d_world.setZero();
        this->root_ang_vel_d.setZero();
        this->root_ang_vel_d_world.setZero();
      
        // Others:
        this->external_wrench << 0, 0, 0, 0, 0, 15;
    }

    // variables
    control_params_constant ctrl_params_const;
    robot_params_constant robot_params_const;

    // system
    double t;               // time in seconds
    double control_dt;
    uint64_t k;

    // gait
    int nIterationsPerMPC;
    double gait_resolution;
    Eigen::Vector2f plan_contacts_phase;
    Eigen::Vector2f plan_swings_phase;
    double gait_period; //symmetric gait
    Eigen::Vector2f stance_duration;


    // mpc
    int mpc_horizon;
    int* mpc_contact_table;
    double mu;

    // robot's phsical configuration
    double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
    double mass;
    Eigen::Matrix3d inertiaMat;
    Eigen::Vector3d sphere_joint_location;


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
    Eigen::Matrix<double, 3, 2> foot_pos_abs; // from CoM to foot expressed in the world frame

    Eigen::Matrix<double, 3, 2> foot_vel_world;
    Eigen::Matrix<double, 3, 2> foot_vel_rel;

    Eigen::Matrix<double, 3, 2> default_foot_pos_rel;
    Eigen::Matrix<double, 3, 2> foothold_dest_rel;    // destination
    Eigen::Matrix<double, 3, 2> foothold_dest_abs;
    Eigen::Matrix<double, 3, 2> foothold_dest_world;

    Eigen::Matrix<double, 3, 2> foot_pos_cmd_rel;    // command
    Eigen::Matrix<double, 3, 2> foot_pos_cmd_abs;
    Eigen::Matrix<double, 3, 2> foot_pos_cmd_world;

    Eigen::Matrix<double, 3, 2> foot_vel_cmd_rel;    // command
    Eigen::Matrix<double, 3, 2> foot_vel_cmd_world;
    
    Eigen::Matrix<double, 3, 2> foot_force_cmd_rel;    // command
    Eigen::Matrix<double, 3, 2> foot_force_cmd_world;

    Eigen::Matrix<double, 3, 3> JacobianFoot[2];

    // motors
    Eigen::Matrix<double, 6, 1> q, qdot, q_cmd, qdot_cmd, tau_ff;
    Eigen::Matrix<double, 6, 1> tau;

    // ik
    int max_iter;
    double ik_eps;

    Eigen::Matrix<double, 3, 2> foot_force_kin;

    // system dynamics:
    // M(q)v̇ + C(q, v)v = tau_g + tau + ∑ J_WBᵀ(q) Fapp_Bo_W
    Eigen::Matrix<double, 12, 12> Mq;   // mass matrix
    Eigen::Matrix<double, 12, 1> Cv;    // bias term
    Eigen::Matrix<double, 12, 1> tau_g; // generalized forces due to gravity


    // Others
    Eigen::Matrix<double, 6, 1> external_wrench;
    

};