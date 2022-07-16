/*
 * @Author: haoyun 
 * @Date: 2022-07-16 14:30:49
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-16 19:58:00
 * @FilePath: /drake/workspace/centaur_sim/controller/CentaurControlStates.h
 * @Description: define all the states that used in controller, mainly 
 *                adapted from https://github.com/ShuoYangRobotics/A1-QP-MPC-Controller
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */


#pragma once

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>


struct control_params_constant {
    double gait_period;

    template <typename Archive>
        void Serialize(Archive* a) {
        a->Visit(DRAKE_NVP(gait_period));
        
    }
};


class CentaurControlStates {
 public:
    CentaurControlStates() {
      const control_params_constant ctrl_params_const = drake::yaml::LoadYamlFile<control_params_constant>(
          drake::FindResourceOrThrow("drake/workspace/centaur_sim/config/centaur_sim_control_params.yaml"));
      this->gait_period = ctrl_params_const.gait_period;



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

    // gait
    double gait_period;

    // robot's states
    Eigen::Vector3d root_pos;
    Eigen::Quaterniond root_quat;
    Eigen::Vector3d root_euler;
    Eigen::Matrix3d root_rot_mat;
    Eigen::Matrix3d root_rot_mat_z;
    Eigen::Vector3d root_lin_vel;
    Eigen::Vector3d root_ang_vel;
    Eigen::Vector3d root_acc;

    Eigen::Vector3d root_pos_d;
    Eigen::Vector3d root_euler_d;
    Eigen::Vector3d root_lin_vel_d;
    Eigen::Vector3d root_lin_vel_d_world;
    Eigen::Vector3d root_ang_vel_d;
    Eigen::Vector3d root_ang_vel_d_world;

    

};