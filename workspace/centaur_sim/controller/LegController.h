/*
 * @Author: haoyun 
 * @Date: 2022-07-22 08:44:58
 * @LastEditors: haoyun 
 * @LastEditTime: 2023-06-13 17:13:21
 * @FilePath: /drake/workspace/centaur_sim/controller/LegController.h
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once

#include "drake/workspace/centaur_sim/controller/CentaurStates.h"
#include "drake/workspace/centaur_sim/controller/global_control_flag.h"

class LegController
{
private:
    /* data */
public:
    LegController();
    Eigen::Matrix<double, 6, 1> joint_impedance_control(CentaurStates& state);
    Eigen::Matrix<double, 6, 1> task_impedance_control(CentaurStates& state);
    Eigen::Matrix<double, 6, 1> wbc_low_level_control(CentaurStates& state);
    Eigen::Matrix<double, 6, 1> wbc_feedforward_ik_control(CentaurStates& state);
    Eigen::Matrix<double, 12, 1> inverse_kinematics(CentaurStates& state);
    void debug_ik(CentaurStates& state);

private:
    Eigen::Matrix<double, 3, 1> _kp_swing, _kp_stance;
    Eigen::Matrix<double, 3, 1> _kd_swing, _kd_stance;

    // joint level gain
    Eigen::Matrix<double, 3, 1> _kp_joint_swing, _kd_joint_swing;
    Eigen::Matrix<double, 3, 1> _kp_joint_stance, _kd_joint_stance;

    Eigen::Matrix<double, 3, 1> _kp_pure_joint_stance, _kd_pure_joint_stance;
    Eigen::Matrix<double, 3, 1> _kp_pure_joint_swing, _kd_pure_joint_swing;

};

