/*
 * @Author: haoyun 
 * @Date: 2022-07-22 08:44:58
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-22 10:37:59
 * @FilePath: /drake/workspace/centaur_sim/controller/LegController.h
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once

#include "drake/workspace/centaur_sim/controller/CentaurStates.h"

class LegController
{
private:
    /* data */
public:
    LegController();
    Eigen::Matrix<double, 6, 1> joint_impedance_control(CentaurStates& state);
    Eigen::Matrix<double, 6, 1> task_impedance_control(CentaurStates& state);

private:
    Eigen::Matrix<double, 3, 1> _kp_swing, _kp_stance;
    Eigen::Matrix<double, 3, 1> _kd_swing, _kd_stance;
};

