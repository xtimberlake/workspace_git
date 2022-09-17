/*
 * @Author: haoyun 
 * @Date: 2022-09-16 17:07:22
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-09-17 20:03:39
 * @FilePath: /drake/workspace/centaur_sim/controller/WBIController.h
 * @Description: Whole-body impulse controller
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once

#include "drake/workspace/centaur_sim/controller/Tasks/TorsoPosTask.hpp"
#include "drake/workspace/centaur_sim/controller/CentaurStates.h"

class WBIController
{
public:
    WBIController(/* args */);
    ~WBIController();
    void run(CentaurStates& state);
    void update_model(CentaurStates& state);
    void update_task_jacobian(CentaurStates& state);
    void kin_wbc();
    void dyn_wbc();

    TorsoPosTask<double> torso_pos_task;
    Eigen::Matrix<double, 12, 12> Hq;   // mass matrix
    Eigen::Matrix<double, 12, 1> Cv;    // nonlinear term
    Eigen::Matrix<double, 12, 1> tau_grty; // due to generalized gravity 
    Eigen::Matrix<double, 12, 1> tau_dist; // disturbance term

    
};


