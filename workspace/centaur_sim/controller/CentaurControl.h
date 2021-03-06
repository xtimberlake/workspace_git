/*
 * @Author: haoyun 
 * @Date: 2022-07-16 14:31:28
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-21 10:52:34
 * @FilePath: /drake/workspace/centaur_sim/controller/CentaurControl.h
 * @Description: centaur root controller
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once

#include "drake/workspace/centaur_sim/controller/CentaurStates.h"
#include "drake/workspace/centaur_sim/controller/ConvexMPC.h"
#include "drake/workspace/centaur_sim/controller/CentaurParams.h"
#include "drake/workspace/centaur_sim/Utils/Utils.h"
#include <type_traits>

class CentaurStates;

class CentaurControl
{
public:
    CentaurControl(const control_params_constant ctrl_params);
    void GenerateSwingTrajectory(CentaurStates& state);
    void InverseKinematics(CentaurStates& state);
    Eigen::Matrix<double, 3, 2> ComputeGoundReactionForce(CentaurStates& state);
    
    // variables:
    

    ConvexMPC *mpc_solver;
    int mpc_horizon;
    double mpc_dt;
    double mu;
    
    Eigen::Matrix<double, NUM_STATE, 1> mpc_q_weights;
    Eigen::Matrix<double, NUM_U, 1> mpc_r_weights;

    // swing
    FootSwingTrajectory<double>  swingtrajectory[2];
    bool FirstTimeSwing;
};

