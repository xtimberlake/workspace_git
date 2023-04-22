/*
 * @Author: haoyun 
 * @Date: 2022-07-16 14:31:28
 * @LastEditors: haoyun 
 * @LastEditTime: 2023-04-22 10:36:41
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
#include "drake/workspace/centaur_sim/Utils/orientationTools.h"
#include "drake/workspace/centaur_sim/Utils/pseudoInverse.h"
#include "drake/workspace/centaur_sim/Utils/cppTypes.h"
#include <type_traits>
#include "drake/workspace/centaur_sim/estimator/contactEventData.h"
#include "drake/workspace/centaur_sim/controller/global_control_flag.h"

class CentaurStates;

class CentaurControl
{
public:
    CentaurControl(const control_params_constant ctrl_params);
    void UpdateDesiredStates(CentaurStates& state);
    void GenerateSwingTrajectory(CentaurStates& state);
    void InverseKinematics(CentaurStates& state);
    void CalcHRITorques(CentaurStates& state);
    // void EstHRIForces(CentaurStates& state);
    Eigen::Matrix<double, 3, 2> ComputeGoundReactionForce(CentaurStates& state);
    Eigen::Matrix<double, 3, 1> SpiralBinarySearch(
     Eigen::Matrix<double, 2, 1> initial_pos,
     map_struct map,
     int layer);
    double iterateTheHeight(
     Eigen::Matrix<double, 3, 1> from,
     Eigen::Matrix<double, 3, 1> to,
     double swing_time,
     double init_height,
     map_struct map);
    
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

