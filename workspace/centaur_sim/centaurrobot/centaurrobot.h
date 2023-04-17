/*
 * @Author: haoyun 
 * @Date: 2022-07-16 16:07:26
 * @LastEditors: haoyun 
 * @LastEditTime: 2023-04-17 20:02:12
 * @FilePath: /drake/workspace/centaur_sim/centaurrobot/centaurrobot.h
 * @Description: define centaur robot handle
 * 
 * Copyright (c) 2022 by HARR-Lab, All Rights Reserved. 
 */
#pragma once

#include "drake/workspace/centaur_sim/controller/CentaurStates.h"
#include "drake/workspace/centaur_sim/controller/CentaurControl.h"
#include "drake/workspace/centaur_sim/controller/CentaurGaitPattern.h"
#include "drake/workspace/centaur_sim/controller/LegController.h"
#include "drake/workspace/centaur_sim/controller/WBIController.h"
// #include "drake/workspace/centaur_sim/dynamics/FloatingBaseModel.h"
#include "drake/workspace/centaur_sim/dynamics/CentaurModel.h"
#include "drake/workspace/centaur_sim/estimator/contactEstimate.h"


class centaurrobot
{
public:
    centaurrobot() 
    
    {
        
        standing = new CentuarGaitPattern(0.5, ctrl_states.ctrl_params_const, Eigen::Vector2f(1.0, 1.0), Eigen::Vector2f(0.0, 0.5)); 
        walking = new CentuarGaitPattern(0.9, ctrl_states.ctrl_params_const, Eigen::Vector2f(0.6, 0.6), Eigen::Vector2f(0.0, 0.5)); 
        // jumping = new CentuarGaitPattern(0.5, ctrl_states.ctrl_params_const, Eigen::Vector2f(0.75, 0.75), Eigen::Vector2f(0.0, 0.0)); 
        // fly_trotting = new CentuarGaitPattern(0.5, ctrl_states.ctrl_params_const, Eigen::Vector2f(0.3, 0.3), Eigen::Vector2f(0.0, 0.5)); 

        controller = new CentaurControl(ctrl_states.ctrl_params_const);
        
        legcontroller = new LegController();

        wbicontroller = new WBIController();

        contactestimate = new contactEstimate<double>();

    }

    // variables
    CentaurStates ctrl_states;
    contactEstimate<double>* contactestimate; 
    CentaurControl* controller;
    LegController* legcontroller;
    WBIController* wbicontroller;
    
    CentuarGaitPattern* standing;
    FloatingBaseModel centaur_dynamics_model;
    CentuarGaitPattern* walking;
    CentuarGaitPattern* jumping;
    // CentuarGaitPattern* fly_trotting;


};




