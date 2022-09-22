/*
 * @Author: haoyun 
 * @Date: 2022-07-16 16:07:26
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-09-22 19:28:18
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
#include "drake/workspace/centaur_sim/dynamics/FloatingBaseModel.h"
#include "drake/workspace/centaur_sim/dynamics/CentaurModel.h"


class centaurrobot
{
public:
    centaurrobot() 
    
    {
        
        standing = new CentuarGaitPattern(0.5, ctrl_states.ctrl_params_const, Eigen::Vector2f(1.0, 1.0), Eigen::Vector2f(0.0, 0.5)); 
        walking = new CentuarGaitPattern(0.5, ctrl_states.ctrl_params_const, Eigen::Vector2f(0.5, 0.5), Eigen::Vector2f(0.0, 0.5)); 
        // jumping = new CentuarGaitPattern(0.5, ctrl_states.ctrl_params_const, Eigen::Vector2f(0.7, 0.7), Eigen::Vector2f(0.0, 0.0)); 
        // galloping = new CentuarGaitPattern(0.5, ctrl_states.ctrl_params_const, Eigen::Vector2f(0.5, 0.5), Eigen::Vector2f(0.0, 0.25)); 

        controller = new CentaurControl(ctrl_states.ctrl_params_const);
        
        legcontroller = new LegController();

        ctModel.buildModel();

    }

    // variables
    CentaurStates ctrl_states;
    CentaurControl* controller;
    LegController* legcontroller;
    WBIController* wbicontroller;
    CentuarGaitPattern* walking;
    CentuarGaitPattern* standing;
    FloatingBaseModel centaur_dynamics_model;
    CentaurModel ctModel;
    // CentuarGaitPattern* jumping;
    // CentuarGaitPattern* galloping;


};




