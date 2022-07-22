/*
 * @Author: haoyun 
 * @Date: 2022-07-16 16:07:26
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-22 09:09:49
 * @FilePath: /drake/workspace/centaur_sim/centaurrobot/centaurrobot.h
 * @Description: define centaur robot handle
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once

#include "drake/workspace/centaur_sim/controller/CentaurStates.h"
#include "drake/workspace/centaur_sim/controller/CentaurControl.h"
#include "drake/workspace/centaur_sim/controller/CentaurGaitPattern.h"
#include "drake/workspace/centaur_sim/controller/LegController.h"


class centaurrobot
{
private:
    /* data */
public:
    centaurrobot() 
    
    {
        
        walking = new CentuarGaitPattern(0.5, ctrl_states.ctrl_params_const, Eigen::Vector2f(0.5, 0.5), Eigen::Vector2f(0.0, 0.5)); 
        standing = new CentuarGaitPattern(0.5, ctrl_states.ctrl_params_const, Eigen::Vector2f(1.0, 1.0), Eigen::Vector2f(0.0, 0.5)); 
        controller = new CentaurControl(ctrl_states.ctrl_params_const);
        legcontroller = new LegController();

    }

    // variables
    CentaurStates ctrl_states;
    CentaurControl* controller;
    LegController* legcontroller;
    CentuarGaitPattern* walking;
    CentuarGaitPattern* standing;

};




