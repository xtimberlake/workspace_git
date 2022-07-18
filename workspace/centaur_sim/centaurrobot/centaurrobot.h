/*
 * @Author: haoyun 
 * @Date: 2022-07-16 16:07:26
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-18 10:02:02
 * @FilePath: /drake/workspace/centaur_sim/centaurrobot/centaurrobot.h
 * @Description: define centaur robot handle
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once

#include "drake/workspace/centaur_sim/controller/CentaurStates.h"
#include "drake/workspace/centaur_sim/controller/CentaurControl.h"
#include "drake/workspace/centaur_sim/controller/CentaurGaitPattern.h"


class centaurrobot
{
private:
    /* data */
public:
    centaurrobot() 
    
    {
        
        walking = new CentuarGaitPattern(0.5, ctrl_states.gait_resolution, 10, Eigen::Vector2f(0.5, 0.5), Eigen::Vector2f(0.0, 0.5)); 
        standing = new CentuarGaitPattern(0.5, ctrl_states.gait_resolution, 10, Eigen::Vector2f(1.0, 1.0), Eigen::Vector2f(0.0, 0.5)); 
    }

    // variables
    CentaurStates ctrl_states;
    CentaurControl controller;
    CentuarGaitPattern* walking;
    CentuarGaitPattern* standing;

};




