/*
 * @Author: haoyun 
 * @Date: 2022-07-16 16:07:26
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-17 10:27:48
 * @FilePath: /drake/workspace/centaur_sim/centaurrobot/centaurrobot.h
 * @Description: define centaur robot handle
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once

#include "drake/workspace/centaur_sim/controller/CentaurStates.h"
#include "drake/workspace/centaur_sim/controller/CentaurControl.h"

class CentaurStates;
class CentaurControl;

class centaurrobot
{
private:
    /* data */
public:
    centaurrobot() {
        
    }

    // variables
    CentaurStates ctrl_states;
    CentaurControl controller;

};




