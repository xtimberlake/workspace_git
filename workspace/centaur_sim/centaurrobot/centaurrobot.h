/*
 * @Author: haoyun 
 * @Date: 2022-07-16 16:07:26
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-16 20:30:58
 * @FilePath: /drake/bazel-out/k8-opt/bin/workspace/centaur_sim/_virtual_includes/centaur_controller_lib/drake/workspace/centaur_sim/centaurrobot/centaurrobot.h
 * @Description: define centaur robot handle
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once

#include "drake/workspace/centaur_sim/controller/CentaurControlStates.h"
#include "drake/workspace/centaur_sim/controller/CentaurControl.h"

class CentaurControlStates;

class centaurrobot
{
private:
    /* data */
public:
    centaurrobot() {
        
    }

    // variables
    CentaurControlStates ctrl_states;
    

};




