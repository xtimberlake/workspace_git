/*
 * @Author: haoyun 
 * @Date: 2022-07-17 11:26:19
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-12-13 17:30:49
 * @FilePath: /drake/home/haoyun/.cache/bazel/_bazel_haoyun/a17c303983e829fea1540ab5133f0aae/execroot/drake/bazel-out/k8-opt/bin/workspace/centaur_sim/_virtual_includes/centaur_controller_lib/drake/workspace/centaur_sim/controller/CentaurGaitPattern.h
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */

#pragma once

#include <iostream>
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/workspace/centaur_sim/controller/CentaurStates.h"
#include "drake/workspace/centaur_sim/controller/global_control_flag.h"



class CentuarGaitPattern
{
private:
    double _gait_period;
    Eigen::Vector2f _stance_duration;
    Eigen::Vector2f _swing_duration;
    Eigen::Vector2f _offset;


    double _gait_total_counter;
    double _gait_counter_speed;
    double _gait_counter;
    double _phase;
    int _iterationsPerMPC;
    int _nMPC_per_period;
    
    
public:
    CentuarGaitPattern(double gait_period,
                        const control_params_constant ctrl_params,
                        Eigen::Vector2f stance_duration,
                        Eigen::Vector2f offset);

    void update_gait_pattern(CentaurStates& state);
    void reset(int reset_type);
    

};


