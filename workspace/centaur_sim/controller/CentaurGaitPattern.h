/*
 * @Author: haoyun 
 * @Date: 2022-07-17 11:26:19
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-17 20:22:22
 * @FilePath: /drake/workspace/centaur_sim/controller/CentaurGaitPattern.h
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */

#pragma once

#include <iostream>
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/workspace/centaur_sim/controller/CentaurStates.h"

class CentaurStates;

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
                        double gait_resolution,
                        int nMPC_per_period, 
                        Eigen::Vector2f stance_duration,
                        Eigen::Vector2f offset);

    void update_gait_pattern(CentaurStates& state);
    void reset();
    

};


