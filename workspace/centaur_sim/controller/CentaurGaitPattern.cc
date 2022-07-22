/*
 * @Author: haoyun 
 * @Date: 2022-07-17 11:21:35
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-22 19:45:07
 * @FilePath: /drake/workspace/centaur_sim/controller/CentaurGaitPattern.cc
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#include "drake/workspace/centaur_sim/controller/CentaurGaitPattern.h"


CentuarGaitPattern::CentuarGaitPattern(double gait_period,
                        const control_params_constant ctrl_params,
                        Eigen::Vector2f stance_duration,
                        Eigen::Vector2f offset)
{
    this->_gait_period = gait_period;
    this->_gait_total_counter = ctrl_params.gait_resolution;

    this->_iterationsPerMPC = ctrl_params.nIterationsPerMPC;
    this->_nMPC_per_period = static_cast<int>(gait_period / (ctrl_params.nIterationsPerMPC * ctrl_params.control_dt));

    drake::log()->info("_nMPC_per_period");
    drake::log()->info(this->_nMPC_per_period);
    this->_stance_duration = stance_duration;
    this->_offset = offset;
    
    this->_swing_duration = Eigen::Vector2f(1.0, 1.0) - _stance_duration;

}

/**
 * @description: Calculate plan contacts, swings, MPCtable for this gait
 * @param {CentaurStates&} state
 * @return {*}
 */
void CentuarGaitPattern::update_gait_pattern(CentaurStates& state)
{
    state.gait_period = this->_gait_period;
    state.stance_duration = this->_stance_duration;
    _gait_counter_speed = _gait_total_counter / (_gait_period / state.control_dt);
    _gait_counter += _gait_counter_speed;
    _gait_counter = std::fmod(_gait_counter, _gait_total_counter);
   
    _phase = _gait_counter / _gait_total_counter;

    for (size_t i = 0; i < NUM_LEG; i++) // two legs
    {
        // stance states
        double progress = _phase - _offset(i);
        if(progress < 0) progress += 1;
        if(progress < _stance_duration(i)) state.plan_contacts_phase(i) = progress / _stance_duration(i);
        else state.plan_contacts_phase(i) = 0;

        // swing states
        double swing_offset = _offset(i) + _stance_duration(i);
        if(swing_offset > 1) swing_offset -= 1;
        progress = _phase - swing_offset;
        if(progress < 0) progress += 1;
        if(progress < _swing_duration(i)) state.plan_swings_phase(i) = progress / _swing_duration(i);
        else state.plan_swings_phase(i) = 0;        

    }

    // mpc table
    double mpc_incremental_phase = 1.f / static_cast<double>(_nMPC_per_period);

    for (int i = 0; i < MPC_HORIZON; i++)
    {
        double next_phase = _phase + mpc_incremental_phase * i;
        next_phase = std::fmod(next_phase, 1.0f);
        if(next_phase > 1.f) next_phase -= 1.f;
        // drake::log()->info(next_phase);
        for (size_t j = 0; j < NUM_LEG; j++)  // two legs
        {
            double progress = next_phase - _offset(j);
            if(progress < 0) progress += 1;
            if(progress < _stance_duration(j)) state.mpc_contact_table[i*2 + j] = 1;
            else state.mpc_contact_table[i*2 + j] = 0;
        }

    }
    
    // drake::log()->info("left :" + std::to_string(state.mpc_contact_table[0*2 + 0]) + ", "
    //     + std::to_string(state.mpc_contact_table[1*2 + 0]) + ", "
    //     + std::to_string(state.mpc_contact_table[2*2 + 0]) + ", "
    //     + std::to_string(state.mpc_contact_table[3*2 + 0]) + ", "
    //     + std::to_string(state.mpc_contact_table[4*2 + 0]) + ", "
    //     + std::to_string(state.mpc_contact_table[5*2 + 0]) + ", "
    //     + std::to_string(state.mpc_contact_table[6*2 + 0]) + ", "
    //     + std::to_string(state.mpc_contact_table[7*2 + 0]) + ", "
    //     + std::to_string(state.mpc_contact_table[8*2 + 0]) + ", "
    //     + std::to_string(state.mpc_contact_table[9*2 + 0]) + ", "
    //     + " right:"  
    //     + std::to_string(state.mpc_contact_table[0*2 + 1]) + ", "
    //     + std::to_string(state.mpc_contact_table[1*2 + 1]) + ", "
    //     + std::to_string(state.mpc_contact_table[2*2 + 1]) + ", "
    //     + std::to_string(state.mpc_contact_table[3*2 + 1]) + ", "
    //     + std::to_string(state.mpc_contact_table[4*2 + 1]) + ", "
    //     + std::to_string(state.mpc_contact_table[5*2 + 1]) + ", "
    //     + std::to_string(state.mpc_contact_table[6*2 + 1]) + ", "
    //     + std::to_string(state.mpc_contact_table[7*2 + 1]) + ", "
    //     + std::to_string(state.mpc_contact_table[8*2 + 1]) + ", "
    //     + std::to_string(state.mpc_contact_table[9*2 + 1]) + ", ");

}


void CentuarGaitPattern::reset() 
{
    _gait_counter = 0.0;
}
