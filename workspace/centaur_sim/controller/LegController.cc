/*
 * @Author: haoyun 
 * @Date: 2022-07-22 08:44:58
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-27 22:17:24
 * @FilePath: /drake/workspace/centaur_sim/controller/LegController.cc
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#include "LegController.h"

LegController::LegController()
{
    this->_kp_stance << 5.0, 2, 2;
    this->_kd_stance << 5.0, 2, 2;

    this->_kp_swing << 100.0, 100, 50;
    this->_kd_swing << 200.0, 200, 100;
}

Eigen::Matrix<double, 6, 1> LegController::joint_impedance_control(CentaurStates& state){

    Eigen::Matrix<double, 6, 1> torques;
    

    for (int leg = 0; leg < 2; leg++)
    {
        if (state.plan_contacts_phase(leg) > .99) {
            torques.segment<3>(3 * leg) = 
                _kp_stance.cwiseProduct(state.q_cmd.segment<3>(3 * leg) - state.q_cmd.segment<3>(3 * leg))
                + _kd_stance.cwiseProduct(state.qdot_cmd.segment<3>(3 * leg) - state.qdot.segment<3>(3 * leg))
                + state.tao_ff.segment<3>(3 * leg);
        }
        else {
           torques.segment<3>(3 * leg) = 
                _kp_swing.cwiseProduct(state.q_cmd.segment<3>(3 * leg) - state.q_cmd.segment<3>(3 * leg))
                + _kd_swing.cwiseProduct(state.qdot_cmd.segment<3>(3 * leg) - state.qdot.segment<3>(3 * leg));
        }
    }

    return torques;
}

Eigen::Matrix<double, 6, 1> LegController::task_impedance_control(CentaurStates& state){

    Eigen::Matrix<double, 6, 1> torques;
    Eigen::Matrix<double, 3, 1> pos_error, vel_error;
    float full_end = 0.2;
    float start_rate = 0.2;
    float decay_coeffienct;
    for (int leg = 0; leg < 2; leg++)
    {
        if (state.plan_contacts_phase(leg) > 0) {
            //soft landing
            // torques.segment<3>(3 * leg) = state.tao_ff.segment<3>(3 * leg);
            if(state.plan_contacts_phase(leg) >= full_end) {
                torques.segment<3>(3 * leg) = state.tao_ff.segment<3>(3 * leg);
            }
            else {
                decay_coeffienct = state.plan_contacts_phase(leg) / full_end * (1 - start_rate) + start_rate;
                torques.segment<3>(3 * leg) = decay_coeffienct * state.tao_ff.segment<3>(3 * leg);
            }
        }
        else {

            pos_error = state.foot_pos_cmd_rel.block<3, 1>(0, leg) - state.foot_pos_rel.block<3, 1>(0, leg);
            vel_error = state.foot_vel_cmd_rel.block<3, 1>(0, leg) - state.foot_vel_rel.block<3, 1>(0, leg);
            state.foot_force_kin.block<3, 1>(0, leg) = _kp_swing.cwiseProduct(pos_error) + _kd_swing.cwiseProduct(vel_error);
            // drake::log()->info(state.foot_force_kin);
            torques.segment<3>(3 * leg) = state.JacobianFoot[leg].lu().solve(state.foot_force_kin.block<3, 1>(0, leg));
        }
    }

    return torques;
}