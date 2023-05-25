/*
 * @Author: haoyun 
 * @Date: 2022-07-22 08:44:58
 * @LastEditors: haoyun 
 * @LastEditTime: 2023-05-25 14:52:32
 * @FilePath: /drake/workspace/centaur_sim/controller/LegController.cc
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#include "LegController.h"

LegController::LegController()
{
    // this->_kp_stance << 5.0, 2, 2;
    // this->_kd_stance << 5.0, 2, 2;

    // this->_kp_swing << 100.0, 100, 50;
    // this->_kd_swing << 200.0, 200, 100;

    // this->_kp_stance << 12.5, 5, 5;
    // this->_kd_stance << 10.0, 10, 12.5;

    this->_kp_stance << 120, 120, 120;
    this->_kd_stance << 10.0, 10, 12.5;

    this->_kp_swing << 200.0, 200, 150;
    this->_kd_swing << 30.0, 20, 10;

    // // wbc 
    // these parameters are quite crutial to retrieve stability!
    this->_kp_joint_stance << 220.0, 220.0, 220.0;
    this->_kd_joint_stance << 50.0, 50, 50;

    this->_kp_joint_swing << 500.0, 500.0, 600.0;
    this->_kd_joint_swing << 80.0, 80.0, 80.0;


    // this->_kp_joint_stance << 120.0, 100.0, 120.0;
    // this->_kd_joint_stance << 200.0, 200.0, 200.0;

    // this->_kp_joint_swing << 400.0, 1000.0, 1500.0;
    // this->_kd_joint_swing << 1000.0, 2000.0, 2000.0;


    // no wbc
    // this->_kp_joint_stance << 200.0, 200.0, 200.0;
    // this->_kd_joint_stance << 10.0, 10.0, 10.0;

    // this->_kp_joint_swing << 1000.0, 1500.0, 1500.0;
    // this->_kd_joint_swing << 200.0, 300.0, 300.0;

}

Eigen::Matrix<double, 6, 1> LegController::joint_impedance_control(CentaurStates& state){

    Eigen::Matrix<double, 6, 1> torques;
    Eigen::Matrix<double, 3, 1> kp, kd;  kp.setZero(); kd.setZero();

    for (int leg = 0; leg < 2; leg++)
    {
        // choose impedance parameters
        #ifdef USE_REACTIVE_CONTROL
        switch (state.foot_contact_event[leg])
        {
        case ContactEvent::SWING: 
        case ContactEvent::LATE_CONTACT:
        {
            kp = _kp_joint_swing;
            kd = _kd_joint_swing;
            break;
        }
        case ContactEvent::EARLY_CONTACT:
        case ContactEvent::STANCE:
        case ContactEvent::RESTANCE:
        default:
        {
            kp = _kp_joint_stance;
            kd = _kd_joint_stance;
            break;
        }
        }
        #else
        if(state.plan_contacts_phase(leg) > 0) {
            kp = _kp_joint_stance;
            kd = _kd_joint_stance;

        } else {
            kp = _kp_joint_swing;
            kd = _kd_joint_swing;
        }
        #endif

        if (state.plan_contacts_phase(leg) > 0.0) {
            torques.segment<3>(3 * leg) = 
                kp.cwiseProduct(state.q_cmd.segment<3>(3 * leg) - state.q.segment<3>(3 * leg))
                + kd.cwiseProduct(state.qdot_cmd.segment<3>(3 * leg) - state.qdot.segment<3>(3 * leg))
                + state.tau_ff.segment<3>(3 * leg);
        }
        else {
           torques.segment<3>(3 * leg) = 
                kp.cwiseProduct(state.q_cmd.segment<3>(3 * leg) - state.q.segment<3>(3 * leg))
                + kd.cwiseProduct(state.qdot_cmd.segment<3>(3 * leg) - state.qdot.segment<3>(3 * leg));
        }
    }

    for (int i = 0; i < 6; i++)
    {
        torques(i) = (torques(i)>800)?(800):(torques(i)<-800?-800:torques(i));
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

            // (static) Jacobian mapping
            state.tau_ff.segment<3>(3 * leg) = -state.JacobianFoot[leg].transpose() * state.foot_force_cmd_rel.block<3, 1>(0, leg);


            //soft landing
            if(state.plan_contacts_phase(leg) >= full_end) {
                torques.segment<3>(3 * leg) = state.tau_ff.segment<3>(3 * leg);
            }
            else {
                decay_coeffienct = (1 - start_rate) * sin(M_PI_2 * state.plan_contacts_phase(leg) / full_end)  + start_rate;
                torques.segment<3>(3 * leg) = decay_coeffienct * state.tau_ff.segment<3>(3 * leg);
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

    state.tau = torques;

    return torques;
}

Eigen::Matrix<double, 6, 1> LegController::wbc_low_level_control(CentaurStates& state){
    Eigen::Matrix<double, 6, 1> torques; torques.setZero();

    Eigen::Matrix<double, 3, 1> kp, kd;  kp.setZero(); kd.setZero();
    for (int leg = 0; leg < 2; leg++)
    {
        // choose impedance parameters
        #ifdef USE_REACTIVE_CONTROL
        switch (state.foot_contact_event[leg])
        {
        case ContactEvent::SWING: 
        case ContactEvent::LATE_CONTACT:
        {
            kp = _kp_joint_swing;
            kd = _kd_joint_swing;
            // std::cout << kp(0) << std::endl;
            break;
        }
        case ContactEvent::EARLY_CONTACT:
        case ContactEvent::STANCE:
        case ContactEvent::RESTANCE:
        default:
        {
            kp = _kp_joint_stance;
            kd = _kd_joint_stance;
            break;
        }
        }
        #else
        if(state.plan_contacts_phase(leg) > 0) {
            kp = _kp_joint_stance;
            kd = _kd_joint_stance;

        } else {
            kp = _kp_joint_swing;
            kd = _kd_joint_swing;
        }

        #endif

        if (state.plan_contacts_phase(leg) > 0.0) {
            torques.segment<3>(3 * leg) = 
                kp.cwiseProduct(state.wbc_q_cmd.segment<3>(3 * leg) - state.q.segment<3>(3 * leg))
                + kd.cwiseProduct(state.wbc_qdot_cmd.segment<3>(3 * leg) - state.qdot.segment<3>(3 * leg))
                + state.wbc_tau_ff.segment<3>(3 * leg);
        }
        else {
           torques.segment<3>(3 * leg) = 
                kp.cwiseProduct(state.wbc_q_cmd.segment<3>(3 * leg) - state.q.segment<3>(3 * leg))
                + kd.cwiseProduct(state.wbc_qdot_cmd.segment<3>(3 * leg) - state.qdot.segment<3>(3 * leg))
                + state.wbc_tau_ff.segment<3>(3 * leg);
                ;
        }
    }
    
    for (int i = 0; i < 6; i++)
    {
        torques(i) = (torques(i)>500)?(500):(torques(i)<-500?-500:torques(i));
    }

    state.tau = torques;

    return torques;
}