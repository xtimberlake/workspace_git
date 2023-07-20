/*
 * @Author: haoyun 
 * @Date: 2022-07-22 08:44:58
 * @LastEditors: haoyun 
 * @LastEditTime: 2023-06-13 18:53:43
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


    this->_kp_pure_joint_stance << 52.0, 25.0, 25.0;
    this->_kd_pure_joint_stance << 0.0, 0.0, 0.0;
    this->_kp_pure_joint_swing << 200.0, 300.0, 300.0;
    this->_kd_pure_joint_swing << 10.0, 10.0, 10.0;



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


// Geometry method to solve the inverse kinematics
Eigen::Matrix<double, 12, 1> LegController::inverse_kinematics(CentaurStates& state){ 
    
    Eigen::Matrix<double, 12, 1> q_and_qdot_ref; q_and_qdot_ref.setZero();
    // Part #1 compute the reference phi angles:
    Eigen::Matrix<double, 3, 2> phi_ref; phi_ref.setZero();
    int leg = 0;
    double d1, d2;

    
    // left leg
    Eigen::Vector3d pos_hipref; pos_hipref.setZero();
    pos_hipref << state.foot_pos_cmd_rel(2, leg) - state.robot_params_const.left_abad_location[2],
                state.foot_pos_cmd_rel(1, leg) - state.robot_params_const.left_abad_location[1],
                -(state.foot_pos_cmd_rel(0, leg) - state.robot_params_const.left_abad_location[0]);
                    
    pos_hipref(0) = (pos_hipref(0)<-0.97)?(-0.97):((pos_hipref(0)>-0.8)?(-0.8):(pos_hipref(0)));

    d1=sqrt(pow(pos_hipref[0],2)+pow(pos_hipref[1],2)-pow(state.robot_params_const.leg_length[0],2));
    phi_ref(0, leg) = (pos_hipref[1] >= 0)?
                    (M_PI_2 - atan(abs(pos_hipref[0])/abs(pos_hipref[1]))-atan(state.robot_params_const.leg_length[0]/d1))
                    :(-M_PI_2 + atan(abs(pos_hipref[0])/abs(pos_hipref[1]))+atan(state.robot_params_const.leg_length[0]/d1));

    
    d2=sqrt(pow(d1,2)+pow((pos_hipref[2]-state.robot_params_const.leg_length[1]),2));
    phi_ref(1, leg)=atan((pos_hipref[2]-state.robot_params_const.leg_length[1])/d1)
                    +acos((pow(state.robot_params_const.leg_length[2],2)+pow(d2,2)-pow(state.robot_params_const.leg_length[3],2))/(2*state.robot_params_const.leg_length[2]*d2));
    
    phi_ref(2, leg)=acos((pow(state.robot_params_const.leg_length[2],2)+pow(state.robot_params_const.leg_length[3],2)-pow(d2,2))/(2*state.robot_params_const.leg_length[2]*state.robot_params_const.leg_length[3]));
    // printf("hip_ref = [%.2f, %.2f, %.2f];  d1=%.2f; d2=%.2f; phi=[%.2f, %.2f, %.2f] \n\r", pos_hipref(0), pos_hipref(1), pos_hipref(2), d1, d2, 
    //         phi_ref(0, 0), phi_ref(1, 0), phi_ref(2, 0));
    // right leg
    leg = 1;

    pos_hipref << state.foot_pos_cmd_rel(2, leg) - state.robot_params_const.left_abad_location[2],
                state.foot_pos_cmd_rel(1, leg) + state.robot_params_const.left_abad_location[1],  /* the difference between left and right leg*/
                -(state.foot_pos_cmd_rel(0, leg) - state.robot_params_const.left_abad_location[0]);

    pos_hipref(0) = (pos_hipref(0)<-0.97)?(-0.97):((pos_hipref(0)>-0.8)?(-0.8):(pos_hipref(0)));
    // std::cout << "pos_hipref = " << state.foot_pos_cmd_rel.block<3, 1>(0, 0).transpose() << std::endl;

    d1=sqrt(pow(pos_hipref[0],2)+pow(pos_hipref[1],2)-pow(state.robot_params_const.leg_length[0],2));
    phi_ref(0, leg) = (pos_hipref[1] <= 0)? /* the difference between left and right leg*/
                    (M_PI_2 - atan(abs(pos_hipref[0])/abs(pos_hipref[1]))-atan(state.robot_params_const.leg_length[0]/d1))
                    :(-M_PI_2 + atan(abs(pos_hipref[0])/abs(pos_hipref[1]))+atan(state.robot_params_const.leg_length[0]/d1));
    d2=sqrt(pow(d1,2)+pow((pos_hipref[2]-state.robot_params_const.leg_length[1]),2));
    phi_ref(1, leg)=atan((pos_hipref[2]-state.robot_params_const.leg_length[1])/d1)
                    +acos((pow(state.robot_params_const.leg_length[2],2)+pow(d2,2)-pow(state.robot_params_const.leg_length[3],2))/(2*state.robot_params_const.leg_length[2]*d2));
    phi_ref(2, leg)=acos((pow(state.robot_params_const.leg_length[2],2)+pow(state.robot_params_const.leg_length[3],2)-pow(d2,2))/(2*state.robot_params_const.leg_length[2]*state.robot_params_const.leg_length[3]));



    // convert phi angles to theta angle in simulation
    Eigen::Matrix<double, 6, 1> q_ref; q_ref.setZero();
    q_ref[0] = phi_ref(0, 0);
    q_ref[1] = phi_ref(1, 0);
    q_ref[2] = -M_PI + phi_ref(2, 0);

    q_ref[3] = -phi_ref(0, 1);
    q_ref[4] = phi_ref(1, 1);
    q_ref[5] = -M_PI + phi_ref(2, 1);

    // std::cout << "q_ref = " << q_ref.transpose() << std::endl;
    
    // Differential Inverse Kinematics:
    Eigen::Matrix<double, 6, 1> qdot_ref; qdot_ref.setZero();
    qdot_ref.segment<3>(0) = state.JacobianFoot[0].inverse() * state.foot_vel_cmd_rel.block<3, 1>(0, 0);
    qdot_ref.segment<3>(3) = state.JacobianFoot[1].inverse() * state.foot_vel_cmd_rel.block<3, 1>(0, 1);

    q_and_qdot_ref.head(6) = q_ref;
    q_and_qdot_ref.tail(6) = qdot_ref;

    // std::cout << "qdot_ref = " << qdot_ref.transpose() << std::endl;

    state.ik_q_and_qdot_cmd = q_and_qdot_ref;

    return q_and_qdot_ref;

}

Eigen::Matrix<double, 6, 1> LegController::wbc_feedforward_ik_control(CentaurStates& state) { 

    Eigen::Matrix<double, 6, 1> torques; torques.setZero();

        Eigen::Matrix<double, 12, 1> desired_states; desired_states.setZero();
        Eigen::Matrix<double, 6, 1> q_cmd; q_cmd.setZero();
        Eigen::Matrix<double, 6, 1> qdot_cmd; qdot_cmd.setZero();

        desired_states = inverse_kinematics(state);
        q_cmd = desired_states.head(6);
        qdot_cmd = desired_states.tail(6);

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
            kp = _kp_pure_joint_swing;
            kd = _kd_pure_joint_swing;
            // std::cout << kp(0) << std::endl;
            break;
        }
        case ContactEvent::EARLY_CONTACT:
        case ContactEvent::STANCE:
        case ContactEvent::RESTANCE:
        default:
        {
            kp = _kp_pure_joint_stance;
            kd = _kd_pure_joint_stance;
            break;
        }
        }
        #else
        if(state.plan_contacts_phase(leg) > 0) {
            kp = _kp_pure_joint_stance;
            kd = _kd_pure_joint_stance;

        } else {
            kp = _kp_pure_joint_swing;
            kd = _kd_pure_joint_swing;
        }

        #endif


        // qdot_cmd.setZero();

        if (state.plan_contacts_phase(leg) > 0.0) {
            torques.segment<3>(3 * leg) = 
                kp.cwiseProduct(q_cmd.segment<3>(3 * leg) - state.q.segment<3>(3 * leg))
                + kd.cwiseProduct(qdot_cmd.segment<3>(3 * leg) - state.qdot.segment<3>(3 * leg))
                + state.wbc_tau_ff.segment<3>(3 * leg);
        }
        else {
           torques.segment<3>(3 * leg) = 
                kp.cwiseProduct(q_cmd.segment<3>(3 * leg) - state.q.segment<3>(3 * leg))
                + kd.cwiseProduct(qdot_cmd.segment<3>(3 * leg) - state.qdot.segment<3>(3 * leg))
                // + state.wbc_tau_ff.segment<3>(3 * leg);
                ;
        }
    }

    return torques;
    
    
}

void LegController::debug_ik(CentaurStates& state) {

    state.debug_q_now.head(6) = state.q;
    state.debug_q_now.tail(6) = state.qdot;
    Eigen::Matrix<double, 3, 2> phi_ref; phi_ref.setZero();
    int leg = 0;
    double d1, d2;

    
    // left leg
    Eigen::Vector3d pos_hipref; pos_hipref.setZero();
    pos_hipref << state.foot_pos_rel(2, leg) - state.robot_params_const.left_abad_location[2],
                state.foot_pos_rel(1, leg) - state.robot_params_const.left_abad_location[1],
                -(state.foot_pos_rel(0, leg) - state.robot_params_const.left_abad_location[0]);
                    
    pos_hipref(0) = (pos_hipref(0)<-0.97)?(-0.97):((pos_hipref(0)>-0.7)?(-0.7):(pos_hipref(0)));

    d1=sqrt(pow(pos_hipref[0],2)+pow(pos_hipref[1],2)-pow(state.robot_params_const.leg_length[0],2));
    phi_ref(0, leg) = (pos_hipref[1] >= 0)?
                    (M_PI_2 - atan(abs(pos_hipref[0])/abs(pos_hipref[1]))-atan(state.robot_params_const.leg_length[0]/d1))
                    :(-M_PI_2 + atan(abs(pos_hipref[0])/abs(pos_hipref[1]))+atan(state.robot_params_const.leg_length[0]/d1));

    
    d2=sqrt(pow(d1,2)+pow((pos_hipref[2]-state.robot_params_const.leg_length[1]),2));
    phi_ref(1, leg)=atan((pos_hipref[2]-state.robot_params_const.leg_length[1])/d1)
                    +acos((pow(state.robot_params_const.leg_length[2],2)+pow(d2,2)-pow(state.robot_params_const.leg_length[3],2))/(2*state.robot_params_const.leg_length[2]*d2));
    
    phi_ref(2, leg)=acos((pow(state.robot_params_const.leg_length[2],2)+pow(state.robot_params_const.leg_length[3],2)-pow(d2,2))/(2*state.robot_params_const.leg_length[2]*state.robot_params_const.leg_length[3]));
    // printf("hip_ref = [%.2f, %.2f, %.2f];  d1=%.2f; d2=%.2f; phi=[%.2f, %.2f, %.2f] \n\r", pos_hipref(0), pos_hipref(1), pos_hipref(2), d1, d2, 
    //         phi_ref(0, 0), phi_ref(1, 0), phi_ref(2, 0));

    // right leg
    leg = 1;
    pos_hipref << state.foot_pos_rel(2, leg) - state.robot_params_const.left_abad_location[2],
                state.foot_pos_rel(1, leg) + state.robot_params_const.left_abad_location[1],  /* the difference between left and right leg*/
                -(state.foot_pos_rel(0, leg) - state.robot_params_const.left_abad_location[0]);

    pos_hipref(0) = (pos_hipref(0)<-0.97)?(-0.97):((pos_hipref(0)>-0.7)?(-0.7):(pos_hipref(0)));
    // std::cout << "pos_hipref = " << state.foot_pos_cmd_rel.block<3, 1>(0, 0).transpose() << std::endl;

    d1=sqrt(pow(pos_hipref[0],2)+pow(pos_hipref[1],2)-pow(state.robot_params_const.leg_length[0],2));
    phi_ref(0, leg) = (pos_hipref[1] <= 0)? /* the difference between left and right leg*/
                    (M_PI_2 - atan(abs(pos_hipref[0])/abs(pos_hipref[1]))-atan(state.robot_params_const.leg_length[0]/d1))
                    :(-M_PI_2 + atan(abs(pos_hipref[0])/abs(pos_hipref[1]))+atan(state.robot_params_const.leg_length[0]/d1));
    d2=sqrt(pow(d1,2)+pow((pos_hipref[2]-state.robot_params_const.leg_length[1]),2));
    phi_ref(1, leg)=atan((pos_hipref[2]-state.robot_params_const.leg_length[1])/d1)
                    +acos((pow(state.robot_params_const.leg_length[2],2)+pow(d2,2)-pow(state.robot_params_const.leg_length[3],2))/(2*state.robot_params_const.leg_length[2]*d2));
    phi_ref(2, leg)=acos((pow(state.robot_params_const.leg_length[2],2)+pow(state.robot_params_const.leg_length[3],2)-pow(d2,2))/(2*state.robot_params_const.leg_length[2]*state.robot_params_const.leg_length[3]));

    // convert phi angles to theta angle in simulation
    Eigen::Matrix<double, 6, 1> q_ref; q_ref.setZero();
    q_ref[0] = phi_ref(0, 0);
    q_ref[1] = phi_ref(1, 0);
    q_ref[2] = -M_PI + phi_ref(2, 0);

    q_ref[3] = -phi_ref(0, 1);
    q_ref[4] = phi_ref(1, 1);
    q_ref[5] = -M_PI + phi_ref(2, 1);

    // Differential Inverse Kinematics:
    Eigen::Matrix<double, 6, 1> qdot_ref; qdot_ref.setZero();
    qdot_ref.segment<3>(0) = state.JacobianFoot[0].inverse() * state.foot_vel_rel.block<3, 1>(0, 0);
    qdot_ref.segment<3>(3) = state.JacobianFoot[1].inverse() * state.foot_vel_rel.block<3, 1>(0, 1);

    state.debug_q_ik_result.head(6) = q_ref;
    state.debug_q_ik_result.tail(6) = qdot_ref;

}