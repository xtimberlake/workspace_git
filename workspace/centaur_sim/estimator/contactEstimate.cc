/*
 * @Author: haoyun 
 * @Date: 2022-11-07 15:32:23
 * @LastEditors: haoyun 
 * @LastEditTime: 2023-04-13 16:42:22
 * @FilePath: /drake/workspace/centaur_sim/estimator/contactEstimate.cc
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#include "drake/workspace/centaur_sim/estimator/contactEstimate.h"


template <typename T>
contactEstimate<T>::contactEstimate() {
    
    _s_contact = new bool[2]; // two legs

    _s_contact[0] = false;
    _s_contact[1] = false;

    _threshold = 0.95;
    _prob_contact.setZero();
    _prob_contact_plan.setZero();
    _prob_contact_pos.setZero();
    _prob_contact_force.setZero();
    _prob_contact_velocity.setZero();

    Eigen::Matrix<T, 2, 2> I2; I2 = DMat<T>::Identity(2, 2);
    Eigen::Matrix<T, 2, 2> A; A.setZero();
    Eigen::Matrix<T, 2, 2> B; B = I2;
    Eigen::Matrix<T, 4, 2> H; 
    H.block(0, 0, 2, 2) = I2; 
    H.block(2, 0, 2, 2) = I2;
    Eigen::Matrix<T, 2, 2> Q; Q.diagonal() << 0.1, 0.1;
    Eigen::Matrix<T, 4, 4> R; R.diagonal() << 0.3, 0.3, 0.1, 0.1;
    
    kalmanFliter = new linearKalmanFilter<T>(A, B, H, Q, R);

    _dt = 0.001;
    _lambda = 200; // cutoff frequency
    _gamma = std::exp(-_lambda*_dt);
    _beta = (1 - _gamma) / (_gamma * _dt);


    Vec2<double> num_b, den_a;
    num_b << 1.0-_gamma, 0.0;
    den_a << 1.0, -_gamma;

    for (int i = 6; i < 12; i++) {
        firstOrderFilter[i] = new ButterworthFilter<double>(1, num_b, den_a);
    }

    Eigen::Matrix<T, 3, 3> eye3; eye3.setIdentity();
    Sl_1.setZero();
    Sl_1.block(0, 6, 3, 3) = eye3;

    Sl_2.setZero();    
    Sl_2.block(0, 9, 3, 3) = eye3;

    _foot_force_hat.setZero();
    this->MassqMtx = Eigen::Matrix<double, 12, 12>::Zero();

    _locked_foot_pos.setZero();

    start_gait_plan_scheduler = true;
    this->_RESET_GAIT_COUNTER = 0;
    this->_time_start_to_extend[0] = 0.0;
    this->_time_start_to_extend[1] = 0.0;

    this->_restance_k.setZero();
    this->footAcc.setZero();

    
    T _acc_lambda = 100; // cutoff frequency in s-domain
    T _acc_gamma = std::exp(-_acc_lambda*_dt); // z-domain
    
    Vec2<double> acc_num_b, acc_den_a;
    acc_num_b << 1.0-_acc_gamma, 0.0;
    acc_den_a << 1.0, -_acc_gamma;

    for (int i = 0; i < 2; i++) {
        firstOrderFilter[i] = new ButterworthFilter<double>(1, acc_num_b, acc_den_a);
    }
    
}

template <typename T>
contactEstimate<T>::~contactEstimate() {}


template <typename T>
void contactEstimate<T>::updateMeasurement(const CentaurStates states) {

    this->_system_time = states.t;
    this->phiContact = states.plan_contacts_phase;
    this->phiSwing = states.plan_swings_phase;
    this->pFoot = states.foot_pos_world;
    this->forceFoot = states.foot_force_world;
    this->footVelocity = states.foot_vel_world;
    this->footAcc = states.foot_acc_world;
    this->MassqMtx_last = this->MassqMtx;
    this->MassqMtx = states.Mq;
    this->coriolisVec = states.Cv;
    this->generalizedQdotVec = states.generalizedQdot;
    this->tau_g = states.tau_g;
    this->J_1 = states.J_1;
    this->J_2 = states.J_2;

    this->tau_actuated = Eigen::Matrix<T, 12, 1>::Zero();
    tau_actuated.segment(6, 6) = states.tau;

    disturbance_term.setZero();

    this->_locked_foot_pos = this->pFoot;
    // std::cout << "generaliezed_qdot = " << this->generalizedQdotVec.transpose() << std::endl;
    // std::cout << "---" << std::endl;
    this->max_extend_time = 0.002;
    
}

template <typename T>
void contactEstimate<T>::updateEstimate() {

    Eigen::Matrix<T, 2, 1> u_prob;    // instant input
    T miu_plan[2] = {0.0, 1.0};
    T sigma2_plan[2] = {0.1, 0.25};
    
    Eigen::Matrix<T, 2, 1> z1_prob;   // meas pFoot
    T miu_pz = 0.12;
    T sigma2_pz = 0.05;

    Eigen::Matrix<T, 2, 1> z2_prob;   // meas fGRF
    T miu_f = 120.0;
    T sigma2_f = 10.0;

    Eigen::Matrix<T, 2, 1> z3_prob;   // velocity
    T miu_v = 0.5;
    T sigma2_v = 0.5;
    

    for (int leg = 0; leg < 2; leg++) {

        // prior #1: plan phases
        if(phiContact[leg] > 0)  {
            u_prob[leg] = 0.5 * (std::erf( (phiContact[leg]-miu_plan[0]) / (std::sqrt(2)*sigma2_plan[0]) )
                               + std::erf( (miu_plan[1]-phiContact[leg]) / (std::sqrt(2)*sigma2_plan[1]) ));
        }
        else {
            u_prob[leg] = 0.5 * (2 + std::erf( (miu_plan[0]-phiSwing[leg]) / (std::sqrt(2)*sigma2_plan[0]) )
                                   + std::erf( (phiSwing[leg]-miu_plan[1]) / (std::sqrt(2)*sigma2_plan[1]) ));
        }
        
        // prior #2: foot position
        T pFoot_z = pFoot(2, leg);
        z1_prob[leg] = 0.5 * (1 - std::erf( (pFoot_z-miu_pz)/(std::sqrt(2)*sigma2_pz) ));

        // prior #3: vertical foot forces
        T fGrf_z = -forceFoot(2, leg);
        z2_prob[leg] = 0.5 * (1 + std::erf( (fGrf_z-miu_f)/(std::sqrt(2)*sigma2_f) ));

        // prior $4: velocity
        T v_z = fabs(footVelocity(2, leg));
        z3_prob[leg] = 0.5 * (1 - std::erf( (v_z-miu_v)/(std::sqrt(2)*sigma2_v) ));
    }
    
    Eigen::Matrix<T, 4, 1> zk; zk.setZero();
    zk.head(2) = z1_prob;
    zk.tail(2) = z2_prob;
    

    this->_prob_contact_plan = u_prob;
    this->_prob_contact_pos = z1_prob;
    this->_prob_contact_force = z2_prob;
    this->_prob_contact_velocity = z3_prob;

    kalmanFliter->updateInputMeasure(u_prob, zk);

    this->_prob_contact = kalmanFliter->getXhat();

    // Generalized-Momentum-based disturbance detection
    Eigen::Matrix<T, 12, 1> generalizedMomentum, dynamics_effect, filteded_dynamics_effect;
    generalizedMomentum.setZero(); dynamics_effect.setZero(); filteded_dynamics_effect.setZero();
    Eigen::Matrix<T, 12, 12> C_mtx; C_mtx.setZero();
    DMat<T> qdot, qdot_pinv;
    qdot = this->generalizedQdotVec;
    pseudoInverse(qdot, 0.0001, qdot_pinv);
    C_mtx = this->coriolisVec * qdot_pinv;
    
    generalizedMomentum = this->MassqMtx * generalizedQdotVec;
    dynamics_effect = _beta*generalizedMomentum /*+ this->tau_actuated */+ C_mtx.transpose()*this->generalizedQdotVec /*- tau_g*/;
    for (int i = 6; i < 12; i++) {
        filteded_dynamics_effect[i] =  firstOrderFilter[i]->feedData(dynamics_effect[i]);
    }
    this->disturbance_term = _beta*generalizedMomentum - filteded_dynamics_effect;
    
    Eigen::Matrix<T, 3, 1> f1_hat, f2_hat;
    // DMat<T> Sl_Jt, Sl_Jt_pinv;
    // Sl_Jt = this->Sl_1 * this->J_1;
    // pseudoInverse(Sl_Jt, 0.001, Sl_Jt_pinv);
    // f1_hat = Sl_Jt_pinv * Sl_1 * this->disturbance_term;
    f1_hat = this->J_1.block(0, 6, 3, 3).transpose().inverse() * this->disturbance_term.segment(6, 3);
    
    // DMat<T> Sl_Jt2, Sl_Jt_pinv2;
    // Sl_Jt2 = this->Sl_2 * this->J_2;
    // pseudoInverse(Sl_Jt2, 0.001, Sl_Jt_pinv2);
    // f2_hat = Sl_Jt_pinv2 * Sl_2 * this->disturbance_term;
    f2_hat = this->J_2.block(0, 9, 3, 3).transpose().inverse() * this->disturbance_term.segment(9, 3);


    this->_foot_force_hat.col(0) = f1_hat;
    this->_foot_force_hat.col(1) = f2_hat;

    

}

template <typename T>
void contactEstimate<T>::eventsDetect() {

    for (size_t leg = 0; leg < 2; leg++) { // for two legs
        if (this->phiContact[leg] > 0) { // only detect the unexpected events for swing leg
            _foot_contact_event[leg] = ContactEvent::STANCE;

        } else { // detect the unexpected events for swing leg
            if(this->phiSwing[leg] < 0.5) _foot_contact_event[leg] = ContactEvent::SWING;
            switch (this->_foot_contact_event[leg])
            {
                case ContactEvent::SWING:
                {

                    _RESET_GAIT_COUNTER = 0;

                    bool start_detect = false;
                    bool collision_detect = false;

                    if (this->phiSwing[leg] > 0.5) start_detect = true; // detect the evnets at second half of the swing phase
                    // if(this->_foot_force_hat(2, leg) > 80.0)  collision_detect = true; // threshold the vertical impulse
                    if(this->footAcc(2, leg) > 40.0)  collision_detect = true; // threshold the vertical acceleration
                    if (start_detect && collision_detect)
                    {

                        this->_locked_foot_pos = this->pFoot; // store the collision pos
                        this->_foot_contact_event[leg] = ContactEvent::EARLY_CONTACT;
                        std::cout << "leg " << leg << ": " << "Early Contact at phase = " << this->phiSwing[leg] << "." << std::endl;
                    }

                    if (start_detect && !collision_detect && this->phiSwing[leg] > 0.996)
                    {
                        this->_locked_foot_pos = this->pFoot; // store the collision pos
                        this->_foot_contact_event[leg] = ContactEvent::LATE_CONTACT;
                        this->_time_start_to_extend[leg] = this->_system_time;
                        std::cout << "leg " << leg << ": " << "Late Contact at phase = " << this->phiSwing[leg] << "." << std::endl;
                        _restance_k[leg]++;

                        // cancell the downward motion
                        this->_foot_contact_event[leg] = ContactEvent::SWING;
                    }
                    

                    break;
                }
                case ContactEvent::EARLY_CONTACT:
                {
                    // stops swinging down; stays at the current position;
                    // decreases the impedance coefficients; wait for the plan stance.
                    _RESET_GAIT_COUNTER = 0;
                    break;
                }
                case ContactEvent::LATE_CONTACT:
                {
                    // swings down to the ground util collision is dectected;
                    // meanwhile freezes the gait scheduler!
                    start_gait_plan_scheduler = false;
                    _RESET_GAIT_COUNTER = 0;
                    double time_pass;
                    bool collision_detect = false;
                    bool extend_time_out = false;
                    time_pass = this->_system_time - this->_time_start_to_extend[leg];
                
                    // if(this->_foot_force_hat(2, leg) > 50.0)  collision_detect = true; // threshold the vertical impulse
                    if(this->footAcc(2, leg) > 40.0)  collision_detect = true; // threshold the vertical acceleration
                    if(time_pass > this->max_extend_time) extend_time_out = true;
                    _restance_k[leg]++;
                    if(collision_detect || extend_time_out)
                    {
                        // restart and RESET the gait scheduler
                        start_gait_plan_scheduler = true;
                        _RESET_GAIT_COUNTER = leg + 1;
                        this->_foot_contact_event[leg] = ContactEvent::RESTANCE;
                        std::cout << "leg " << leg << " : ReStance.";
                        if(extend_time_out && !collision_detect) {std::cout << "(because of timeout), current impulse force = " << this->_foot_force_hat(2, leg) << ".  ";}
                        if(!extend_time_out && collision_detect) {std::cout << "(because of collision), time elipsed = " << time_pass << ".  ";}
                        std::cout << "Total control steps=" << this->_restance_k << std::endl;
                        _restance_k.setZero();
                        
                    }

                    break;
                }
                case ContactEvent::RESTANCE:
                { 
                    // restart and RESET the gait scheduler
                    // stays at the current position; wait for the plan stance.
                    start_gait_plan_scheduler = true;
                    break;
                }
                case ContactEvent::STANCE:
                {
                    start_gait_plan_scheduler = true;
                    break;
                }         
                default: break;
            }
        


        }
    }

}


template <typename T>
void contactEstimate<T>::publishStates(CentaurStates& states) {

    states.RESET_GAIT_COUNTER = this->_RESET_GAIT_COUNTER;
    states.locked_foot_pos = this->_locked_foot_pos;
    states.foot_contact_event[0] = this->_foot_contact_event[0];
    states.foot_contact_event[1] = this->_foot_contact_event[1];
    // std::cout << "left state = " << static_cast<int>(states.foot_contact_event[0]) << ", right state = " << static_cast<int>(states.foot_contact_event[1]) << std::endl;
    states.restance_k = this->_restance_k;
    
    states.filted_collision_signal[0] = firstOrderFilter[0]->feedData(footAcc(2, 0));
    states.filted_collision_signal[1] = firstOrderFilter[1]->feedData(footAcc(2, 1));
}