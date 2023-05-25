/*
 * @Author: haoyun 
 * @Date: 2022-11-07 15:32:34
 * @LastEditors: haoyun 
 * @LastEditTime: 2023-05-21 21:34:53
 * @FilePath: /drake/workspace/centaur_sim/estimator/contactEstimate.h
 * @Description: contact estimate that fuses plan_states, foot_pos & grf_z
 * [ref]: 
 * Bledt, Gerardo, Patrick M. Wensing, Sam Ingersoll, and Sangbae Kim. "Contact model
 *  fusion for event-based locomotion in unstructured terrains." In 2018 IEEE International
 *  Conference on Robotics and Automation (ICRA), pp. 4399-4406. IEEE, 2018.
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once
#include <iostream>
#include <drake/workspace/centaur_sim/Utils/cppTypes.h>
#include <drake/workspace/centaur_sim/estimator/linearKalmanFilter.h>
#include <drake/workspace/centaur_sim/controller/CentaurStates.h>
#include <drake/workspace/centaur_sim/Utils/butterworthFilter.h>
#include <drake/workspace/centaur_sim/Utils/pseudoInverse.h>
#include "drake/workspace/centaur_sim/estimator/contactEventData.h"

class CentaurStates;




template <typename T>
class contactEstimate {
public:
 contactEstimate();
 ~contactEstimate();

 bool* getBinaryContactState() const { return _s_contact; }
 void getContactProbabilities(Vec2<T>& prob_contact) const { prob_contact = _prob_contact; }
 void getContactProbabilitiesBasedonPlan(Vec2<T>& prob_contact) const { prob_contact = _prob_contact_plan; }
 void getContactProbabilitiesBasedonPos(Vec2<T>& prob_contact) const { prob_contact = _prob_contact_pos; }
 void getContactProbabilitiesBasedonForce(Vec2<T>& prob_contact) const { prob_contact = _prob_contact_force; }
 void getContactProbabilitiesBasedonVelocity(Vec2<T>& prob_contact) const { prob_contact = _prob_contact_velocity; }
 void getEstimatedContactForce(Eigen::Matrix<double, 3, 2>& foot_forces) const { foot_forces = _foot_force_hat; }
 void setThresholdValue(const T& threshold) { _threshold = threshold; }

 void updateMeasurement(const CentaurStates states);
 void updateEstimate();
 void eventsDetect();
 void publishStates(CentaurStates& states);

 // variables 
// protected:
 
 /* Measurement data */
 // prior
 DVec<T> phiContact;
 DVec<T> phiSwing;
 // kinematics 
 DMat<T> pFoot;

 // differential kinematics
 DMat<T> footVelocity;

 // acc
 Eigen::Matrix<double, 3, 2> footAcc;

 // dynamics 
 DMat<T> MassqMtx;
 DMat<T> MassqMtx_last;
 DVec<T> generalizedQdotVec;
 DVec<T> coriolisVec;
 DVec<T> tau_g;
 DVec<T> tau_actuated;
 DMat<T> forceFoot;
 DMat<T> J_1, J_2;
 DVec<T> disturbance_term;
 Eigen::Matrix<T, 3, 12> Sl_1, Sl_2;


 bool *_s_contact;
 double _system_time;
 T _dt; // sampling time
 T _threshold;
 T _lambda, _gamma, _beta; // cut-off frequency
 Vec2<T> _prob_contact, _prob_contact_plan, _prob_contact_pos, _prob_contact_force;
 Vec2<T> _prob_contact_velocity;
 linearKalmanFilter<T>* kalmanFliter;
 ButterworthFilter<T>* firstOrderFilter[12];
 Eigen::Matrix<T, 3, 2> _foot_force_hat;
 Eigen::Matrix<T, 2, 1> _filted_collision_signal;

 ContactEvent _foot_contact_event[2]; // two legs
 Eigen::Matrix<int, 2, 1> _restance_k;
 Eigen::Matrix<T, 3, 2> _locked_foot_pos;
 double _time_start_to_extend[2];
 bool start_gait_plan_scheduler;
 double max_extend_time;
 
 int _RESET_GAIT_COUNTER; // 0: void, 1:left stance, 2:left swing



};

template class contactEstimate<double>;