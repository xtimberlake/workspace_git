/*
 * @Author: haoyun 
 * @Date: 2022-11-07 15:32:34
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-11-16 21:21:50
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
 void setThresholdValue(const T& threshold) { _threshold = threshold; }


 void updateEstimate(const DVec<T>& phiContact, const DVec<T> phiSwing,
                     const DMat<T>& pFoot, const DMat<T>& forceFoot, const DMat<T>& footVelocity);

 // variables 
protected:
 bool *_s_contact;
 T _threshold;
 Vec2<T> _prob_contact, _prob_contact_plan, _prob_contact_pos, _prob_contact_force;
 Vec2<T> _prob_contact_velocity;
 linearKalmanFilter<T>* kalmanFliter;
};

template class contactEstimate<double>;