/*
 * @Author: haoyun 
 * @Date: 2022-09-17 16:49:42
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-10-14 21:05:47
 * @FilePath: /drake/workspace/centaur_sim/controller/Tasks/TorsoPosTask.hpp
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once

#include "drake/workspace/centaur_sim/controller/Task.hpp"
#include "drake/workspace/centaur_sim/dynamics/FloatingBaseModel.h"


template <typename T>
class TorsoPosTask : public Task<T> 
{
public:
    TorsoPosTask(const FloatingBaseModel* robot)
    : Task<T>(3, 12), _robot_sys(robot){

        // qdot = [omega, v, qj_dot]
        Task<T>::Jt_ = DMat<T>::Zero(Task<T>::dim_task_, Task<T>::dim_of_decision_variable_);
        Task<T>::Jt_.block(0, 3, 3, 3).setIdentity();
        Task<T>::JtDotQdot_ = DVec<T>::Zero(Task<T>::dim_task_);
        
        _Kp_kin = DVec<T>::Constant(Task<T>::dim_task_, .0);
        _Kp = DVec<T>::Constant(Task<T>::dim_task_, .0);
        _Kd = DVec<T>::Constant(Task<T>::dim_task_, .0);

    }
    ~TorsoPosTask() {}

    void _UpdateTaskJacobian() {

        Quat<T> quat = _robot_sys->_state.bodyOrientation;
        Mat3<T> Rot = ori::quaternionToRotationMatrix(quat);
        Task<T>::Jt_.block(0, 3, 3, 3) = Rot.transpose();
    }

    void _UpdateCommand(const void* pos_des, 
                        const DVec<T>& vel_des,
                        const DVec<T>& acc_des) {

        // 
        Vec3<T> pos_cmd = *(static_cast<const Vec3<T>*>(pos_des));
        Vec3<T> link_pos = _robot_sys->_state.bodyPosition;


        // torso velocity expressed in the absolute coodinates
        Quat<T> quat = _robot_sys->_state.bodyOrientation;
        Mat3<T> Rot = ori::quaternionToRotationMatrix(quat);
        SVec<T> curr_vel = _robot_sys->_state.bodyVelocity;
        curr_vel.tail(3) = Rot.transpose() * curr_vel.tail(3);

        for (int i = 0; i < 3; i++) {
            Task<T>::delta_x_des_[i] = _Kp_kin[i] * (pos_cmd[i] - link_pos[i]);
            Task<T>::xdot_des_[i] = vel_des[i];
            Task<T>::xddot_dest_[i] = acc_des[i];

            Task<T>::op_cmd_[i] = _Kp[i] * (pos_cmd[i] - link_pos[i]) +
                     _Kd[i] * (Task<T>::xdot_des_[i] - curr_vel[i + 3]) +
                     Task<T>::xddot_dest_[i];
        }

    }

   bool _UpdateTaskJDotQdot() {
        return true;
   }
   bool _AdditionalUpdate() { return true; }

    const FloatingBaseModel* _robot_sys;
    DVec<T> _Kp_kin;
    DVec<T> _Kp, _Kd;
};
