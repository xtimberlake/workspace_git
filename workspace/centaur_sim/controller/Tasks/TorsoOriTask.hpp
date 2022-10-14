/*
 * @Author: haoyun 
 * @Date: 2022-09-17 16:49:42
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-10-14 21:33:00
 * @FilePath: /drake/workspace/centaur_sim/controller/Tasks/TorsoOriTask.hpp
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once

#include "drake/workspace/centaur_sim/controller/Task.hpp"
#include "drake/workspace/centaur_sim/dynamics/FloatingBaseModel.h"

template <typename T>
class TorsoOriTask : public Task<T>
{
 public:
    TorsoOriTask(const FloatingBaseModel* robot)
    : Task<T>(3, 12), _robot_sys(robot){

        // qdot = [omega, v, qj_dot]
        Task<T>::Jt_ = DMat<T>::Zero(Task<T>::dim_task_, Task<T>::dim_of_decision_variable_);
        Task<T>::Jt_.block(0, 3, 3, 3).setIdentity();
        Task<T>::JtDotQdot_ = DVec<T>::Zero(Task<T>::dim_task_);
        
        _Kp_kin = DVec<T>::Constant(Task<T>::dim_task_, 0.0);
        _Kp = DVec<T>::Constant(Task<T>::dim_task_, 0.0);
        _Kd = DVec<T>::Constant(Task<T>::dim_task_, 0.0);

    }
    ~TorsoOriTask() {}
        void _UpdateTaskJacobian() {

        Quat<T> quat = _robot_sys->_state.bodyOrientation;
        Mat3<T> Rot = ori::quaternionToRotationMatrix(quat);
        Task<T>::Jt_.block(0, 0, 3, 3) = Rot.transpose();
    }

    void _UpdateCommand(const void* pos_des, 
                        const DVec<T>& vel_des,
                        const DVec<T>& acc_des) {

        // 
        Quat<T> ori_cmd = *(static_cast<const Quat<T>*>(pos_des));
      
        Quat<T> link_ori = _robot_sys->_state.bodyOrientation;

        Quat<T> link_ori_inv;
        link_ori_inv[0] = link_ori[0];
        link_ori_inv[1] = -link_ori[1];
        link_ori_inv[2] = -link_ori[2];
        link_ori_inv[3] = -link_ori[3];

         // Explicit because operational space is in global frame
        Quat<T> ori_err = ori::quatProduct(ori_cmd, link_ori_inv);
        if (ori_err[0] < 0.) {
            ori_err *= (-1.);
        }
        Vec3<T> ori_err_so3;
        ori::quaternionToso3(ori_err, ori_err_so3);


        SVec<T> curr_vel = _robot_sys->_state.bodyVelocity;
        // Configuration space: Local
        // Operational Space: Global
        Mat3<T> Rot = ori::quaternionToRotationMatrix(link_ori);
        Vec3<T> vel_err = Rot.transpose()*(TK::xdot_des_ - curr_vel.head(3)); //Global frame

        // Rx, Ry, Rz
        for (int i(0); i < 3; ++i) {
            TK::delta_x_des_[i] = _Kp_kin[i] * ori_err_so3[i];
            TK::xdot_des_[i] = vel_des[i];
            TK::xddot_dest_[i] = acc_des[i];

            TK::op_cmd_[i] = _Kp[i] * ori_err_so3[i] +
                            _Kd[i] * vel_err[i] + TK::xddot_dest_[i];
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

template class TorsoOriTask<double>;
// template class TorsoOriTask<float>;

