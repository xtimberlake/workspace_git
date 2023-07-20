/*
 * @Author: haoyun 
 * @Date: 2022-10-10 17:19:10
 * @LastEditors: haoyun 
 * @LastEditTime: 2023-06-19 21:45:49
 * @FilePath: /drake/workspace/centaur_sim/controller/Tasks/LinkPosTask.hpp
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once
#include "drake/workspace/centaur_sim/controller/Task.hpp"

class FloatingBaseModel;

template <typename T>
class LinkPosTask : public Task<T> {
 public:
  LinkPosTask(const FloatingBaseModel* robot, int link_idx,
              bool virtual_depend = true) 
    :Task<T>(3, 12), robot_sys_(robot), link_idx_(link_idx), virtual_depend_(virtual_depend)
    {
        Task<T>::Jt_ = DMat<T>::Zero(Task<T>::dim_task_, Task<T>::dim_of_decision_variable_);
        Task<T>::Jt_.block(0, 3, 3, 3).setIdentity();
        Task<T>::JtDotQdot_ = DVec<T>::Zero(Task<T>::dim_task_);
        
        _Kp_kin = DVec<T>::Zero(Task<T>::dim_task_);
        _Kp = DVec<T>::Zero(Task<T>::dim_task_);
        _Kd = DVec<T>::Zero(Task<T>::dim_task_);

        _Kp_kin << 10.0, 5.0, 10.0;
        _Kp << 2.4, 0.8, 2.0;
        _Kd << 1.8, 0.8, 2.3;



    }
  ~LinkPosTask() {}

  DVec<T> _Kp, _Kd, _Kp_kin;

//  protected:
  // Update op_cmd_
  void _UpdateCommand(const void* pos_des, 
                        const DVec<T>& vel_des,
                        const DVec<T>& acc_des)  {
        Vec3<T> pos_cmd = *(static_cast<const Vec3<T>*>(pos_des));
        Vec3<T> link_pos;
        link_pos = robot_sys_->_pGC[link_idx_];

        // X, Y, Z
        for (int i(0); i < 3; ++i) {
            TK::delta_x_des_[i] = _Kp_kin[i]* (pos_cmd[i] - link_pos[i]);
            TK::xdot_des_[i] = vel_des[i];
            TK::xddot_dest_[i] = acc_des[i];
        }

        // std::cout << "which link = " << link_idx_  << "       ";
        // std::cout << "xddot_dest_ = (" << TK::xddot_dest_.rows() << "," << TK::xddot_dest_.cols() << ") = " << std::endl;
        // std::cout << TK::xddot_dest_.transpose() << std::endl;

        // Op acceleration command
        for (size_t i(0); i < TK::dim_task_; ++i) {
            
            TK::op_cmd_[i] =
                        _Kp[i] * TK::delta_x_des_[i] +
                        _Kd[i] * (TK::xdot_des_[i] - robot_sys_->_vGC[link_idx_][i]) +
                        TK::xddot_dest_[i];
        }
    }
  // Update Jt_
  void _UpdateTaskJacobian() {

    TK::Jt_ = robot_sys_->_Jc[link_idx_];
    if (!virtual_depend_) {
        TK::Jt_.block(0, 0, 3, 6) = DMat<T>::Zero(3, 6);
    }

  }
  // Update JtDotQdot_
  bool _UpdateTaskJDotQdot() {
    TK::JtDotQdot_ = robot_sys_->_Jcdqd[link_idx_];
    return true;
  }
  bool _AdditionalUpdate() { return true; }

  const FloatingBaseModel* robot_sys_;
  int link_idx_; // the contact index
  bool virtual_depend_;
};

template class LinkPosTask<double>;
// template class LinkPosTask<float>;
