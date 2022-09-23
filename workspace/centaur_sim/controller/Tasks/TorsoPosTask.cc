/*
 * @Author: haoyun 
 * @Date: 2022-09-17 16:49:20
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-09-23 10:11:51
 * @FilePath: /drake/workspace/centaur_sim/controller/Tasks/TorsoPosTask.cc
 * @Description:  To achieve torso's desired position in the world frame:
 *                task Jacobian: Jt
 *                desired states: pos, vel, acc
 *                
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#include "drake/workspace/centaur_sim/controller/Tasks/TorsoPosTask.hpp"


template <typename T>
TorsoPosTask<T>::TorsoPosTask()
    : Task<T>(3, 12) {
    
}

template <typename T>
TorsoPosTask<T>::~TorsoPosTask() {}
// 👿

template <typename T>
void TorsoPosTask<T>::_UpdateTaskJacobian() {

}

template <typename T>
void TorsoPosTask<T>::_UpdateCommand(const Eigen::Matrix<T, -1, 1>& pos_des,
                                const Eigen::Matrix<T, -1, 1>& vel_des,
                                const Eigen::Matrix<T, -1, 1>& acc_des) {

    Task<T>::delta_x_des_ = pos_des;
    Task<T>::xdot_des_ = vel_des;
    Task<T>::xddot_dest_ = acc_des;
}
