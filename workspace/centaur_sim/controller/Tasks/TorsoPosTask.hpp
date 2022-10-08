/*
 * @Author: haoyun 
 * @Date: 2022-09-17 16:49:42
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-10-08 21:58:30
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

    }
    ~TorsoPosTask() {}

    void _UpdateTaskJacobian() {
        
    }

    void _UpdateCommand(const Eigen::Matrix<T, -1, 1>& pos_des,
                                const Eigen::Matrix<T, -1, 1>& vel_des,
                                const Eigen::Matrix<T, -1, 1>& acc_des) {

        Task<T>::delta_x_des_ = pos_des;
        Task<T>::xdot_des_ = vel_des;
        Task<T>::xddot_dest_ = acc_des;

    }
    const FloatingBaseModel* _robot_sys;
};
