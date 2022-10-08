/*
 * @Author: haoyun 
 * @Date: 2022-09-17 14:48:02
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-10-08 21:57:44
 * @FilePath: /drake/workspace/centaur_sim/controller/Task.hpp
 * @Description: A task is defined by a task Jacobian and
 *               the desired target states as function depen.
 *               That is, Task:= {Jt, delta_x_des, xdot_des, xddot_des}
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once
#include <eigen3/Eigen/Dense>

template <typename T>
class Task
{
public:
     Task(size_t dim_of_task, size_t dim_of_decision_variable):
     dim_task_(dim_of_task) {
        // resize the task Jacobian and desired states with zero matrix/vector
        Jt_ = Eigen::Matrix<T, -1, -1>::Zero(dim_of_task, dim_of_decision_variable);
        delta_x_des_ = Eigen::Matrix<T, -1, 1>::Zero(dim_of_task);
        xdot_des_ = Eigen::Matrix<T, -1, 1>::Zero(dim_of_task);
        xddot_dest_ = Eigen::Matrix<T, -1, 1>::Zero(dim_of_task);
     }
    virtual ~ Task() {;}


protected:

    virtual void _UpdateTaskJacobian() = 0;

    virtual void _UpdateCommand(const Eigen::Matrix<T, -1, 1>& pos_des,
                                const Eigen::Matrix<T, -1, 1>& vel_des,
                                const Eigen::Matrix<T, -1, 1>& acc_des) = 0;


    size_t dim_task_;
    Eigen::Matrix<T, -1, -1> Jt_;           // task Jacobian matrix
    Eigen::Matrix<T, -1, 1> delta_x_des_;
    Eigen::Matrix<T, -1, 1> xdot_des_;
    Eigen::Matrix<T, -1, 1> xddot_dest_;

    
};



