/*
 * @Author: haoyun 
 * @Date: 2022-09-17 14:48:02
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-10-13 19:21:35
 * @FilePath: /drake/workspace/centaur_sim/controller/Task.hpp
 * @Description: A task is defined by a task Jacobian and
 *               the desired target states as function dependent on configuration. 
 *               That is, Task:= {Jt, delta_x_des, xdot_des, xddot_des}
 * 
 * 
 */
#pragma once
#include <eigen3/Eigen/Dense>
#include "drake/workspace/centaur_sim/Utils/cppTypes.h"

#define TK Task<T>

template <typename T>
class Task
{
public:
     Task(size_t dim_of_task, size_t dim_of_decision_variable):
     dim_task_(dim_of_task),
     dim_of_decision_variable_(dim_of_decision_variable) {
        // resize the task Jacobian and desired states with zero matrix/vector
        op_cmd_ = Eigen::Matrix<T, -1, 1>::Zero(dim_of_task);
        Jt_ = Eigen::Matrix<T, -1, -1>::Zero(dim_of_task, dim_of_decision_variable);
        JtDotQdot_ = Eigen::Matrix<T, -1, 1>::Zero(dim_of_task);
        delta_x_des_ = Eigen::Matrix<T, -1, 1>::Zero(dim_of_task);
        xdot_des_ = Eigen::Matrix<T, -1, 1>::Zero(dim_of_task);
        xddot_dest_ = Eigen::Matrix<T, -1, 1>::Zero(dim_of_task);
     }
    virtual ~Task() {}

    bool UpdateTask(const void* pos_des, const DVec<T>& vel_des,
                  const DVec<T>& acc_des) {
    _UpdateTaskJacobian();
    _UpdateTaskJDotQdot();
    _UpdateCommand(pos_des, vel_des, acc_des);
    _AdditionalUpdate();
    // b_set_task_ = true;
    return true;
    }

// protected:

    virtual void _UpdateTaskJacobian() = 0;

    virtual void _UpdateCommand(const void* pos_des, 
                        const DVec<T>& vel_des,
                        const DVec<T>& acc_des) = 0;
                        
    virtual bool _UpdateTaskJDotQdot() = 0;
    virtual bool _AdditionalUpdate() = 0;

    void getCommand(DVec<T>& op_cmd) { op_cmd = op_cmd_; }
    void getTaskJacobian(DMat<T>& Jt) { Jt = Jt_; }
    void getTaskJacobianDotQdot(DVec<T>& JtDotQdot) { JtDotQdot = JtDotQdot_; }
    const DVec<T>& getPosError() { return delta_x_des_; }
    const DVec<T>& getDesVel() { return xdot_des_; }
    const DVec<T>& getDesAcc() { return xddot_dest_; }

    size_t dim_task_, dim_of_decision_variable_;
    DVec<T> op_cmd_; // the final generalized acceleration that used to proceed InveseDynamics
    Eigen::Matrix<T, -1, -1> Jt_;           // task Jacobian matrix
    Eigen::Matrix<T, -1, 1> JtDotQdot_;
    Eigen::Matrix<T, -1, 1> delta_x_des_;   // operational command expressed in the global coordinates
    Eigen::Matrix<T, -1, 1> xdot_des_;
    Eigen::Matrix<T, -1, 1> xddot_dest_;
    
    
};



