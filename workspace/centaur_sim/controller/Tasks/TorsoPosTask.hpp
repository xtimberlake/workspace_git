/*
 * @Author: haoyun 
 * @Date: 2022-09-17 16:49:42
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-09-23 10:12:35
 * @FilePath: /drake/workspace/centaur_sim/controller/Tasks/TorsoPosTask.hpp
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once

#include "drake/workspace/centaur_sim/controller/Task.hpp"


template <typename T>
class TorsoPosTask : public Task<T> 
{
public:
    TorsoPosTask();
    ~TorsoPosTask();

    void _UpdateTaskJacobian() override;

    void _UpdateCommand(const Eigen::Matrix<T, -1, 1>& pos_des,
                                const Eigen::Matrix<T, -1, 1>& vel_des,
                                const Eigen::Matrix<T, -1, 1>& acc_des) override;

};
