/*
 * @Author: haoyun 
 * @Date: 2022-09-16 17:07:22
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-10-10 22:28:56
 * @FilePath: /drake/workspace/centaur_sim/controller/WBIController.h
 * @Description: Whole-body impulse controller
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once

#include "drake/workspace/centaur_sim/controller/Tasks/TorsoPosTask.hpp"
#include "drake/workspace/centaur_sim/controller/Tasks/LinkPosTask.hpp"
#include "drake/workspace/centaur_sim/controller/CentaurStates.h"
#include "drake/workspace/centaur_sim/dynamics/CentaurModel.h"
#include "drake/workspace/centaur_sim/controller/ContactSet/SingleContact.hpp"

// template<typename T>
// class TorsoPosTask;

class WBIController
{
public:
    WBIController(/* args */);
    ~WBIController();
    void run(CentaurStates& state);
    void update_model(CentaurStates& state);
    void update_contact_task(CentaurStates& state);
    void kin_wbc();
    void dyn_wbc();
    void clean_up();


    std::vector<Task<double> * > _task_list;
    std::vector<ContactSpec<double> * > _contact_list;

    // contact pts
    ContactSpec<double>* _foot_contact[2];

    // task lists
    Task<double>* _torso_pos_task;
    Task<double>* _foot_task[2];


    Eigen::Matrix<double, 12, 12> _A, _Ainv;   // mass matrix
    Eigen::Matrix<double, 12, 1> _coriolis;    // nonlinear term
    Eigen::Matrix<double, 12, 1> _grav;        // due to generalized gravity 
    Eigen::Matrix<double, 12, 1> tau_dist;     // disturbance term

    Quat<double> _quat_des;
    Vec3<double> _pBody_des;
    Vec3<double> _pFoot_des[4]; // why [2] will occur error?


    CentaurModel ctModel;
};



