/*
 * @Author: haoyun 
 * @Date: 2022-09-16 17:07:22
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-10-12 20:33:33
 * @FilePath: /drake/workspace/centaur_sim/controller/WBIController.h
 * @Description: Whole-body impulse controller
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once

#include "drake/workspace/centaur_sim/controller/Tasks/TorsoPosTask.hpp"
#include "drake/workspace/centaur_sim/controller/Tasks/TorsoOriTask.hpp"
#include "drake/workspace/centaur_sim/controller/Tasks/LinkPosTask.hpp"
#include "drake/workspace/centaur_sim/controller/CentaurStates.h"
#include "drake/workspace/centaur_sim/dynamics/CentaurModel.h"
#include "drake/workspace/centaur_sim/controller/ContactSet/SingleContact.hpp"
#include "drake/workspace/centaur_sim/Utils/pseudoInverse.h"

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

    // support functions
    bool kin_wbcFindConfiguration(const DVec<double>& curr_config,
                         const std::vector<Task<double>*>& task_list,
                         const std::vector<ContactSpec<double>*>& contact_list,
                         DVec<double>& jpos_cmd, DVec<double>& jvel_cmd);
    void _PseudoInverse(const DMat<double> J, DMat<double>& Jinv);
    void _BuildProjectionMatrix(const DMat<double>& J, DMat<double>& N);

    // dynamics-wbc optimization functions
    void _SetOptimizationSize();
    void _ContactBuilding();
    void _SetCost();
    // void _SetEqualityConstraint(const DVec<double>& qddot);
    void _SetInEqualityConstraint();
    // void _GetSolution(const Dvec<double>& qddot, DVec<double>& cmd);




    std::vector<Task<double> * > _task_list;
    std::vector<ContactSpec<double> * > _contact_list;

    // contact pts
    ContactSpec<double>* _foot_contact[2];

    // task lists
    Task<double>* _torso_pos_task;
    Task<double>* _torso_ori_task;
    Task<double>* _foot_task[2];


    Eigen::Matrix<double, 12, 12> _A, _Ainv;   // mass matrix
    Eigen::Matrix<double, 12, 1> _coriolis;    // nonlinear term
    Eigen::Matrix<double, 12, 1> _grav;        // due to generalized gravity 
    Eigen::Matrix<double, 12, 1> tau_dist;     // disturbance term

    Quat<double> _quat_des;
    Vec3<double> _pBody_des;
    Vec3<double> _pFoot_des[4]; // why _pFoot_des[2] will occur error?

    DVec<double> _full_config;
    DVec<double> _tau_ff;
    DVec<double> _des_jpos;
    DVec<double> _des_jvel;

    CentaurModel ctModel;

    // kin wbc
    size_t num_qdot_;
    size_t num_act_joint_;
    DMat<double> I_mtx;

    /*--------dynamics wbc-------*/ 
    // optimization dimension
    size_t _num_qdot;
    size_t _dim_opt;      // floating base acc, reaction force
    size_t _dim_eq_cstr;  // equality constraints: 6 stands for floating base dyn.

    size_t _dim_rf;  // inequality constraints
    size_t _dim_Uf;  // statck all the contact friction pyramid

    size_t _dim_floating; // 6

    // for quadratic programming:
    // min_{_dim_opt}: ||_opt||_G + g0*_opt
    // s.t. CE * _opt =  ce0
    //      CI * _opt <= ci0
    DMat<double> G;   // weighting matrix that penalize the decision variables _opt
    DVec<double> g0;  // gradient vector
    DMat<double> CE;  // equality mapping matrix
    DVec<double> ce0; // equality vector
    DMat<double> CI;  // inequality mapping matrix
    DVec<double> ci0; // inequality vector

    // contact Jacobian & JcDotQdot & friction pyramid constrants
    DMat<double> _Jc;
    DVec<double> _JcDotQdot;
    DVec<double> _Fr_des; // grfs computed from MPC
    DMat<double> _Uf;
    DVec<double> _Uf_ieq_vec;


};



