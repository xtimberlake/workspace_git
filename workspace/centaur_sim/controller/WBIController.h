/*
 * @Author: haoyun 
 * @Date: 2022-09-16 17:07:22
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-10-14 15:35:13
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
#include "drake/workspace/centaur_sim/controller/ConvexMPC.h"

struct wbc_interface_data {
    Eigen::Matrix<double, 6, 1> qj_cmd;
    Eigen::Matrix<double, 6, 1> qjdot_cmd;
    Eigen::Matrix<double, 6, 1> tau_ff;
};

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

    bool kin_wbcFindConfiguration(const DVec<double>& curr_config,
                         const std::vector<Task<double>*>& task_list,
                         const std::vector<ContactSpec<double>*>& contact_list,
                         DVec<double>& jpos_cmd, DVec<double>& jvel_cmd);
    

    // dynamics-wbc optimization functions
    void _SetOptimizationSize();
    void _ContactBuilding();
    void _SetCost();
    void _SetEqualityConstraint(const DVec<double>& qddot);
    void _SetInEqualityConstraint();    
    double _SolveQuadraticProgramming(Eigen::VectorXd& z);
    void _InverseDyn(const DVec<double>& qddot_cmd, DVec<double>& tao_j);
    void update_command(CentaurStates& state, const DVec<double>& qj, const DVec<double>& qj_dot, const DVec<double>& tau);

    // support functions
    void _PseudoInverse(const DMat<double> J, DMat<double>& Jinv) {
        pseudoInverse(J, 0.001, Jinv);
    }
    void _BuildProjectionMatrix(const DMat<double>& J, DMat<double>& N) {
        // the simplest method to build a projection matrix
        DMat<double> J_pinv;
        _PseudoInverse(J, J_pinv);
        N = I_mtx - J_pinv * J;
    }
    void _WeightedInverse(const DMat<double>& J, const DMat<double>& Winv, DMat<double>& Jinv,
                        double threshold = 0.0001) {
        DMat<double> lambda(J * Winv * J.transpose());
        DMat<double> lambda_inv;
        pseudoInverse(lambda, threshold, lambda_inv);
        Jinv = Winv * J.transpose() * lambda_inv;                    
    }


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
    DVec<double> _f_mpc;

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
    DVec<double> z_star;
    DVec<double> initial_guess_vec;
    size_t last_dim_decision_variables;

    // contact Jacobian & JcDotQdot & friction pyramid constrants
    DMat<double> _Jc;
    DVec<double> _JcDotQdot;
    DVec<double> _Fr_des; // grfs computed from MPC
    DMat<double> _Uf;
    DVec<double> _Uf_ieq_vec;

    // identity mtx
    DMat<double> _eye;
    DMat<double> _eye_floating;

    // select mtx
    DMat<double> Sa_;  // Actuated joint
    DMat<double> Sv_;  // Virtual joint


};






