/*
 * @Author: haoyun 
 * @Date: 2022-09-16 17:07:03
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-10-12 20:48:35
 * @FilePath: /drake/workspace/centaur_sim/controller/WBIController.cc
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#include "drake/workspace/centaur_sim/controller/WBIController.h"


WBIController::WBIController(/* args */):
    _full_config(centaurParam::num_act_joint + 7),
    _tau_ff(centaurParam::num_act_joint),
    _des_jpos(centaurParam::num_act_joint),
    _des_jvel(centaurParam::num_act_joint) {



    num_qdot_ = centaurParam::dim_config; // 12
    num_act_joint_ = centaurParam::num_act_joint; // 6
    I_mtx =  DMat<double>::Identity(num_qdot_, num_qdot_); // 12x 12 Identity 

    _quat_des.setZero();
    _pBody_des.setZero();
    for (int i = 0; i < 4; i++) {
        _pFoot_des[i].setZero();
    }

    _full_config.setZero();
    _tau_ff.setZero();
    _des_jpos.setZero();
    _des_jvel.setZero();
  

    ctModel.buildModel();
    ctModel._fb_model.printModelTable();


    _torso_pos_task = new TorsoPosTask<double>(&(ctModel._fb_model));
    _torso_ori_task = new TorsoOriTask<double>(&(ctModel._fb_model));

    for (int i = 0; i < 2; i++) {
        _foot_contact[i] = new SingleContact<double>(&(ctModel._fb_model), i); // foot contact
        _foot_task[i] = new LinkPosTask<double>(&(ctModel._fb_model), i);
    }
    
}


WBIController::~WBIController(){ }


void WBIController::run(CentaurStates& state)
{
    update_model(state);
    update_contact_task(state);
    kin_wbc();
    dyn_wbc();
}


void WBIController::update_model(CentaurStates& state) {
    

    // Part1: update configuration of floating base model;
    // and store the dynamics matrix in whole-body controller.

    FBModelState<double> fb_states;
    /* The quat's coeffs in Drake is ordered as (x, y, z, w), which
       is different from MIT-Cheetah code*/
    fb_states.bodyOrientation[0] = state.root_quat.coeffs()[3];
    fb_states.bodyOrientation[1] = state.root_quat.coeffs()[0];
    fb_states.bodyOrientation[2] = state.root_quat.coeffs()[1];
    fb_states.bodyOrientation[3] = state.root_quat.coeffs()[2];

    fb_states.bodyPosition = state.root_pos; // Vec_from_world_to_base, expressed in world
    fb_states.bodyVelocity.head(3) = state.root_ang_vel_rel;
    fb_states.bodyVelocity.tail(3) = state.root_lin_vel_rel;
    fb_states.q = state.q;
    fb_states.qd = state.qdot;

    ctModel._fb_model.setState(fb_states);
    ctModel._fb_model.forwardKinematics();
    ctModel._fb_model.contactJacobians();
    ctModel._fb_model.massMatrix();
    ctModel._fb_model.generalizedGravityForce();
    ctModel._fb_model.generalizedCoriolisForce();
    
    
    // The equations of motion can be expressed as:
    // _A*qddot + _coriolis + _grav = tau + J^{T} * Fc
    _A = ctModel._fb_model.getMassMatrix();
    _grav = ctModel._fb_model.getGravityForce();
    _coriolis = ctModel._fb_model.getCoriolisForce();
    _Ainv = _A.inverse();
    
    // Part 2: copy the configuration(only care about the joint configuration 
    // in kinamatics-WBC?)
    for(size_t i(0); i<3; ++i){
        for (size_t leg = 0; leg < 2; leg++) {
            _full_config[3*leg + i + 6] = state.q[3*leg + i];
        }
    }
    


}

void WBIController::clean_up() {
    _task_list.clear();
    _contact_list.clear();
}

void WBIController::update_contact_task(CentaurStates& state) {

    // wash out the previous setup
    clean_up();
    
    // receice target position
    _pBody_des = state.root_pos_d;
    _quat_des = ori::rpyToQuat(state.root_euler_d);
    _pFoot_des[0] = state.foot_pos_cmd_world.block<3, 1>(0, 0);
    _pFoot_des[1] = state.foot_pos_cmd_world.block<3, 1>(0, 1);

    // update desired variables for each tasks
    // Vec3<double> zero_vec3; zero_vec3.setZero();
    _torso_pos_task->UpdateTask(&_pBody_des, state.root_lin_vel_d_world, state.root_acc_d_world);
    _torso_ori_task->UpdateTask(&_quat_des, state.root_ang_vel_d_world, state.root_ang_acc_d_world);

    // store in the task lists
    _task_list.push_back(_torso_pos_task);
    _task_list.push_back(_torso_ori_task);
    
    for (size_t leg(0); leg < 2; leg++)
    {
        if (state.plan_contacts_phase[leg] > 0.) { // contact
            _foot_contact[leg]->setRFDesired(state.foot_force_cmd_world.block<3, 1>(0, leg));
            _foot_contact[leg]->UpdateContactSpec();
            _contact_list.push_back(_foot_contact[leg]);

        }else{ // swing foot task
            _foot_task[leg]->UpdateTask(&_pFoot_des[leg],
                                            state.foot_vel_cmd_world.block<3, 1>(0, leg),
                                            state.foot_acc_cmd_world.block<3, 1>(0, leg));
            _task_list.push_back(_foot_task[leg]);
        }
    }
    
    
}

void WBIController::kin_wbc() {
    kin_wbcFindConfiguration(_full_config, _task_list, _contact_list,
                              _des_jpos, _des_jvel);
}


void WBIController::dyn_wbc() {

    // resize G, g0, CE, ce0, CI, ci0
    _SetOptimizationSize();
    _SetCost();
    
}



bool WBIController::kin_wbcFindConfiguration(const DVec<double>& curr_config,
                         const std::vector<Task<double>*>& task_list,
                         const std::vector<ContactSpec<double>*>& contact_list,
                         DVec<double>& jpos_cmd, DVec<double>& jvel_cmd) {

    // Contact Jacobian Setup
    // stack all the contact Jacobian matrix togather
    DMat<double> Nc(num_qdot_, num_qdot_); Nc.setIdentity();                        
    if(contact_list.size() > 0){
        DMat<double> Jc, Jc_i;
        contact_list[0]->getContactJacobian(Jc);
        size_t num_rows = Jc.rows();

        for (size_t i(1); i < contact_list.size(); ++i) {
        contact_list[i]->getContactJacobian(Jc_i);
        size_t num_new_rows = Jc_i.rows();
        Jc.conservativeResize(num_rows + num_new_rows, num_qdot_);
        Jc.block(num_rows, 0, num_new_rows, num_qdot_) = Jc_i;
        num_rows += num_new_rows;
        }

        // Projection Matrix
        _BuildProjectionMatrix(Jc, Nc);
    }

    // First Task
    DVec<double> delta_q, qdot;
    DMat<double> Jt, JtPre, JtPre_pinv, N_nx, N_pre;

    Task<double>* task = task_list[0];
    task->getTaskJacobian(Jt);
    JtPre = Jt * Nc;
    _PseudoInverse(JtPre, JtPre_pinv);

    delta_q = JtPre_pinv * (task->getPosError());
    qdot = JtPre_pinv * (task->getDesVel());

    DVec<double> prev_delta_q = delta_q;
    DVec<double> prev_qdot = qdot;

    _BuildProjectionMatrix(JtPre, N_nx);
    N_pre = Nc * N_nx;
    // recall: an iterative approach to calculate Null space projection matrix:
    // N_2 = N_1 * NullSpace(J_2 * N_1)
    // N_3 = N_2 * NullSpace(J_3 * N_2)
    // ...


    for (size_t i(1); i < task_list.size(); ++i) {
        task = task_list[i];

        task->getTaskJacobian(Jt);
        JtPre = Jt * N_pre;

        _PseudoInverse(JtPre, JtPre_pinv);
        delta_q =
            prev_delta_q + JtPre_pinv * (task->getPosError() - Jt * prev_delta_q);
        qdot = prev_qdot + JtPre_pinv * (task->getDesVel() - Jt * prev_qdot);

        // For the next task
        _BuildProjectionMatrix(JtPre, N_nx);
        N_pre *= N_nx;
        prev_delta_q = delta_q;
        prev_qdot = qdot;
    }

    for (size_t i(0); i < num_act_joint_; ++i) {
        jpos_cmd[i] = curr_config[i + 6] + delta_q[i + 6];
        jvel_cmd[i] = qdot[i + 6];
    }

    return true;
}


void WBIController::_SetOptimizationSize() {

  // Dimension
  _dim_rf = 0;  // Dimension of contact forces
  _dim_Uf = 0;  // Dimension of inequality constraint
  for (size_t i(0); i < (_contact_list).size(); ++i) {
    _dim_rf += (_contact_list)[i]->getDim();
    _dim_Uf += (_contact_list)[i]->getDimRFConstraint();
  }

  _dim_floating = 6;
  _dim_opt = _dim_floating + _dim_rf;
  _dim_eq_cstr = _dim_floating; // floating base dyn. equality constraints

  G.resize(_dim_opt, _dim_opt); G.setZero();
  g0.resize(_dim_opt); g0.setZero();
  CE.resize(_dim_opt, _dim_eq_cstr); CE.setZero();
  ce0.resize(_dim_eq_cstr); ce0.setZero();

  if(_dim_rf > 0) { // at least have one contact
    CI.resize(_dim_opt, _dim_Uf); CI.setZero();
    ci0.resize(_dim_Uf); ci0.setZero();
    
    _Jc = DMat<double>::Zero(_dim_rf, num_qdot_);
    _JcDotQdot = DVec<double>::Zero(_dim_rf);
    _Fr_des = DVec<double>::Zero(_dim_rf);

    _Uf = DMat<double>::Zero(_dim_Uf, _dim_rf);
    _Uf_ieq_vec = DVec<double>::Zero(_dim_Uf);

  } else {
    CI.resize(_dim_opt, 1); CI.setZero();
    ci0.resize(1); ci0.setZero();
  }
  
}
void WBIController::_ContactBuilding() {


}
void WBIController::_SetCost() {
    // Set Cost
  size_t idx_offset(0);
  for (size_t i(0); i < _dim_floating; ++i) {
    G(i + idx_offset,i + idx_offset) = 0.1;
  }
  idx_offset += _dim_floating;
  for (size_t i(0); i < _dim_rf; ++i) {
    G(i + idx_offset, i + idx_offset) = 1.0;
  }

//   std::cout << G << std::endl;

}
void WBIController::_SetInEqualityConstraint() {

}


// Support functions
void WBIController::_PseudoInverse(const DMat<double> J, DMat<double>& Jinv) {
    pseudoInverse(J, 0.001, Jinv);
}
void WBIController::_BuildProjectionMatrix(const DMat<double>& J, DMat<double>& N) {
    // the simplest method to build a projection matrix
    DMat<double> J_pinv;
    _PseudoInverse(J, J_pinv);
    N = I_mtx - J_pinv * J;
}