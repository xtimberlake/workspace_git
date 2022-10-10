/*
 * @Author: haoyun 
 * @Date: 2022-09-16 17:07:03
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-10-10 22:32:04
 * @FilePath: /drake/workspace/centaur_sim/controller/WBIController.cc
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#include "drake/workspace/centaur_sim/controller/WBIController.h"


WBIController::WBIController(/* args */) {

    _quat_des.setZero();
    _pBody_des.setZero();
    for (int i = 0; i < 4; i++) {
        _pFoot_des[i].setZero();
    }
    
  

    ctModel.buildModel();
    ctModel._fb_model.printModelTable();


    _torso_pos_task = new TorsoPosTask<double>(&(ctModel._fb_model));

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

}


void WBIController::update_model(CentaurStates& state) {
    
    state.tau_g.setZero();
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

    // fb_states.bodyPosition << 0, 0, 0.9; 
    // fb_states.bodyOrientation << 1, 0, 0, 0;
    // fb_states.q.setZero();
    // fb_states.qd.setZero();
    // fb_states.bodyVelocity.setZero();


    ctModel._fb_model.setState(fb_states);
    ctModel._fb_model.forwardKinematics();
    ctModel._fb_model.contactJacobians();
    ctModel._fb_model.massMatrix();
    ctModel._fb_model.generalizedGravityForce();
    ctModel._fb_model.generalizedCoriolisForce();
    
    
    // The equations of motion can be expressed as:
    // _A*qddot + _coriolis + _grav = tao + J^{T} * Fc
    _A = ctModel._fb_model.getMassMatrix();
    _grav = ctModel._fb_model.getGravityForce();
    _coriolis = ctModel._fb_model.getCoriolisForce();
    _Ainv = _A.inverse();
    
    // std::cout << spatial::translationFromSXform(this->ctModel._fb_model._Xa[8]).transpose() << std::endl;
    // std::cout << ctModel._fb_model._pGC.at(0).transpose() << std::endl;
    // std::cout << "foot pos: " 
    // << ctModel._fb_model._pGC.at(0)[0] << "/" << state.foot_pos_world.block<3, 1>(0, 0)[0] << ", "
    // << ctModel._fb_model._pGC.at(0)[1] << "/" << state.foot_pos_world.block<3, 1>(0, 0)[1] << ", "
    // << ctModel._fb_model._pGC.at(0)[2] << "/" << state.foot_pos_world.block<3, 1>(0, 0)[2] << std::endl;   

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
    _pFoot_des[0] = state.foot_pos_world.block<3, 1>(0, 0);
    _pFoot_des[1] = state.foot_pos_world.block<3, 1>(0, 1);

    // update desired variables for each tasks
    // Vec3<double> zero_vec3; zero_vec3.setZero();
    _torso_pos_task->_UpdateCommand(&_pBody_des, state.root_lin_vel_d_rel, state.root_acc_d_rel);
    
    // store in the task lists
    _task_list.push_back(_torso_pos_task);
    
    for (size_t leg(0); leg < 2; leg++)
    {
        if (state.plan_contacts_phase[leg] > 0.) { // contact
            _foot_contact[leg]->setRFDesired(state.foot_force_cmd_world.block<3, 1>(0, leg));
            _foot_contact[leg]->UpdateContactSpec();
            _contact_list.push_back(_foot_contact[leg]);

        }else{ // swing foot task
            _foot_task[leg]->_UpdateCommand(&_pFoot_des[leg],
                                            state.foot_vel_cmd_world.block<3, 1>(0, leg),
                                            state.foot_acc_cmd_world.block<3, 1>(0, leg));


        }
        
    }
    
    
}

void WBIController::kin_wbc() {


}


void WBIController::dyn_wbc() {
    
}
