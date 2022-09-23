/*
 * @Author: haoyun 
 * @Date: 2022-09-16 17:07:03
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-09-23 21:14:23
 * @FilePath: /drake/workspace/centaur_sim/controller/WBIController.cc
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#include "drake/workspace/centaur_sim/controller/WBIController.h"


WBIController::WBIController(/* args */) {

    ctModel.buildModel();

    ctModel._fb_model.printModelTable();
    
}


WBIController::~WBIController(){ }


void WBIController::run(CentaurStates& state)
{
    update_model(state);
    // this->ctModel->buildModel();

}


void WBIController::update_model(CentaurStates& state) {
    
    state.tau_g.setZero();
    FBModelState<double> fb_states;
    /* The quat's coeffs is ordered as (x, y, z, w), which
       is different from MIT code*/
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


    ctModel._fb_model.setState(fb_states);;
    ctModel._fb_model.forwardKinematics();
    
    // std::cout << spatial::translationFromSXform(this->ctModel._fb_model._Xa[8]).transpose() << std::endl;
    // std::cout << ctModel._fb_model._pGC.at(0).transpose() << std::endl;
    std::cout << "foot pos: " 
    << ctModel._fb_model._pGC.at(0)[0] << "/" << state.foot_pos_world.block<3, 1>(0, 0)[0] << ", "
    << ctModel._fb_model._pGC.at(0)[1] << "/" << state.foot_pos_world.block<3, 1>(0, 0)[1] << ", "
    << ctModel._fb_model._pGC.at(0)[2] << "/" << state.foot_pos_world.block<3, 1>(0, 0)[2] << std::endl;   

}



void WBIController::update_task_jacobian(CentaurStates& state) {
    state.Cv[0] = 0;
}

void WBIController::kin_wbc() {


}


void WBIController::dyn_wbc() {
    
}
