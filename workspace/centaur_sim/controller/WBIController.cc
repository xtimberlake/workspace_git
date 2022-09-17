/*
 * @Author: haoyun 
 * @Date: 2022-09-16 17:07:03
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-09-17 20:02:57
 * @FilePath: /drake/workspace/centaur_sim/controller/WBIController.cc
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#include "drake/workspace/centaur_sim/controller/WBIController.h"


WBIController::WBIController(/* args */) { }


WBIController::~WBIController(){ }


void WBIController::run(CentaurStates& state)
{
    update_model(state);

}


void WBIController::update_model(CentaurStates& state) {
    state.Cv[0] = 0;

}



void WBIController::update_task_jacobian(CentaurStates& state) {
    state.Cv[0] = 0;
}

void WBIController::kin_wbc() {


}


void WBIController::dyn_wbc() {
    
}
