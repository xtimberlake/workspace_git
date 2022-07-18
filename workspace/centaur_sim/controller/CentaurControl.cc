/*
 * @Author: haoyun 
 * @Date: 2022-07-16 14:31:07
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-18 17:45:54
 * @FilePath: /drake/workspace/centaur_sim/controller/CentaurControl.cc
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#include "drake/workspace/centaur_sim/controller/CentaurControl.h"

CentaurControl::CentaurControl(const control_params_constant ctrl_params) {


    DRAKE_DEMAND(ctrl_params.mpc_horizon == MPC_HORIZON);
    this->mpc_horizon = ctrl_params.mpc_horizon;

    this->mpc_dt = ctrl_params.control_dt;

    DRAKE_DEMAND(ctrl_params.q_weights.size() == NUM_STATE);
    for (size_t i = 0; i < ctrl_params.q_weights.size(); i++)
        this->mpc_q_weights(i) = ctrl_params.q_weights.at(i);

    DRAKE_DEMAND(ctrl_params.r_weights.size() == NUM_U);
    for (size_t i = 0; i < ctrl_params.r_weights.size(); i++)
        this->mpc_r_weights(i) = ctrl_params.r_weights.at(i);   
        
    mpc_solver = new ConvexMPC(this->mpc_horizon, this->mpc_dt, this->mpc_q_weights, this->mpc_r_weights);


}

void CentaurControl::compute_grf(CentaurStates& state)
{
    mpc_solver->update_A_d(state.root_euler);

}
