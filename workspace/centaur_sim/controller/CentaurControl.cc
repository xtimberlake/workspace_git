/*
 * @Author: haoyun 
 * @Date: 2022-07-16 14:31:07
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-18 10:18:12
 * @FilePath: /drake/workspace/centaur_sim/controller/CentaurControl.cc
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#include "drake/workspace/centaur_sim/controller/CentaurControl.h"

CentaurControl::CentaurControl() {

    Eigen::VectorXd q(10);
    q << 1.0,5.0;
    mpc_solver = new ConvexMPC(10, q, Eigen::Vector3d(1.0, 4.0, 1.0));
}
