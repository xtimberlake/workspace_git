/*
 * @Author: haoyun 
 * @Date: 2022-07-18 14:20:26
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-22 20:32:06
 * @FilePath: /drake/workspace/centaur_sim/controller/CentaurParams.h
 * @Description: define some const
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once

#ifndef CT_PARAMS_H
#define CT_PARAMS_H

#define FILP_DIR -1


#define NUM_LEG 2

// mpc info 
#define MPC_HORIZON 10
#define NUM_STATE 12
#define NUM_U 6

#define FOOT_SWING_CLEARANCE1 0.0f
#define FOOT_SWING_CLEARANCE2 0.2f

#define FOOT_DELTA_X_LIMIT 0.25f
#define FOOT_DELTA_Y_LIMIT 0.25f




#endif //CT_PARAMS_H