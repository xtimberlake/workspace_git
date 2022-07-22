/*
 * @Author: haoyun 
 * @Date: 2022-07-22 14:42:15
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-22 14:44:01
 * @FilePath: /drake/workspace/fixed_centaur_sim/fixed_centaur_sim.cc
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#include "fixed_centaur_sim.h"

int main(int argc, char *argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::log()->info("hello centaur~");

    drake::workspace::fixed_centaur_sim::DoMain();
    return 0;
}