/*
 * @Author: haoyun 
 * @Date: 2023-02-02 17:02:00
 * @LastEditors: haoyun 
 * @LastEditTime: 2023-02-02 17:11:32
 * @FilePath: /drake/workspace/backpack_sim/backpack_sim.cc
 * @Description: 
 * 
 * Copyright (c) 2023 by HAR-Lab, All Rights Reserved. 
 */
#include "drake/workspace/backpack_sim/backpack_sim.h"

int main(int argc, char *argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::log()->info("hello backpack~");

    drake::workspace::backpack_sim::DoMain();
    return 0;
}
