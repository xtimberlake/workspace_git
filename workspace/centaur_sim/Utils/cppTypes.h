/*
 * @Author: haoyun 
 * @Date: 2022-09-19 16:18:02
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-09-19 16:24:39
 * @FilePath: /drake/workspace/centaur_sim/Utils/cppTypes.h
 * @Description: define Eigen types, template types, aliases, ...
 * 
 * from MIT open source code
 */
#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>

// Rotation Matrix
template <typename T>
using RotMat = typename Eigen::Matrix<T, 3, 3>;

