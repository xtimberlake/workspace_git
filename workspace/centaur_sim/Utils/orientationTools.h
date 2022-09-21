/*
 * @Author: haoyun 
 * @Date: 2022-09-20 11:26:43
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-09-20 11:34:15
 * @FilePath: /drake/workspace/centaur_sim/Utils/orientationTools.h
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once
#include "drake/workspace/centaur_sim/Utils/cppTypes.h"

namespace ori {

template <typename T>
Mat3<typename T::Scalar> vectorToSkewMat(const Eigen::MatrixBase<T>& v) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                "Must have 3x1 matrix");
  Mat3<typename T::Scalar> m;
  m << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
  return m;
}

}
