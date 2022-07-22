/*
 * @Author: haoyun 
 * @Date: 2022-07-19 11:07:07
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-19 11:14:17
 * @FilePath: /drake/workspace/centaur_sim/Utils/Utils.h
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
//
// Created by shuoy on 10/19/21.
//

#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <type_traits>


class Utils {
public:
    // compare to Eigen's default eulerAngles
    // this function returns yaw angle within -pi to pi
    static Eigen::Vector3d quat_to_euler(Eigen::Quaterniond quat);
    static Eigen::Matrix3d skew(Eigen::Vector3d vec);
    static Eigen::Matrix3d pseudo_inverse(const Eigen::Matrix3d &mat);
    static double cal_dihedral_angle(Eigen::Vector3d surf_coef_1, Eigen::Vector3d surf_coef_2);
};

class BezierUtils {
    // TODO: allow degree change? may not be necessary, we can stick to degree 4
public:
    BezierUtils () {
        curve_constructed = false;
        bezier_degree = 4;
    }
    // set of functions create bezier curves, get points, reset
    Eigen::Vector3d get_foot_pos_curve(float t,
                                       Eigen::Vector3d foot_pos_start,
                                       Eigen::Vector3d foot_pos_final,
                                       double terrain_pitch_angle);

    void reset_foot_pos_curve() {curve_constructed = false;}
private:
    double bezier_curve(double t, const std::vector<double> &P);

    bool curve_constructed;
    float bezier_degree;
};

template<typename T>
class FootSwingTrajectory {
public:
  FootSwingTrajectory() {
    _p0.setZero();
    _pf.setZero();
    _p.setZero();
    _v.setZero();
    _a.setZero();
    _height = 0;
  }

  void setInitialPosition(Eigen::Matrix<T, 3, 1> p0) {
    _p0 = p0;
  }

  void setFinalPosition(Eigen::Matrix<T, 3, 1> pf) {
    _pf = pf;
  }

  void setHeight(T h) {
    _height = h;
  }

  void computeSwingTrajectoryBezier(T phase, T swingTime);

  Eigen::Matrix<T, 3, 1> getPosition() {
    return _p;
  }

  Eigen::Matrix<T, 3, 1> getVelocity() {
    return _v;
  }

  Eigen::Matrix<T, 3, 1> getAcceleration() {
    return _a;
  }

private:
  Eigen::Matrix<T, 3, 1> _p0, _pf, _p, _v, _a;
  T _height;
};

namespace Interpolate {

/*!
 * Linear interpolation between y0 and yf.  x is between 0 and 1
 */
template <typename y_t, typename x_t>
y_t lerp(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  return y0 + (yf - y0) * x;
}

/*!
 * Cubic bezier interpolation between y0 and yf.  x is between 0 and 1
 */
template <typename y_t, typename x_t>
y_t cubicBezier(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  y_t yDiff = yf - y0;
  x_t bezier = x * x * x + x_t(3) * (x * x * (x_t(1) - x));
  return y0 + bezier * yDiff;
}

/*!
 * Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and
 * 1
 */
template <typename y_t, typename x_t>
y_t cubicBezierFirstDerivative(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  y_t yDiff = yf - y0;
  x_t bezier = x_t(6) * x * (x_t(1) - x);
  return bezier * yDiff;
}

/*!
 * Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and
 * 1
 */
template <typename y_t, typename x_t>
y_t cubicBezierSecondDerivative(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  y_t yDiff = yf - y0;
  x_t bezier = x_t(6) - x_t(12) * x;
  return bezier * yDiff;
}

}  // namespace Interpolate
