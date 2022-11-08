/*
 * @Author: haoyun 
 * @Date: 2022-11-07 19:14:11
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-11-08 16:35:14
 * @FilePath: /drake/workspace/centaur_sim/estimator/linearKalmanFilter.h
 * @Description: implement the linear(invarient) Kalman Filter in the following model
 * 
 *  x_k = A_k * x_{k-1} + B * u_k + w_k
 *  z_k = H_k * x_{k-1} + v_k
 * 
 *  where w_k ~ N(0, Qk) and w_k ~ N(0, Rk) are the noises that subject to
 *  normal distribution.
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */

#include <iostream>
#include "drake/workspace/centaur_sim/Utils/cppTypes.h"

template <typename T>
class linearKalmanFilter
{
public:
    linearKalmanFilter(const DMat<T>& A, const DMat<T>& B, const DMat<T>& H,
                       const DMat<T>& Q, const DMat<T>& R);
    ~linearKalmanFilter();
    void setAmtx(const DMat<T>& A) { _A = A; }
    void setBmtx(const DMat<T>& B) { _B = B; }
    void setHmtx(const DMat<T>& H) { _H = H; }
    void setQmtx(const DMat<T>& Q) { _Q = Q; }
    void setRmtx(const DMat<T>& R) { _R = R; }
    DVec<T> getXhat() { return _xk; }
    void updateInputMeasure(const DVec<T>& uk, const DVec<T>& zk);

// variables:
    DMat<T> _A, _B, _H;
    DMat<T> _Q, _R;
    DVec<T> _xk, _uk, _zk;
    DMat<T> _errCovP;
    long int _dim_state, _dim_meas, _dim_input;
    
    
};


template class linearKalmanFilter<double>;
