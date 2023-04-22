/*
 * @Author: haoyun 
 * @Date: 2022-11-07 19:14:01
 * @LastEditors: haoyun 
 * @LastEditTime: 2023-04-21 20:44:04
 * @FilePath: /drake/workspace/centaur_sim/estimator/linearKalmanFilter.cc
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#include "drake/workspace/centaur_sim/estimator/linearKalmanFilter.h"

template <typename T>
linearKalmanFilter<T>::linearKalmanFilter(
                        const DMat<T>& A, const DMat<T>& B, const DMat<T>& H,
                        const DMat<T>& Q, const DMat<T>& R) 
    :_dim_state(A.rows()), _dim_meas(H.rows()), _dim_input(B.cols())
{
    _A = A;
    _B = B;
    _H = H;
    _Q = Q;
    _R = R;

    // check the dimension of matries
    if(_dim_state != _A.cols()) {throw std::runtime_error("[linearKF]: Amat must be suqare! \n");}
    if(_dim_state != _B.rows()) {throw std::runtime_error("[linearKF]: Bmat dimension error! \n");}
    if(_Q.rows() != _Q.cols()) {throw std::runtime_error("[linearKF]: Qmat must be suqare! \n");}
    if(_R.rows() != _R.cols()) {throw std::runtime_error("[linearKF]: Rmat must be suqare! \n");}
    if(_H.cols() != _dim_state) {throw std::runtime_error("[linearKF]: Hmat col dimension error! \n");}
    if(_Q.rows() != _dim_state) {throw std::runtime_error("[linearKF]: Q dimension error! \n");}
    if(_R.rows() != _dim_meas) {throw std::runtime_error("[linearKF]: R dimension error! \n");}


    _uk = DVec<T>::Zero(_dim_input);
    _zk = DVec<T>::Zero(_dim_meas);

    // initial condition(guess)
    _xk = DVec<T>::Zero(_dim_state);
    _errCovP.setIdentity(_dim_state, _dim_state);
    std::cout << "Initial P = " << std::endl << _errCovP << std::endl;

    
}


template <typename T>
linearKalmanFilter<T>::~linearKalmanFilter() {;}

template <typename T>
void linearKalmanFilter<T>::updateInputMeasure(const DVec<T>& uk, const DVec<T>& zk) {

    DVec<T> x_before; x_before.setZero(_dim_state);
    DMat<T> P_before; P_before.setZero(_dim_state, _dim_state);
    DMat<T> eye;      eye.setIdentity(_dim_state, _dim_state);
    DMat<T> Kk;       Kk.setZero(_dim_state, _dim_meas);
    _uk = uk;
    _zk = zk;
    
    // predict
    x_before = _A * _xk + _B * _uk;
    P_before = _A * _errCovP * _A.transpose() + _Q;
    
    // mesurement update !! need to update!
    Kk = P_before * _H.transpose() * (_H * P_before * _H.transpose() + _R).inverse();
    _xk = x_before + Kk * (_zk - _H * x_before);
    _errCovP = (eye - Kk * _H) * P_before;
   
}



