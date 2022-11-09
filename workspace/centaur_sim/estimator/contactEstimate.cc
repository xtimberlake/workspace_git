/*
 * @Author: haoyun 
 * @Date: 2022-11-07 15:32:23
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-11-09 22:50:12
 * @FilePath: /drake/workspace/centaur_sim/estimator/contactEstimate.cc
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#include "drake/workspace/centaur_sim/estimator/contactEstimate.h"


template <typename T>
contactEstimate<T>::contactEstimate() {
    
    _s_contact = new bool[2]; // two legs

    _s_contact[0] = false;
    _s_contact[1] = false;

    _threshold = 0.95;
    _prob_contact.setZero();
    _prob_contact_plan.setZero();
    _prob_contact_pos.setZero();
    _prob_contact_force.setZero();

    Eigen::Matrix<T, 2, 2> I2; I2 = DMat<T>::Identity(2, 2);
    Eigen::Matrix<T, 2, 2> A; A.setZero();
    Eigen::Matrix<T, 2, 2> B; B = I2;
    Eigen::Matrix<T, 4, 2> H; 
    H.block(0, 0, 2, 2) = I2; 
    H.block(2, 0, 2, 2) = I2;
    Eigen::Matrix<T, 2, 2> Q; Q.diagonal() << 0.1, 0.1;
    Eigen::Matrix<T, 4, 4> R; R.diagonal() << 0.3, 0.3, 0.1, 0.1;
    
    kalmanFliter = new linearKalmanFilter<T>(A, B, H, Q, R);

}

template <typename T>
contactEstimate<T>::~contactEstimate() {}

template <typename T>
void contactEstimate<T>::updateEstimate(
                    const DVec<T>& phiContact, const DVec<T> phiSwing,
                    const DMat<T>& pFoot, const DMat<T>& forceFoot) {

    Eigen::Matrix<T, 2, 1> u_prob;    // instant input
    T miu_plan[2] = {0.0, 1.0};
    T sigma2_plan[2] = {0.025, 0.025};
    
    Eigen::Matrix<T, 2, 1> z1_prob;   // meas pFoot
    T miu_pz = -0.005;
    T sigma2_pz = 0.08;

    Eigen::Matrix<T, 2, 1> z2_prob;   // meas fGRF
    T miu_f = 120.0;
    T sigma2_f = 10.0;


    

    for (int leg = 0; leg < 2; leg++) {

        // priority #1: plan phases
        if(phiContact[leg] > 0)  {
            u_prob[leg] = 0.5 * (std::erf( (phiContact[leg]-miu_plan[0]) / (std::sqrt(2)*sigma2_plan[0]) )
                               + std::erf( (miu_plan[1]-phiContact[leg]) / (std::sqrt(2)*sigma2_plan[1]) ));
        }
        else {
            u_prob[leg] = 0.5 * (2 + std::erf( (miu_plan[0]-phiSwing[leg]) / (std::sqrt(2)*sigma2_plan[0]) )
                                   + std::erf( (phiSwing[leg]-miu_plan[1]) / (std::sqrt(2)*sigma2_plan[1]) ));
        }
        
        // priority #2: foot position
        T pFoot_z = -pFoot(2, leg);
        z1_prob[leg] = (1 + std::erf( (pFoot_z-miu_pz)/(std::sqrt(2)*sigma2_pz) ));

        // priority #3: vertical foot forces
        T fGrf_z = -forceFoot(2, leg);
        z2_prob[leg] = 0.5 * (1 + std::erf( (fGrf_z-miu_f)/(std::sqrt(2)*sigma2_f) ));
    }
    
    Eigen::Matrix<T, 4, 1> zk; zk.setZero();
    zk.head(2) = z1_prob;
    zk.tail(2) = z2_prob;
    

    this->_prob_contact_plan = u_prob;
    this->_prob_contact_pos = z1_prob;
    this->_prob_contact_force = z2_prob;


    kalmanFliter->updateInputMeasure(u_prob, zk);

    this->_prob_contact = kalmanFliter->getXhat();

    // DVec<T> res; 
    // res = kalmanFliter->getXhat();
    // std::cout << "res = " << res.transpose() << std::endl;
}
