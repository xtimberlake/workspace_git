/*
 * @Author: haoyun 
 * @Date: 2022-09-19 20:30:21
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-09-22 19:28:48
 * @FilePath: /drake/workspace/centaur_sim/dynamics/CentaurModel.h
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once 
#include "drake/workspace/centaur_sim/dynamics/FloatingBaseModel.h"



class CentaurModel {
 public:
    CentaurModel();
    ~CentaurModel();

    void buildModel();

    // physical parameters; the inertia value is expressed
    // in the body(local) frame.
    double _torsoLength, _torsoWidth, _torseHeight;
    Eigen::Matrix3d _torso_rotational_inertia;
    double _torsoMass;
    SpatialInertia<double> _torsoInertia;

    Eigen::Vector3d _abAd_pos_from_torso;
    RotMat<double> _abAd_rot_from_torso;
    SXform<double> _Xtree_abAd;
    double _abAd_link_length;
    Eigen::Matrix3d _abAd_rotational_inertia;
    Eigen::Vector3d _abAd_CoM_from_joint_frame;
    double _abAd_link_mass;
    SpatialInertia<double> _abAdInertia;

    Eigen::Vector3d _hip_pos_from_adAd;
    RotMat<double> _hip_rot_from_adAd;
    SXform<double> _Xtree_hip;
    double _hip_link_length; // thigh length
    Eigen::Matrix3d _hip_rotational_inertia;
    Eigen::Vector3d _hip_CoM_from_joint_frame;
    double _hip_link_mass;
    SpatialInertia<double> _hipInertia;

    
    Eigen::Vector3d _knee_pos_from_hip;
    RotMat<double> _knee_rot_from_hip;
    SXform<double> _Xtree_knee;
    double _knee_link_length; // shank length
    Eigen::Matrix3d _knee_rotational_inertia;
    Eigen::Vector3d _knee_CoM_from_joint_frame;
    double _knee_link_mass;
    SpatialInertia<double> _kneeInertia;
    

    // floating base model parameters
    FloatingBaseModel _fb_model;
    
};