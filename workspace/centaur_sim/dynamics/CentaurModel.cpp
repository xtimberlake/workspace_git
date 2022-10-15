/*
 * @Author: haoyun 
 * @Date: 2022-09-19 20:29:47
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-10-15 10:13:40
 * @FilePath: /drake/workspace/centaur_sim/dynamics/CentaurModel.cpp
 * @Description: build the dynamic model of Centaur robot
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#include "drake/workspace/centaur_sim/dynamics/CentaurModel.h"

CentaurModel::CentaurModel() {
    // torso 
    
    _torsoLength = 0.432;
    _torsoWidth = 0.146;
    _torseHeight = 0.17;
    _torso_rotational_inertia.diagonal() = Vec3<double>(0.0837, 0.3592, 0.3465);
    _torsoMass = 20.0;
    _torsoInertia = SpatialInertia<double>(_torsoMass, Vec3<double>::Zero(), _torso_rotational_inertia);


    // (left)abAd hip link
    _abAd_pos_from_torso << -0.048, 0.173, 0;
    _abAd_rot_from_torso = Mat3<double>::Identity();
    _abAd_link_length = 0.14;
    _abAd_rotational_inertia.diagonal() = Vec3<double>(0.0053, 0.0053, 0.006);
    _abAd_CoM_from_joint_frame << -_abAd_link_length, 0.04, 0;
    _abAd_link_mass = 1.7;
    _abAdInertia = SpatialInertia<double>(_abAd_link_mass, _abAd_CoM_from_joint_frame, _abAd_rotational_inertia);
    

    _hip_pos_from_adAd << -_abAd_link_length, 0.04, 0;
    _hip_rot_from_adAd = Mat3<double>::Identity();
    _hip_link_length = 0.512;
    _hip_rotational_inertia.diagonal() = Vec3<double>(0.0141, 0.0141, 0.0001);
    _hip_CoM_from_joint_frame << 0, 0, -_hip_link_length/2.0;
    _hip_link_mass = 0.5;
    _hipInertia = SpatialInertia<double>(_hip_link_mass, _hip_CoM_from_joint_frame, _hip_rotational_inertia);
    
  
    _knee_pos_from_hip << 0, 0, -_hip_link_length;
    // _knee_pos_from_hip = -_knee_pos_from_hip;
    _knee_rot_from_hip = Mat3<double>::Identity();
    _Xtree_knee = spatial::createSXform(_knee_rot_from_hip, _knee_pos_from_hip);
    _knee_link_length = 0.48;
    _knee_rotational_inertia.diagonal() = Vec3<double>(0.0083, 0.0083, 0.0001);
    _knee_CoM_from_joint_frame << 0, 0, -_knee_link_length/2.0;
    _knee_link_mass = 0.5;
    _kneeInertia = SpatialInertia<double>(_knee_link_mass, _knee_CoM_from_joint_frame, _knee_rotational_inertia);
    
}

CentaurModel::~CentaurModel() {}

void CentaurModel::buildModel() {
    
    _fb_model.addBase(_torsoInertia);

    // note: this ID counts from zero
    const int baseID = 5;
    int bodyId = 5;
    std::string adAd_link[2] = {"left abAD link", "right ab/ad link"};
    std::string hip_link[2] = {"left hip link", "right hip link"};
    std::string knee_link[2] = {"left knee link", "right knee link"};

    for (int i = 0; i < 2; i++)
    {
        // i = 0 for left, 1 for right

        // abAd link
        bodyId++; // 6, 9
        _abAd_pos_from_torso[1] = powf64(-1, i) * _abAd_pos_from_torso[1]; //flip along y axis
        _Xtree_abAd = spatial::createSXform(_abAd_rot_from_torso, _abAd_pos_from_torso);
        _fb_model.addBody(adAd_link[i], _abAdInertia, baseID, JointType::Revolute,
                   ori::CoordinateAxis::X, _Xtree_abAd);

        // hip link
        bodyId++; // 7, 10
        _hip_pos_from_adAd[1] = powf64(-1, i) * _hip_pos_from_adAd[1];
        _Xtree_hip = spatial::createSXform(_hip_rot_from_adAd, _hip_pos_from_adAd);
        _fb_model.addBody(hip_link[i], _hipInertia, bodyId-1, JointType::Revolute,
                    ori::CoordinateAxis::Y, _Xtree_hip);

        // knee link
        bodyId++; // 8, 11
        _fb_model.addBody(knee_link[i],_kneeInertia, bodyId-1, JointType::Revolute,
                    ori::CoordinateAxis::Y, _Xtree_knee);
        // add contact point at the foot-end
        _fb_model.addGroundContactPoint(bodyId, Vec3<double>(0, 0, -_knee_link_length-0.01)); // 0.01 is the radiu of foot ball
    }
    


}
