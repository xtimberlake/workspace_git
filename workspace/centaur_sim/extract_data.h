/*
 * @Author: haoyun 
 * @Date: 2022-07-19 09:55:16
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-10-22 21:12:36
 * @FilePath: /drake/workspace/centaur_sim/extract_data.h
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/multibody/plant/multibody_plant.h"
#include <drake/multibody/tree/multibody_tree.h>
#include <iostream>

namespace drake{
namespace workspace{
namespace centaur_sim{


template <typename T>
class extractData : public drake::systems::LeafSystem<T>
{
public:
    extractData(const multibody::MultibodyPlant<T>& plant)
        : _plant(plant) {

        _plant_context = _plant.CreateDefaultContext();
        this->DeclareVectorInputPort("sim_scene_states", 24);
        std::cout<< (this->DeclareAbstractInputPort("spatial_forces_in",
            Value<std::vector<drake::multibody::SpatialForce<double>>>()).get_index());
        std::cout << "index is above" << std::endl;

        this->DeclareVectorOutputPort(
            "full_states", 25,
            &extractData::ExtractFullStates);

        this->DeclareVectorOutputPort(
            "log_data", 18,
            &extractData::CalcLogData);
        
        this->DeclareVectorOutputPort(
            "position_rotation", 12,
            &extractData::ExtractPosRot);
        
    }

private:
    const multibody::MultibodyPlant<T>& _plant;
    std::unique_ptr<systems::Context<T>> _plant_context;

    void ExtractFullStates(const systems::Context<T>& context,
                           systems::BasicVector<T>* output) const {

        Eigen::VectorXd full_states(25);
        Eigen::Vector4d quat;
        Eigen::Vector3d pos;
        Eigen::Vector3d omega;
        Eigen::Vector3d v;
        Eigen::VectorXd q(6);
        Eigen::VectorXd qdot(6);

        Eigen::VectorXd scene_states(24);
        scene_states = this->GetInputPort("sim_scene_states").Eval(context);
        _plant.SetPositionsAndVelocities(_plant_context.get(), scene_states);

        // Spatial forces
        // TODO: what does ".template" mean?
        
        const multibody::Frame<T>& floating_base_farme = _plant.GetFrameByName("floating_base");

        // quaternion w.r.t world
        quat = floating_base_farme.CalcRotationMatrixInWorld(*_plant_context).ToQuaternionAsVector4();

        // linear position in world
        pos = floating_base_farme.CalcPoseInWorld(*_plant_context).translation();

        // angular rate in world
        omega = floating_base_farme.CalcAngularVelocity(*_plant_context,
                                                        _plant.world_frame(),
                                                        _plant.world_frame());

        v = floating_base_farme.CalcSpatialVelocityInWorld(*_plant_context).translational();

        // q: x y z, r p y, q1,...q6 
        // q = scene_states.segment<6>(3);
        q = scene_states.segment<6>(6);

        // qdot                   
        qdot = scene_states.segment<6>(18);                 

        // Q: what is the reference frame of floating base system in drake?
        // A: world
        full_states.segment<4>(0) = quat;
        full_states.segment<3>(4) = pos;
        full_states.segment<6>(7) = q;
        full_states.segment<3>(13) = omega;
        full_states.segment<3>(16) = v;
        full_states.segment<6>(19) = qdot;
        
        output->set_value(full_states);
    }

    void CalcLogData(const systems::Context<T>& context,
                           systems::BasicVector<T>* logoutput) const {

        Eigen::VectorXd log_data(18);
        log_data.setZero();
        // Part 1: kinematics data
        Eigen::VectorXd scene_states(18);
        scene_states = this->GetInputPort("sim_scene_states").Eval(context);
        _plant.SetPositionsAndVelocities(_plant_context.get(), scene_states);

        const multibody::Frame<T>& floating_base_farme = _plant.GetFrameByName("floating_base");

        // euler angle
        Eigen::Vector3d euler = math::RollPitchYaw<T>(floating_base_farme.CalcRotationMatrixInWorld(*_plant_context)).vector();;
        
        // linear position in world
        Eigen::Vector3d pos = floating_base_farme.CalcPoseInWorld(*_plant_context).translation();

        // angular rate in world
        Eigen::Vector3d omega = floating_base_farme.CalcAngularVelocity(*_plant_context,
                                                        _plant.world_frame(),
                                                        _plant.world_frame());
        // linear velocity in world
        Eigen::Vector3d linear_vel = floating_base_farme.CalcSpatialVelocityInWorld(*_plant_context).translational();

        log_data.segment<3>(0) = euler;
        log_data.segment<3>(3) = pos;
        log_data.segment<3>(6) = omega;
        log_data.segment<3>(9) = linear_vel;

        // Part 2: forces data
        // spatial_vec include 12x6 wrenches for each joint; including the fixed joints and the 'worldWeld' joint
        const std::vector<drake::multibody::SpatialForce<double>>& spatial_vec =
            this->GetInputPort("spatial_forces_in").template Eval<std::vector<drake::multibody::SpatialForce<double>>>(context);
        Eigen::Matrix<double ,6, 1> wrenches = spatial_vec[11].get_coeffs();
        // std::cout << "spatial forces dimention = " << spatial_vec.size() << std::endl;
        log_data.segment<6>(12) = wrenches;

        logoutput->set_value(log_data);
    }

    void ExtractPosRot(const systems::Context<T>& context,
                           systems::BasicVector<T>* output) const {
        Eigen::VectorXd scene_states(24); // [x, y ,z], [r, p, y], [q1, ..., q6]
        scene_states = this->GetInputPort("sim_scene_states").Eval(context);
        Eigen::VectorXd pos_and_rpy(12); pos_and_rpy.setZero();
        pos_and_rpy.segment<3>(0) = scene_states.segment<3>(0); // [x, y ,z]
        pos_and_rpy.segment<3>(3) = scene_states.segment<3>(3); // [r, p, y]
        pos_and_rpy.segment<3>(6) = scene_states.segment<3>(12);
        pos_and_rpy.segment<3>(9) = scene_states.segment<3>(15);
        
        output->set_value(pos_and_rpy);

    }

};


} // namespace centaur_sim
} // namespace workspace
} // namespace drake
