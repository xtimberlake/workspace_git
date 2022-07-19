/*
 * @Author: haoyun 
 * @Date: 2022-07-19 09:55:16
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-19 10:18:24
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
        this->DeclareVectorInputPort("sim_scene_states", 18);
        this->DeclareVectorOutputPort(
            "full_states", 25,
            &extractData::ExtractFullStates);
        
        
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
        Eigen::VectorXd q(_plant.num_actuators());
        Eigen::VectorXd qdot(_plant.num_actuators());

        Eigen::VectorXd scene_states(18);
        scene_states = this->GetInputPort("sim_scene_states").Eval(context);
        _plant.SetPositionsAndVelocities(_plant_context.get(), scene_states);
        
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

        // q
        q = scene_states.segment<6>(3);

        // qdot                   
        qdot = scene_states.segment<6>(12);                 

        // Q: what is the reference frame of floating base system in drake?
        // A: 
        full_states.segment<4>(0) = quat;
        full_states.segment<3>(4) = pos;
        full_states.segment<6>(7) = q;
        full_states.segment<3>(13) = omega;
        full_states.segment<3>(16) = v;
        full_states.segment<6>(19) = qdot;
        
        output->set_value(full_states);
    }

};


} // namespace centaur_sim
} // namespace workspace
} // namespace drake
