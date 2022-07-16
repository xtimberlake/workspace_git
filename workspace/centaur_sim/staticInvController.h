/*
 * @Author: haoyun 
 * @Date: 2022-07-11 10:09:07
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-16 20:12:30
 * @FilePath: /A1-QP-MPC-Controller/home/haoyun/Data/Code/drake/workspace/centaur_sim/staticInvController.h
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake{
namespace workspace{
namespace centaur_sim{

template <typename T>
class staticInvController : public drake::systems::LeafSystem<T> 
{
private:
    /* data */
public:
    staticInvController(const multibody::MultibodyPlant<T>& plant) 
        : plant_(plant){
    
        this->DeclareVectorInputPort("desired_position", 6);
        this->DeclareVectorInputPort("states", 12);
        this->DeclareVectorOutputPort(
            "motor_torques", 6,
            &staticInvController::CalcJacobianOutput);

        plant_context_ = plant_.CreateDefaultContext();

    }

private:
    const multibody::MultibodyPlant<T>& plant_;
    std::unique_ptr<systems::Context<T>> plant_context_;


    void CalcJacobianOutput(const systems::Context<T>& context,
                  systems::BasicVector<T>* output) const {

    Vector6d computed_torques;
    computed_torques.setZero();

    VectorX<double> states;
    
    Vector6d pos_des, pos_cur, pos_diff;
    pos_des = this->GetInputPort("desired_position").Eval(context);
    states = this->GetInputPort("states").Eval(context);

    plant_.SetPositionsAndVelocities(plant_context_.get(), states);
    

    math::RigidTransformd X_BF_left = 
        plant_.GetBodyByName("hind_left_foot").body_frame().CalcPose(
            *plant_context_,
            plant_.GetBodyByName("floating_base").body_frame());
    
    math::RigidTransformd X_BF_right = 
        plant_.GetBodyByName("hind_right_foot").body_frame().CalcPose(
            *plant_context_,
            plant_.GetBodyByName("floating_base").body_frame());

    pos_cur.head(3) = X_BF_left.translation();
    pos_cur.tail(3) = X_BF_right.translation();
    pos_diff = pos_des - pos_cur;


    MatrixX<double> J_BF_left(3, 6), J_BF_right(3, 6);
    plant_.CalcJacobianTranslationalVelocity(*plant_context_,
                                                multibody::JacobianWrtVariable::kQDot,
                                                plant_.GetBodyByName("hind_left_foot").body_frame(),
                                                Vector3<double>::Zero(),
                                                plant_.GetBodyByName("floating_base").body_frame(),
                                                plant_.GetBodyByName("floating_base").body_frame(),
                                                &J_BF_left);

    plant_.CalcJacobianTranslationalVelocity(*plant_context_,
                                                multibody::JacobianWrtVariable::kQDot,
                                                plant_.GetBodyByName("hind_right_foot").body_frame(),
                                                Vector3<double>::Zero(),
                                                plant_.GetBodyByName("floating_base").body_frame(),
                                                plant_.GetBodyByName("floating_base").body_frame(),
                                                &J_BF_right);

    // // tao = J^T * k * (pos_diff)         
    computed_torques.head(3) = J_BF_left.block<3, 3>(0, 0).transpose() * 1000 * pos_diff.head(3);
    computed_torques.tail(3) = J_BF_right.block<3, 3>(0, 3).transpose() * 1000 * pos_diff.tail(3);
    
    output->set_value(computed_torques);

    }

};



} // namespace centaur_sim
} // namespace workspace
} // namespace drake