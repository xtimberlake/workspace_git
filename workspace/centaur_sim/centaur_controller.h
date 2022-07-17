/*
 * @Author: haoyun 
 * @Date: 2022-07-14 12:43:34
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-17 21:01:20
 * @FilePath: /drake/workspace/centaur_sim/centaur_controller.h
 * @Description: controller block for drake simulation
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once

#include <iostream>

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/common/find_resource.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/frame.h"
#include <drake/multibody/math/spatial_velocity.h>
#include <drake/math/roll_pitch_yaw.h>

#include "drake/workspace/centaur_sim/controller/CentaurGaitPattern.h"
#include "drake/workspace/centaur_sim/controller/CentaurStates.h"
#include "drake/workspace/centaur_sim/centaurrobot/centaurrobot.h"




class centaurrobot;


namespace drake{
namespace workspace{
namespace centaur_sim{

template<typename T>
class CentaurController : public systems::LeafSystem<T>
{
public:
    CentaurController(const std::string &control_model_file_name)
        : _control_model(0.0) {

        multibody::Parser(&_control_model).AddModelFromFile(
            FindResourceOrThrow(control_model_file_name));
        drake::log()->info("succeed to load " + control_model_file_name + " as control model!");

        _control_model.Finalize();
        _plant_context = _control_model.CreateDefaultContext();

        this->DeclareVectorInputPort("full_states",
                                    _control_model.num_positions() + _control_model.num_velocities());
        this->DeclareVectorOutputPort("actuated_torque", 6,
                                    &CentaurController::CalcTorques);
        
        
    }

    std::unique_ptr<centaurrobot> ct = std::make_unique<centaurrobot>();
    

private:
    multibody::MultibodyPlant<T> _control_model;
    std::unique_ptr<systems::Context<T>> _plant_context;


    void CalcTorques(const systems::Context<T>& context,
                  systems::BasicVector<T>* output) const {

        Eigen::VectorXd output_torques(_control_model.num_actuators()); output_torques.setZero();
        update_states(context);
        
    
        // drake::log()->warn("time = " +  std::to_string(ct->ctrl_states.t));
        // if (ct->ctrl_states.t > 0.8 && ct->ctrl_states.t < 0.9)
        // {
        //     std::cout << ct->ctrl_states.root_euler.transpose() << std::endl;
        // }
        
        if((ct->ctrl_states.t - ct->ctrl_states.k * ct->ctrl_states.control_dt) > ct->ctrl_states.control_dt)
        {
            ct->ctrl_states.k++;
            ct->walking->update_gait_pattern(ct->ctrl_states);
            std::cout << "time: " << ct->ctrl_states.t << ", ";
            std::cout << ct->ctrl_states.plan_contacts_phase(0) << ", ";
            std::cout << ct->ctrl_states.plan_contacts_phase(1) << std::endl;
        }
        
       
        output->set_value(output_torques);
    
    }

    /**
     * @description: update all states from scene plant
     * @return void
     */
    void update_states(const systems::Context<T>& context) const
    {
        Eigen::VectorXd states = this->GetInputPort("full_states").Eval(context);
        _control_model.SetPositionsAndVelocities(_plant_context.get(), states);
        ct->ctrl_states.t = context.get_time();

        const multibody::BodyFrame<T>& FloatingBodyFrame = 
                _control_model.GetBodyByName("floating_base").body_frame();

        //TODO(haoyun) acceleration is actuators- and wrench- dependent states
        // // ct->ctrl_states.root_acc = FloatingBodyFrame.CalcSpatialAccelerationInWorld(*_plant_context).translational(); 

        // kinematics states
        ct->ctrl_states.root_quat = FloatingBodyFrame.CalcRotationMatrixInWorld(*_plant_context).ToQuaternion();
        ct->ctrl_states.root_pos = FloatingBodyFrame.CalcPoseInWorld(*_plant_context).translation();
        ct->ctrl_states.root_ang_vel = FloatingBodyFrame.CalcSpatialVelocityInWorld(*_plant_context).rotational();
        ct->ctrl_states.root_lin_vel = FloatingBodyFrame.CalcSpatialVelocityInWorld(*_plant_context).translational();
        ct->ctrl_states.root_rot_mat = FloatingBodyFrame.CalcRotationMatrixInWorld(*_plant_context).matrix();
        ct->ctrl_states.root_rot_mat_z = FloatingBodyFrame.CalcRotationMatrixInWorld(*_plant_context).matrix().transpose();
        ct->ctrl_states.root_euler = math::RollPitchYaw<T>(FloatingBodyFrame.CalcRotationMatrixInWorld(*_plant_context)).vector();
        
        
        
        
        

    }
};


} // namespace centaur_sim
} // namespace workspace
} // namespace drake