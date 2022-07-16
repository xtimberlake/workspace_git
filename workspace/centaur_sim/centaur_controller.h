/*
 * @Author: haoyun 
 * @Date: 2022-07-14 12:43:34
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-07-16 21:15:46
 * @FilePath: /drake/workspace/centaur_sim/centaur_controller.h
 * @Description: controller block for drake simulation
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/common/find_resource.h"

#include "drake/workspace/centaur_sim/centaurrobot/centaurrobot.h"
#include "drake/workspace/centaur_sim/controller/CentaurControlStates.h"

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
        
        drake::log()->info("the robot gait period is " + std::to_string(ct->ctrl_states.gait_period));
        
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
        math::RigidTransformd X_BF_right = 
            _control_model.GetBodyByName("floating_base").body_frame().CalcPoseInWorld(*_plant_context);
        if(ct->ctrl_states.t < .001)
            drake::log()->info(X_BF_right.translation());
        
        // ct->ctrl_states.root_pos = 
        

    }
};


} // namespace centaur_sim
} // namespace workspace
} // namespace drake