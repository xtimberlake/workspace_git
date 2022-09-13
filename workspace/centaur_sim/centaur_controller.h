/*
 * @Author: haoyun 
 * @Date: 2022-07-14 12:43:34
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-09-13 17:13:44
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
        this->DeclareVectorOutputPort("controller_log_data", 5,
                                    &CentaurController::OutpotLog);
        
        
    }

    std::unique_ptr<centaurrobot> ct = std::make_unique<centaurrobot>();
    

private:
    multibody::MultibodyPlant<T> _control_model;
    std::unique_ptr<systems::Context<T>> _plant_context;


    void CalcTorques(const systems::Context<T>& context,
                  systems::BasicVector<T>* output) const {

        Eigen::VectorXd output_torques(_control_model.num_actuators()); output_torques.setZero();
        update_states(context);
        
        if((ct->ctrl_states.t - ct->ctrl_states.k * ct->ctrl_states.control_dt) > ct->ctrl_states.control_dt)
        {
            ct->ctrl_states.k++;

            if(ct->ctrl_states.t < 0.5) {
                ct->standing->update_gait_pattern(ct->ctrl_states);
            }
            else {
                ct->walking->update_gait_pattern(ct->ctrl_states);
            }

            // drake::log()->info("swing left = " + std::to_string(ct->ctrl_states.plan_swings_phase[0]) + "  right = " +  std::to_string(ct->ctrl_states.plan_swings_phase[1]));
            // drake::log()->info("stance left = " + std::to_string(ct->ctrl_states.plan_contacts_phase[0]) + "  right = " +  std::to_string(ct->ctrl_states.plan_contacts_phase[1]));
             
            ct->controller->GenerateSwingTrajectory(ct->ctrl_states);
            // drake::log()->info(ct->ctrl_states.foot_pos_world.transpose());
            if(ct->ctrl_states.k == 1 ||ct->ctrl_states.k % ct->ctrl_states.nIterationsPerMPC == 0) {
                ct->controller->ComputeGoundReactionForce(ct->ctrl_states);
            }
            output_torques = ct->legcontroller->task_impedance_control(ct->ctrl_states);   
            // drake::log()->info(output_torques.transpose());
            // if(ct->ctrl_states.t < 1.0)
            // {
            //     output_torques.setZero();
            // }
        }
        
       
        output->set_value(output_torques);
    
    }

    void OutpotLog(const systems::Context<T>& context,
                  systems::BasicVector<T>* output) const {

            Eigen::VectorXd output_log_vector(5); output_log_vector.setZero();
            /* 
                0: time
                1: left_contact_state
                2: (left)grf_x
                3: grf_y
                4: grf_z
            */
            output_log_vector[0] = context.get_time();
            output_log_vector[1] = ct->ctrl_states.plan_contacts_phase[0];
            output_log_vector.segment<3>(2) = ct->ctrl_states.foot_force_cmd_rel.block<3, 1>(0, 0);

            output->set_value(output_log_vector);

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
        ct->ctrl_states.root_euler = math::RollPitchYaw<T>(FloatingBodyFrame.CalcRotationMatrixInWorld(*_plant_context)).vector();
        
        double sin_yaw, cos_yaw;
        sin_yaw = sin(ct->ctrl_states.root_euler[2]);
        cos_yaw = cos(ct->ctrl_states.root_euler[2]);

        ct->ctrl_states.root_rot_mat_z << cos_yaw, -sin_yaw, 0,
                                          sin_yaw, cos_yaw, 0,
                                          0, 0, 1;

        const multibody::BodyFrame<T>& LeftFootFrame = _control_model.GetBodyByName("left_foot").body_frame();
        ct->ctrl_states.foot_pos_rel.block<3, 1>(0, 0) = LeftFootFrame.CalcPose(*_plant_context, FloatingBodyFrame).translation();
        const multibody::BodyFrame<T>& RightFootFrame = _control_model.GetBodyByName("right_foot").body_frame();
        ct->ctrl_states.foot_pos_rel.block<3, 1>(0, 1) = RightFootFrame.CalcPose(*_plant_context, FloatingBodyFrame).translation();
    
        ct->ctrl_states.foot_pos_abs.block<3, 1>(0, 0) = ct->ctrl_states.root_rot_mat * ct->ctrl_states.foot_pos_rel.block<3, 1>(0, 0);
        ct->ctrl_states.foot_pos_abs.block<3, 1>(0, 1) = ct->ctrl_states.root_rot_mat * ct->ctrl_states.foot_pos_rel.block<3, 1>(0, 1);

        ct->ctrl_states.foot_pos_world.block<3, 1>(0, 0) = ct->ctrl_states.root_pos + ct->ctrl_states.root_rot_mat * ct->ctrl_states.foot_pos_rel.block<3, 1>(0, 0);
        ct->ctrl_states.foot_pos_world.block<3, 1>(0, 1) = ct->ctrl_states.root_pos + ct->ctrl_states.root_rot_mat * ct->ctrl_states.foot_pos_rel.block<3, 1>(0, 1);
        
        // Eigen::Vector3d temp;
        // temp = LeftFootFrame.CalcPoseInWorld(*_plant_context).translation();
        // drake::log()->info("real:" );
        // drake::log()->info(temp);

        MatrixX<double> J_BF_left(3, _control_model.num_positions());
        _control_model.CalcJacobianTranslationalVelocity(*_plant_context,
                                                         multibody::JacobianWrtVariable::kQDot,
                                                         LeftFootFrame,
                                                         Vector3<double>::Zero(),
                                                         FloatingBodyFrame,
                                                         FloatingBodyFrame,
                                                         &J_BF_left);
                                                        
        ct->ctrl_states.JacobianFoot[0] = J_BF_left.block<3, 3>(0, 7);

        MatrixX<double> J_BF_right(3, _control_model.num_positions());
        _control_model.CalcJacobianTranslationalVelocity(*_plant_context,
                                                         multibody::JacobianWrtVariable::kQDot,
                                                         RightFootFrame,
                                                         Vector3<double>::Zero(),
                                                         FloatingBodyFrame,
                                                         FloatingBodyFrame,
                                                         &J_BF_right);
                                                
        ct->ctrl_states.JacobianFoot[1] = J_BF_right.block<3, 3>(0, 10);
                                                        
 
        Eigen::Matrix<double, 13, 1> qvec;
        qvec = _control_model.GetPositions(*_plant_context);
        ct->ctrl_states.q << qvec.tail(6);

        Eigen::Matrix<double, 12, 1> qdot_vec = _control_model.GetVelocities(*_plant_context);
        ct->ctrl_states.qdot = qdot_vec.segment<6>(6);

        ct->ctrl_states.foot_vel_rel.block<3, 1>(0, 0) = ct->ctrl_states.JacobianFoot[0] * ct->ctrl_states.qdot.segment<3>(0);
        ct->ctrl_states.foot_vel_rel.block<3, 1>(0, 1) = ct->ctrl_states.JacobianFoot[1] * ct->ctrl_states.qdot.segment<3>(3);

        ct->ctrl_states.foot_vel_world.block<3, 1>(0, 0) = ct->ctrl_states.root_rot_mat_z * ct->ctrl_states.foot_vel_rel.block<3, 1>(0, 0);
        ct->ctrl_states.foot_vel_world.block<3, 1>(0, 1) = ct->ctrl_states.root_rot_mat_z * ct->ctrl_states.foot_vel_rel.block<3, 1>(0, 1);
        // if(context.get_time() < 0.001) {
        //     drake::log()->info("ct->ctrl_states.q = ");
        //     drake::log()->info(ct->ctrl_states.q);
        // }
        

    }
};


} // namespace centaur_sim
} // namespace workspace
} // namespace drake