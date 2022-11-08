/*
 * @Author: haoyun 
 * @Date: 2022-07-14 12:43:34
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-11-08 15:26:25
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
#include "drake/workspace/centaur_sim/controller/Task.hpp"
#include "drake/workspace/centaur_sim/controller/WBIController.h"




class centaurrobot;

class WBIController;


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

        this->DeclareVectorInputPort("position_rotation",
                                    12);

        this->DeclareVectorInputPort("force_sensors_output",
                                    12);

        this->DeclareVectorOutputPort("actuated_torque", 9,
                                    &CentaurController::CalcTorques);
        this->DeclareVectorOutputPort("controller_log_data", 8,
                                    &CentaurController::OutpotLog);
        
        
    }

    std::unique_ptr<centaurrobot> ct = std::make_unique<centaurrobot>();
    

private:
    multibody::MultibodyPlant<T> _control_model;
    std::unique_ptr<systems::Context<T>> _plant_context;
    

    void CalcTorques(const systems::Context<T>& context,
                  systems::BasicVector<T>* output) const {

            
        Eigen::VectorXd total_torques(3 + _control_model.num_actuators()); total_torques.setZero();
        static Eigen::Matrix<double, 6, 1> output_torques = Eigen::Matrix<double, -1, 1>::Zero(6);
        // output_torques.setZero();
        update_states(context);

        Eigen::VectorXd pos_rpy_states = this->GetInputPort("position_rotation").Eval(context);
        Eigen::Vector3d prismatic_joint_q; prismatic_joint_q.setZero();
        Eigen::Vector3d prismatic_joint_qdot; prismatic_joint_qdot.setZero();
        static Eigen::Vector3d prismatic_joint_q_des = Eigen::Vector3d::Zero();
        Eigen::Vector3d prismatic_joint_qdot_des; prismatic_joint_qdot_des.setZero();

        prismatic_joint_q = pos_rpy_states.head(3);
        prismatic_joint_qdot = pos_rpy_states.segment<3>(6);

        double v_des, v_now, delta_x, buffTime;
        // static int sign = 1;
        delta_x = 0;
        v_now = 0.0;
        v_des = 0.6;
        buffTime = 2.0; // secs
        
        if(ct->ctrl_states.t > 0.1) {

            if(ct->ctrl_states.t < buffTime) {
                // v_now = v_des * sin(M_PI_2 * ct->ctrl_states.t / buffTime);
                v_now = 0;
                // v_now += v_des/buffTime;
            } 
            else if(ct->ctrl_states.t < 2 * buffTime)
            {
                double phase = ct->ctrl_states.t - buffTime;
                v_now = v_des * sin(M_PI_2 * phase / buffTime);
            }
            else
            {
                v_now = v_des;
            }
            // else if(ct->ctrl_states.t < 3 * buffTime)
            // {
            //     double phase = ct->ctrl_states.t - 2*buffTime;
            //     v_now = -v_des * sin(M_PI_2 * phase / buffTime);
            // }
            // else if(ct->ctrl_states.t < 4 * buffTime)
            // {
            //     double phase = ct->ctrl_states.t - 3*buffTime;
            //     v_now = -v_des + v_des * sin(M_PI_2 * phase / buffTime);
            // }

           

        }
        delta_x = v_now * 5e-4;

        prismatic_joint_q_des[0] += delta_x;

        // position
        ct->ctrl_states.root_pos_d[0] = prismatic_joint_q_des[0];
        ct->ctrl_states.root_pos_d[1] = prismatic_joint_q_des[1];

        // velocity
        ct->ctrl_states.root_lin_vel_d_world[0] = v_now / 4.0;


        total_torques.head(3) = 100000 * (prismatic_joint_q_des - pos_rpy_states) 
                                + 20000 * (prismatic_joint_qdot_des - prismatic_joint_qdot);

        
        if((ct->ctrl_states.t - ct->ctrl_states.k * ct->ctrl_states.control_dt) > ct->ctrl_states.control_dt)
        {   
            // running at 1 kHz
            ct->ctrl_states.k++;

            if(ct->ctrl_states.t < 0.1) { // start trotting in 0.1 seconds
                ct->standing->update_gait_pattern(ct->ctrl_states);
                ct->ctrl_states.firstRun = true;
            }
            else {
                ct->walking->update_gait_pattern(ct->ctrl_states);
                // ct->jumping->update_gait_pattern(ct->ctrl_states);
                // ct->fly_trotting->update_gait_pattern(ct->ctrl_states);
                
            }
            Eigen::Vector2d contact_phases;
            Eigen::Vector2d swing_phases;
            contact_phases << ct->ctrl_states.plan_contacts_phase[0], ct->ctrl_states.plan_contacts_phase[1];
            swing_phases << ct->ctrl_states.plan_swings_phase[0], ct->ctrl_states.plan_swings_phase[1];
            
            ct->contactestimate->updateEstimate(contact_phases, swing_phases,
                                            ct->ctrl_states.foot_pos_world, ct->ctrl_states.foot_force_world);

            // ct->contactestimate->updateEstimate(ct->ctrl_states.plan_contacts_phase, ct->ctrl_states.plan_swings_phase,
            //                                 ct->ctrl_states.foot_pos_world, ct->ctrl_states.foot_force_world);

            // swing
            ct->controller->GenerateSwingTrajectory(ct->ctrl_states);

            // compute mpc grfs at 100 Hz
            if(ct->ctrl_states.k == 1 ||ct->ctrl_states.k % ct->ctrl_states.nIterationsPerMPC == 0) {
                ct->controller->ComputeGoundReactionForce(ct->ctrl_states);
            }

            // whole-body impluse controller
            ct->wbicontroller->run(ct->ctrl_states);

            if(ct->ctrl_states.t < 0.1) { // start trotting in 0.5 seconds
                // output_torques = ct->legcontroller->task_impedance_control(ct->ctrl_states);
                output_torques = ct->legcontroller->wbc_low_level_control(ct->ctrl_states);
            }
            else {
                // output_torques = ct->legcontroller->task_impedance_control(ct->ctrl_states);
                output_torques = ct->legcontroller->wbc_low_level_control(ct->ctrl_states);
            }
            

        }

        total_torques.tail(6) = output_torques;

        // std::cout << "total_torques = " << total_torques.transpose() << std::endl;
       
        output->set_value(total_torques);
    
    }

    void OutpotLog(const systems::Context<T>& context,
                  systems::BasicVector<T>* output) const {

            Eigen::VectorXd output_log_vector(8); output_log_vector.setZero();
            // /* 
            //     0: time
            //     1: left_contact_state
            //     2: (left)grf_x
            //     3: grf_y
            //     4: grf_z
            // */
            // output_log_vector[0] = context.get_time();
            // output_log_vector[1] = ct->ctrl_states.plan_contacts_phase[0];
            // output_log_vector.segment<3>(2) = ct->ctrl_states.foot_force_cmd_rel.block<3, 1>(0, 0);

            /* 
                0: time
                1: left_contact_state
                2: (left)grf_x
                3: grf_y
                4: grf_z
            */
           double s_phi, s_phi_bar;
            if(ct->ctrl_states.plan_contacts_phase[0] > 0)  {
                s_phi = 1.0;
                s_phi_bar = 0.0;
            }
            else {
                s_phi = 0.0;
                s_phi_bar = 1.0;
            }
            double lambda[2] = {0.0, 1.0};
            double sigma2[2] = {0.025, 0.025};
            double prob;
            
            prob = 0.5 * (s_phi * (std::erf( (ct->ctrl_states.plan_contacts_phase[0]-lambda[0]) / (std::sqrt(2)*sigma2[0]) )
                                 + std::erf( (lambda[1]-ct->ctrl_states.plan_contacts_phase[0]) / (std::sqrt(2)*sigma2[1]) ))
                        + s_phi_bar * (2 + std::erf( (lambda[0]-ct->ctrl_states.plan_swings_phase[0]) / (std::sqrt(2)*sigma2[0]) )
                                         + std::erf( (ct->ctrl_states.plan_swings_phase[0]-lambda[1]) / (std::sqrt(2)*sigma2[1]) )));


            output_log_vector[0] = context.get_time();

            // output_log_vector[0] = ct->ctrl_states.root_pos(2);
            output_log_vector[0] = prob;
            output_log_vector[1] = ct->ctrl_states.plan_contacts_phase[0];

            output_log_vector.segment<3>(2) = -ct->ctrl_states.foot_force_world.block<3, 1>(0, 1);
            output_log_vector.segment<3>(5) = ct->ctrl_states.foot_force_cmd_world.block<3, 1>(0, 1);

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

        Eigen::VectorXd force_sensor_data = this->GetInputPort("force_sensors_output").Eval(context);

        ct->ctrl_states.tau_feedback = force_sensor_data.head(6); 
        ct->ctrl_states.hri_wrench_realtime = force_sensor_data.tail(6); 

        //TODO(haoyun) acceleration is actuators- and wrench- dependent states
        // // ct->ctrl_states.root_acc = FloatingBodyFrame.CalcSpatialAccelerationInWorld(*_plant_context).translational(); 

        // kinematics states
        ct->ctrl_states.root_quat = FloatingBodyFrame.CalcRotationMatrixInWorld(*_plant_context).ToQuaternion();
        ct->ctrl_states.root_pos = FloatingBodyFrame.CalcPoseInWorld(*_plant_context).translation();
        ct->ctrl_states.root_ang_vel_world = FloatingBodyFrame.CalcSpatialVelocityInWorld(*_plant_context).rotational();
        ct->ctrl_states.root_lin_vel_world = FloatingBodyFrame.CalcSpatialVelocityInWorld(*_plant_context).translational();
        ct->ctrl_states.root_rot_mat = FloatingBodyFrame.CalcRotationMatrixInWorld(*_plant_context).matrix();
        ct->ctrl_states.root_ang_vel_rel = ct->ctrl_states.root_rot_mat.transpose() * ct->ctrl_states.root_ang_vel_world;
        ct->ctrl_states.root_lin_vel_rel = ct->ctrl_states.root_rot_mat.transpose() * ct->ctrl_states.root_lin_vel_world;
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

        MatrixX<double> J_BF_left(3, _control_model.num_velocities());
        _control_model.CalcJacobianTranslationalVelocity(*_plant_context,
                                                         multibody::JacobianWrtVariable::kV,
                                                         LeftFootFrame,
                                                         Vector3<double>::Zero(),
                                                         _control_model.world_frame(),
                                                         _control_model.world_frame(),
                                                         &J_BF_left);

        // the floating base qdot is expressed in the base coodinates
        J_BF_left.block<3, 3>(0, 0) = J_BF_left.block<3, 3>(0, 0) * ct->ctrl_states.root_rot_mat;
        J_BF_left.block<3, 3>(0, 3) = J_BF_left.block<3, 3>(0, 3) * ct->ctrl_states.root_rot_mat;
        // ct->ctrl_states.JacobianFoot[0] = J_BF_left.block<3, 3>(0, 6);
        // std::cout << "ground true = " << std::endl << J_BF_left << std::endl;
        
        // // expressed in the body frame                 
        ct->ctrl_states.JacobianFoot[0] = ct->ctrl_states.root_rot_mat.transpose() * J_BF_left.block<3, 3>(0, 6);

        ct->ctrl_states.foot_force_rel.block<3, 1>(0, 0) = ct->ctrl_states.JacobianFoot[0].transpose().inverse() * ct->ctrl_states.tau_feedback.head(3);
        ct->ctrl_states.foot_force_world.block<3, 1>(0, 0) = ct->ctrl_states.root_rot_mat * ct->ctrl_states.foot_force_rel.block<3, 1>(0, 0);

        MatrixX<double> J_BF_right(3, _control_model.num_velocities());
        _control_model.CalcJacobianTranslationalVelocity(*_plant_context,
                                                         multibody::JacobianWrtVariable::kV,
                                                         RightFootFrame,
                                                         Vector3<double>::Zero(),
                                                         _control_model.world_frame(),
                                                         _control_model.world_frame(),
                                                         &J_BF_right);

        J_BF_right.block<3, 3>(0, 0) = J_BF_right.block<3, 3>(0, 0) * ct->ctrl_states.root_rot_mat;
        J_BF_right.block<3, 3>(0, 3) = J_BF_right.block<3, 3>(0, 3) * ct->ctrl_states.root_rot_mat;
        // ct->ctrl_states.JacobianFoot[1] = J_BF_right.block<3, 3>(0, 9);
        ct->ctrl_states.JacobianFoot[1] = ct->ctrl_states.root_rot_mat.transpose() * J_BF_right.block<3, 3>(0, 9);
                                                        
        ct->ctrl_states.foot_force_rel.block<3, 1>(0, 1) = ct->ctrl_states.JacobianFoot[1].transpose().inverse() * ct->ctrl_states.tau_feedback.tail(3);
        ct->ctrl_states.foot_force_world.block<3, 1>(0, 1) = ct->ctrl_states.root_rot_mat * ct->ctrl_states.foot_force_rel.block<3, 1>(0, 1);

        Eigen::Matrix<double, 13, 1> qvec;
        qvec = _control_model.GetPositions(*_plant_context);
        ct->ctrl_states.q << qvec.tail(6); // joint angle

        Eigen::Matrix<double, 12, 1> qdot_vec = _control_model.GetVelocities(*_plant_context);
        ct->ctrl_states.qdot = qdot_vec.segment<6>(6); // joint velocity

        ct->ctrl_states.foot_vel_rel.block<3, 1>(0, 0) = ct->ctrl_states.JacobianFoot[0] * ct->ctrl_states.qdot.segment<3>(0);
        ct->ctrl_states.foot_vel_rel.block<3, 1>(0, 1) = ct->ctrl_states.JacobianFoot[1] * ct->ctrl_states.qdot.segment<3>(3);

        // TODOs: how to 
        ct->ctrl_states.foot_vel_world.block<3, 1>(0, 0) = ct->ctrl_states.root_rot_mat * ct->ctrl_states.foot_vel_rel.block<3, 1>(0, 0);
        ct->ctrl_states.foot_vel_world.block<3, 1>(0, 1) = ct->ctrl_states.root_rot_mat * ct->ctrl_states.foot_vel_rel.block<3, 1>(0, 1);

        // ct->ctrl_states.foot_vel_world.block<3, 1>(0, 0) = LeftFootFrame.CalcSpatialVelocityInWorld(*_plant_context).translational();
        // ct->ctrl_states.foot_vel_world.block<3, 1>(0, 1) = RightFootFrame.CalcSpatialVelocityInWorld(*_plant_context).translational();

        // ct->ctrl_states.foot_vel_rel.block<3, 1>(0, 0) = ct->ctrl_states.root_rot_mat.transpose() * ct->ctrl_states.foot_vel_world.block<3, 1>(0, 0);
        // ct->ctrl_states.foot_vel_rel.block<3, 1>(0, 1) = ct->ctrl_states.root_rot_mat.transpose() * ct->ctrl_states.foot_vel_world.block<3, 1>(0, 1);
        


        // if(context.get_time() < 0.001) {
        //     drake::log()->info("ct->ctrl_states.q = ");
        //     drake::log()->info(ct->ctrl_states.q);
        // }

        // systems dynamics matrix
        _control_model.CalcMassMatrix(*_plant_context, &ct->ctrl_states.Mq);
        _control_model.CalcBiasTerm(*_plant_context, &ct->ctrl_states.Cv);
        ct->ctrl_states.tau_g = _control_model.CalcGravityGeneralizedForces(*_plant_context);
    
        
    }
};


} // namespace centaur_sim
} // namespace workspace
} // namespace drake