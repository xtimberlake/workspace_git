/*
 * @Author: haoyun 
 * @Date: 2022-07-14 12:43:34
 * @LastEditors: haoyun-x13 pioneeroppenheimer@163.com
 * @LastEditTime: 2023-06-28 15:50:23
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
#include <drake/common/yaml/yaml_io.h>

#include "drake/workspace/centaur_sim/controller/CentaurGaitPattern.h"
#include "drake/workspace/centaur_sim/controller/CentaurStates.h"
#include "drake/workspace/centaur_sim/centaurrobot/centaurrobot.h"
#include "drake/workspace/centaur_sim/controller/Task.hpp"
#include "drake/workspace/centaur_sim/controller/WBIController.h"
#include "drake/workspace/centaur_sim/controller/global_control_flag.h"




class centaurrobot;

class WBIController;


namespace drake{
namespace workspace{
namespace centaur_sim{

    struct record_states_struct {
        std::vector<double> time_stamp;
        std::vector<double> roll_ref, pitch_ref, yaw_ref, height_ref;
        std::vector<double> roll, pitch, yaw, height;
        std::vector<double> px, py;
        std::vector<double> mx, my, mz, fx, fy, fz;
        std::vector<double> gait_cycle;
        std::vector<double> left_foot_force, right_foot_force;
        std::vector<double> human_px, human_py, human_pz;
        


        template <typename Archive>
        void Serialize(Archive* a) {
        a->Visit(DRAKE_NVP(time_stamp));
        a->Visit(DRAKE_NVP(roll_ref)); a->Visit(DRAKE_NVP(pitch_ref)); a->Visit(DRAKE_NVP(yaw_ref)); a->Visit(DRAKE_NVP(height_ref)); 
        a->Visit(DRAKE_NVP(roll)); a->Visit(DRAKE_NVP(pitch)); a->Visit(DRAKE_NVP(yaw)); a->Visit(DRAKE_NVP(height)); 
        a->Visit(DRAKE_NVP(px)); a->Visit(DRAKE_NVP(py));
        a->Visit(DRAKE_NVP(mx)); a->Visit(DRAKE_NVP(my)); a->Visit(DRAKE_NVP(mz)); a->Visit(DRAKE_NVP(fx)); a->Visit(DRAKE_NVP(fy)); a->Visit(DRAKE_NVP(fz)); 
        a->Visit(DRAKE_NVP(gait_cycle));
        a->Visit(DRAKE_NVP(left_foot_force)); a->Visit(DRAKE_NVP(right_foot_force));
        a->Visit(DRAKE_NVP(human_px)); a->Visit(DRAKE_NVP(human_py)); a->Visit(DRAKE_NVP(human_pz)); 
        }
    };



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
                                    18);

        this->DeclareVectorOutputPort("actuated_torque", 12,
                                    &CentaurController::CalcTorques);
        // if use passive ball joint
        // this->DeclareVectorOutputPort("actuated_torque", 9,
        //                             &CentaurController::CalcTorques);

        this->DeclareVectorOutputPort("controller_log_data", 8,
                                    &CentaurController::OutpotLog);
        
        
    }

    std::unique_ptr<centaurrobot> ct = std::make_unique<centaurrobot>();
    

private:
    multibody::MultibodyPlant<T> _control_model;
    std::unique_ptr<systems::Context<T>> _plant_context;
    

    void CalcTorques(const systems::Context<T>& context,
                  systems::BasicVector<T>* output) const {

        // if use passive ball joint    
        // static Eigen::VectorXd total_torques(3 + _control_model.num_actuators()); 
        
        static Eigen::VectorXd total_torques(6 + _control_model.num_actuators()); 
        // total_torques.setZero();
        static Eigen::Matrix<double, 6, 1> output_torques;
        // output_torques.setZero();
        update_states(context);

        // Eigen::VectorXd pos_rpy_states = this->GetInputPort("position_rotation").Eval(context);
        // Eigen::Vector3d prismatic_joint_q; prismatic_joint_q.setZero();
        // Eigen::Vector3d prismatic_joint_qdot; prismatic_joint_qdot.setZero();
        // static Eigen::Vector3d prismatic_joint_q_des = Eigen::Vector3d::Zero();
        // Eigen::Vector3d prismatic_joint_qdot_des; prismatic_joint_qdot_des.setZero();

        // prismatic_joint_q = pos_rpy_states.head(3);
        // prismatic_joint_qdot = pos_rpy_states.segment<3>(6);

        // double v_des, v_now, delta_x, buffTime;
        // // static int sign = 1;
        // delta_x = 0;
        // v_now = 0.0;
        // v_des = 0.0;
        // buffTime = 2.0; // secs
        
        // if(ct->ctrl_states.t > 0.1) {

        //     if(ct->ctrl_states.t < buffTime) {
        //         // v_now = v_des * sin(M_PI_2 * ct->ctrl_states.t / buffTime);
        //         v_now = 0;
        //         // v_now += v_des/buffTime;
        //     } 
        //     else if(ct->ctrl_states.t < 2 * buffTime)
        //     {
        //         double phase = ct->ctrl_states.t - buffTime;
        //         v_now = v_des * sin(M_PI_2 * phase / buffTime);
        //     }
        //     else
        //     {
        //         v_now = v_des;
        //     }
        //     // else if(ct->ctrl_states.t < 3 * buffTime)
        //     // {
        //     //     double phase = ct->ctrl_states.t - 2*buffTime;
        //     //     v_now = -v_des * sin(M_PI_2 * phase / buffTime);
        //     // }
        //     // else if(ct->ctrl_states.t < 4 * buffTime)
        //     // {
        //     //     double phase = ct->ctrl_states.t - 3*buffTime;
        //     //     v_now = -v_des + v_des * sin(M_PI_2 * phase / buffTime);
        //     // }

           

        // }
        // delta_x = v_now * 5e-4;

        // prismatic_joint_q_des[0] += delta_x;

        // // position
        // ct->ctrl_states.root_pos_d[0] = prismatic_joint_q_des[0];
        // ct->ctrl_states.root_pos_d[1] = prismatic_joint_q_des[1];

        // // velocity
        // // ct->ctrl_states.root_lin_vel_d_world[0] = v_now / 4.0 + 0.2;

        // // 500, 300 for comopliance contact
        // total_torques[0] = 5000 * (prismatic_joint_q_des[0] - pos_rpy_states[0]) 
        //                         + 3000 * (prismatic_joint_qdot_des[0] - prismatic_joint_qdot[0]);

        // total_torques[1] = 5000 * (prismatic_joint_q_des[1] - pos_rpy_states[1]) 
        //                         + 3000 * (prismatic_joint_qdot_des[1] - prismatic_joint_qdot[1]);
        // // z
        // total_torques[2] = 10000 * (prismatic_joint_q_des[2] - pos_rpy_states[2]) 
        //                         + 5000 * (prismatic_joint_qdot_des[2] - prismatic_joint_qdot[2]);

        
        if((ct->ctrl_states.t - ct->ctrl_states.k * ct->ctrl_states.control_dt) > ct->ctrl_states.control_dt)
        {   
            // running at 1 kHz
            ct->ctrl_states.k++;


            ct->controller->CalcHRITorques(ct->ctrl_states);
            ct->controller->UpdateDesiredStates(ct->ctrl_states);
            total_torques.segment<3>(0) = ct->ctrl_states.hri_actuated_torques.head(3);
            // if use passive ball joint
            total_torques.segment<3>(3) = ct->ctrl_states.hri_actuated_torques.tail(3);

            if(ct->ctrl_states.t < 0.1) { // start trotting in 0.1 seconds
                ct->standing->update_gait_pattern(ct->ctrl_states);
                ct->ctrl_states.firstRun = true;
            }
            else {
                ct->walking->update_gait_pattern(ct->ctrl_states);
                // ct->standing->update_gait_pattern(ct->ctrl_states);
                // ct->jumping->update_gait_pattern(ct->ctrl_states);
                // ct->fly_trotting->update_gait_pattern(ct->ctrl_states);
                
            }
            
            // ct->controller->EstHRIForces(ct->ctrl_states);
            // swing
            ct->controller->GenerateSwingTrajectory(ct->ctrl_states);

            // compute mpc grfs at 100 Hz
            if(ct->ctrl_states.k == 1 ||ct->ctrl_states.k % ct->ctrl_states.nIterationsPerMPC == 0) {
                ct->controller->ComputeGoundReactionForce(ct->ctrl_states);
            }

            // whole-body impluse controller
            ct->wbicontroller->run(ct->ctrl_states);
            
            // if(ct->ctrl_states.t < 0.1) { // start trotting in 0.1 seconds
            //     // output_torques = ct->legcontroller->task_impedance_control(ct->ctrl_states);
            //     output_torques = ct->legcontroller->wbc_low_level_control(ct->ctrl_states);
            // }
            // else {
            //     // output_torques = ct->legcontroller->task_impedance_control(ct->ctrl_states);
            //     output_torques = ct->legcontroller->wbc_low_level_control(ct->ctrl_states);
            // }

            output_torques = ct->legcontroller->wbc_low_level_control(ct->ctrl_states);
            double random_scale[6] = {10, 20, 20, 10, 20, 20};
            // std::cout << "enable actuators' domain randomization: ";
            for (int i = 0; i < 6; i++)
            {
                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_real_distribution<> distrib_(-1, 1);
                output_torques[i] += random_scale[i] * distrib_(gen);
            }
            
            
            // output_torques = ct->legcontroller->wbc_feedforward_ik_control(ct->ctrl_states);
            // ct->legcontroller->debug_ik(ct->ctrl_states);
            

            // ct->controller->InverseKinematics(ct->ctrl_states);
            // output_torques = ct->legcontroller->joint_impedance_control(ct->ctrl_states);


            #ifdef USE_REACTIVE_CONTROL
            ct->contactestimate->updateMeasurement(ct->ctrl_states);
            ct->contactestimate->updateEstimate();
            ct->contactestimate->eventsDetect();
            ct->contactestimate->publishStates(ct->ctrl_states);
            #endif
            // ct->contactestimate->getContactProbabilities(ct->ctrl_states.prob_contact);
            // ct->contactestimate->getContactProbabilitiesBasedonPlan(ct->ctrl_states.prob_contact_of_plan);
            // ct->contactestimate->getContactProbabilitiesBasedonPos(ct->ctrl_states.prob_contact_of_pos);
            // ct->contactestimate->getContactProbabilitiesBasedonForce(ct->ctrl_states.prob_contact_of_force);
            // ct->contactestimate->getContactProbabilitiesBasedonVelocity(ct->ctrl_states.prob_contact_of_velocity);
            // ct->contactestimate->getEstimatedContactForce(ct->ctrl_states.foot_force_est_world);
            

        }

        total_torques.tail(6) = output_torques;

        
       
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
        //    double s_phi, s_phi_bar;
        //     if(ct->ctrl_states.plan_contacts_phase[0] > 0)  {
        //         s_phi = 1.0;
        //         s_phi_bar = 0.0;
        //     }
        //     else {
        //         s_phi = 0.0;
        //         s_phi_bar = 1.0;
        //     }
        //     double lambda[2] = {0.0, 1.0};
        //     double sigma2[2] = {0.025, 0.025};
            // double prob;
            
            // prob = 0.5 * (s_phi * (std::erf( (ct->ctrl_states.plan_contacts_phase[0]-lambda[0]) / (std::sqrt(2)*sigma2[0]) )
            //                      + std::erf( (lambda[1]-ct->ctrl_states.plan_contacts_phase[0]) / (std::sqrt(2)*sigma2[1]) ))
            //             + s_phi_bar * (2 + std::erf( (lambda[0]-ct->ctrl_states.plan_swings_phase[0]) / (std::sqrt(2)*sigma2[0]) )
            //                              + std::erf( (ct->ctrl_states.plan_swings_phase[0]-lambda[1]) / (std::sqrt(2)*sigma2[1]) )));


            output_log_vector[0] = context.get_time();

            // // joint angle
            // output_log_vector[0] = ct->ctrl_states.wbc_q_cmd(2);
            // output_log_vector[1] = ct->ctrl_states.q(2);

            // output_log_vector[2] = ct->ctrl_states.wbc_qdot_cmd(2);
            // output_log_vector[3] = ct->ctrl_states.qdot(2);

            // force
            // output_log_vector[0] = ct->ctrl_states.wbc_tau_ff(0);
            // output_log_vector[1] = ct->ctrl_states.wbc_tau_ff(0);

            // output_log_vector[2] = ct->ctrl_states.wbc_tau_ff(1);
            // output_log_vector[3] = ct->ctrl_states.wbc_tau_ff(2);

            // output_log_vector[4] = -ct->ctrl_states.foot_force_simulation(2, 0);
            // output_log_vector[5] = ct->contactestimate->_foot_force_hat(2, 0);

            // // foot pos
            // output_log_vector[0] = ct->ctrl_states.foot_pos_cmd_world(0, 0);
            // output_log_vector[1] = ct->ctrl_states.foot_pos_world(0, 0);

            // output_log_vector[2] = ct->ctrl_states.foot_pos_cmd_world(1, 0);
            // output_log_vector[3] = ct->ctrl_states.foot_pos_world(1, 0);

            // output_log_vector[4] = ct->ctrl_states.foot_pos_cmd_world(2,0);
            // output_log_vector[5] = ct->ctrl_states.foot_pos_world(2,0);

            // output_log_vector[2] = ct->ctrl_states.foot_vel_world(0, 0);
            // output_log_vector[3] = ct->ctrl_states.foot_vel_cmd_world(0, 0);


        
            // output_log_vector[2] = ct->ctrl_states.root_pos_d(2);
            // output_log_vector[3] = ct->ctrl_states.root_pos(2);
       
            //  output_log_vector[4] = ct->ctrl_states.theta_opt;
            // output_log_vector[5] = ct->ctrl_states.root_euler(1);

            // output_log_vector[6] = ct->ctrl_states.foot_pos_cmd_world(2,0);
            // output_log_vector[7] = ct->ctrl_states.foot_pos_world(2,0);
            
            // output_log_vector[0] = ct->ctrl_states.root_pos_d[2];
            // output_log_vector[1] = ct->ctrl_states.root_pos[2];

            // output_log_vector[2] = ct->ctrl_states.root_euler_d(1);
            // output_log_vector[3] = ct->ctrl_states.root_euler(1);

            // output_log_vector[4] = ct->ctrl_states.root_euler_d(1);

            // output_log_vector[6] = ct->contactestimate->_foot_force_hat(2, 0);
            // output_log_vector[7] = ct->contactestimate->_foot_force_hat(2, 1);

            // output_log_vector[0] = ct->ctrl_states.ik_q_and_qdot_cmd(0);
            // output_log_vector[1] = ct->ctrl_states.q(0);
            // output_log_vector[2] = ct->ctrl_states.ik_q_and_qdot_cmd(1);
            // output_log_vector[3] = ct->ctrl_states.q(1);
            // output_log_vector[4] = ct->ctrl_states.ik_q_and_qdot_cmd(2);
            // output_log_vector[5] = ct->ctrl_states.q(2);

            // output_log_vector[6] = ct->ctrl_states.plan_contacts_phase(0);
            // output_log_vector[5] = ct->ctrl_states.q(2);
  


            // output_log_vector[4] = ct->contactestimate->footAcc(2,0);
            // output_log_vector[5] = ct->contactestimate->footAcc(2,1);


            // debug ik
            // output_log_vector[0] = ct->ctrl_states.debug_q_now[6];
            // output_log_vector[1] = ct->ctrl_states.debug_q_ik_result[6];
            // output_log_vector[2] = ct->ctrl_states.debug_q_now[7];
            // output_log_vector[3] = ct->ctrl_states.debug_q_ik_result[7];
            // output_log_vector[4] = ct->ctrl_states.debug_q_now[8];
            // output_log_vector[5] = ct->ctrl_states.debug_q_ik_result[8];


            // output_log_vector.head(6) = ct->ctrl_states.hri_wrench_realtime;

            output_log_vector.segment<3>(0) = ct->ctrl_states.foot_force_world.col(0);
            output_log_vector.segment<3>(3) = ct->ctrl_states.foot_force_cmd_world.col(1);
            // output_log_vector.segment<3>(0) = ct->ctrl_states.root_lin_vel_world;

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
        ct->ctrl_states.hri_wrench_realtime = force_sensor_data.segment<6>(6); 
        ct->ctrl_states.foot_force_simulation.block<3, 1>(0, 0) = force_sensor_data.segment<3>(12); 
        ct->ctrl_states.foot_force_simulation.block<3, 1>(0, 1) = force_sensor_data.segment<3>(15); 

        //TODO(haoyun) acceleration is actuators- and wrench- dependent states
        // // ct->ctrl_states.root_acc = FloatingBodyFrame.CalcSpatialAccelerationInWorld(*_plant_context).translational(); 

        // kinematics states
        ct->ctrl_states.root_quat = FloatingBodyFrame.CalcRotationMatrixInWorld(*_plant_context).ToQuaternion();
        ct->ctrl_states.root_pos = FloatingBodyFrame.CalcPoseInWorld(*_plant_context).translation();
        ct->ctrl_states.root_ang_vel_world = FloatingBodyFrame.CalcSpatialVelocityInWorld(*_plant_context).rotational();
        ct->ctrl_states.root_lin_vel_world = FloatingBodyFrame.CalcSpatialVelocityInWorld(*_plant_context).translational();
        ct->ctrl_states.root_rot_mat = FloatingBodyFrame.CalcRotationMatrixInWorld(*_plant_context).matrix(); // from body to world
        ct->ctrl_states.root_ang_vel_rel = ct->ctrl_states.root_rot_mat.transpose() * ct->ctrl_states.root_ang_vel_world;
        ct->ctrl_states.root_lin_vel_rel = ct->ctrl_states.root_rot_mat.transpose() * ct->ctrl_states.root_lin_vel_world;

        ct->ctrl_states.Hri_pos = ct->ctrl_states.root_pos + ct->ctrl_states.root_rot_mat * ct->ctrl_states.sphere_joint_location;
        // std::cout << "hri pos = " << ct->ctrl_states.Hri_pos.transpose() << std::endl;

        static bool init_flag = false;
        if(!init_flag) {
            std::cout << "height = " << ct->ctrl_states.root_pos[2] << std::endl;
            init_flag = true;
        }
        

        Eigen::Matrix<double, 3, 1> temp_rpy;
        temp_rpy = math::RollPitchYaw<T>(FloatingBodyFrame.CalcRotationMatrixInWorld(*_plant_context)).vector();
        ct->ctrl_states.root_euler = temp_rpy;
        if(temp_rpy[2] - ct->ctrl_states.last_robot_yaw < -M_PI) {
            ct->ctrl_states.robot_yaw_circle++;
        }
        else if(temp_rpy[2] - ct->ctrl_states.last_robot_yaw > M_PI) {
            ct->ctrl_states.robot_yaw_circle--;
        }
        ct->ctrl_states.last_robot_yaw = temp_rpy[2];
        ct->ctrl_states.root_euler[2] = 2 * M_PI * ct->ctrl_states.robot_yaw_circle + temp_rpy[2];

        
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
        
        // expressed in the body frame                 
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

        // body velocity + joint velocity
        ct->ctrl_states.generalizedQdot.segment<3>(0) = ct->ctrl_states.root_ang_vel_rel;
        ct->ctrl_states.generalizedQdot.segment<3>(3) = ct->ctrl_states.root_lin_vel_rel;
        ct->ctrl_states.generalizedQdot.segment<6>(6) = ct->ctrl_states.qdot;

        // TODOs: how to 
        Eigen::Vector3d current_vel; current_vel.setZero();
        current_vel = ct->ctrl_states.root_rot_mat * ct->ctrl_states.foot_vel_rel.block<3, 1>(0, 0);
        ct->ctrl_states.foot_acc_world.block<3, 1>(0, 0) = (current_vel - ct->ctrl_states.foot_vel_world.block<3, 1>(0, 0))/ct->ctrl_states.control_dt;
        // if(ct->ctrl_states.foot_acc_world(2, 0) > 0) ct->ctrl_states.foot_acc_world(2, 0) = 0.0;
        ct->ctrl_states.foot_vel_world.block<3, 1>(0, 0) = current_vel;

        current_vel = ct->ctrl_states.root_rot_mat * ct->ctrl_states.foot_vel_rel.block<3, 1>(0, 1);
        ct->ctrl_states.foot_acc_world.block<3, 1>(0, 1) = (current_vel - ct->ctrl_states.foot_vel_world.block<3, 1>(0, 1))/ct->ctrl_states.control_dt;
        // if(ct->ctrl_states.foot_acc_world(2, 1) > 0) ct->ctrl_states.foot_acc_world(2, 1) = 0.0;
        ct->ctrl_states.foot_vel_world.block<3, 1>(0, 1) = current_vel;
        
        // ct->ctrl_states.foot_vel_world.block<3, 1>(0, 0) = LeftFootFrame.CalcSpatialVelocityInWorld(*_plant_context).translational();
        // ct->ctrl_states.foot_vel_world.block<3, 1>(0, 1) = RightFootFrame.CalcSpatialVelocityInWorld(*_plant_context).translational();

        // ct->ctrl_states.foot_vel_rel.block<3, 1>(0, 0) = ct->ctrl_states.root_rot_mat.transpose() * ct->ctrl_states.foot_vel_world.block<3, 1>(0, 0);
        // ct->ctrl_states.foot_vel_rel.block<3, 1>(0, 1) = ct->ctrl_states.root_rot_mat.transpose() * ct->ctrl_states.foot_vel_world.block<3, 1>(0, 1);
        


        // if(context.get_time() < 0.001) {
        //     drake::log()->info("ct->ctrl_states.q = ");
        //     drake::log()->info(ct->ctrl_states.q);
        // }

        // // systems dynamics matrix
        // _control_model.CalcMassMatrix(*_plant_context, &ct->ctrl_states.Mq);
        // _control_model.CalcBiasTerm(*_plant_context, &ct->ctrl_states.Cv);
        // ct->ctrl_states.tau_g = _control_model.CalcGravityGeneralizedForces(*_plant_context);


        // human-position states:
        Eigen::Matrix<double, 12, 1> pos_rpy_states = this->GetInputPort("position_rotation").Eval(context);
        ct->ctrl_states.hri_joint_states = pos_rpy_states;


        static record_states_struct record_states;
        static bool finished_write = false;
        double now = context.get_time();
        if((ct->ctrl_states.t - ct->ctrl_states.k * ct->ctrl_states.control_dt) > ct->ctrl_states.control_dt)
        {

        
        if(!finished_write)
        {
            record_states.time_stamp.push_back(now);
            record_states.mx.push_back(ct->ctrl_states.hri_wrench_realtime[0]);
            record_states.my.push_back(ct->ctrl_states.hri_wrench_realtime[1]);
            record_states.mz.push_back(ct->ctrl_states.hri_wrench_realtime[2]);
            record_states.fx.push_back(ct->ctrl_states.hri_wrench_realtime[3]);
            record_states.fy.push_back(ct->ctrl_states.hri_wrench_realtime[4]);
            record_states.fz.push_back(ct->ctrl_states.hri_wrench_realtime[5]);

            record_states.roll.push_back(ct->ctrl_states.root_euler[0]);
            record_states.pitch.push_back( ct->ctrl_states.root_euler[1]);
            record_states.yaw.push_back( ct->ctrl_states.root_euler[2]);

            record_states.roll_ref.push_back( ct->ctrl_states.root_euler_d[0]);
            record_states.pitch_ref.push_back( ct->ctrl_states.root_euler_d[1]);
            record_states.yaw_ref.push_back( ct->ctrl_states.root_euler_d[2]);

            record_states.px.push_back( ct->ctrl_states.root_pos[0]);
            record_states.py.push_back(ct->ctrl_states.root_pos[1]);
            record_states.height_ref.push_back(ct->ctrl_states.root_pos_d[2]);
            record_states.height.push_back(ct->ctrl_states.root_pos[2]);

            record_states.gait_cycle.push_back(ct->ctrl_states.plan_contacts_phase[0]);

            record_states.human_px.push_back(ct->ctrl_states.hri_joint_states[0]);
            record_states.human_py.push_back(ct->ctrl_states.hri_joint_states[1]);
            record_states.human_pz.push_back(ct->ctrl_states.hri_joint_states[2]);
            // record_states.left_foot_force.push_back(ct->ctrl_states.foot_force_simulation(2, 0));
            // record_states.right_foot_force.push_back(ct->ctrl_states.foot_force_simulation(2, 1));

        }
        }
                
        // if(now > 12.5 && !finished_write) {
        //     yaml::SaveYamlFile("/home/haoyun/Data/Code/drake/workspace/centaur_sim/log/adjust_pitch_and_height.yaml", record_states);
        //     finished_write = true;
        //     std::cout << "write data ... " << record_states.time_stamp.size() << "in total." << std::endl;
        // }

    }
};


} // namespace centaur_sim
} // namespace workspace
} // namespace drake