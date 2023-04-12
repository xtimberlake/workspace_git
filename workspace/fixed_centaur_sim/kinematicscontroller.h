/*
 * @Author: haoyun 
 * @Date: 2022-07-22 14:53:06
 * @LastEditors: haoyun 
 * @LastEditTime: 2023-04-12 21:44:29
 * @FilePath: /drake/workspace/fixed_centaur_sim/kinematicscontroller.h
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once

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

#include <gflags/gflags.h>
#include <iostream>

#include <drake/common/text_logging.h>
#include <drake/common/find_resource.h>
#include <drake/common/drake_assert.h>
#include <drake/common/proto/call_python.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_interface_system.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/geometry/scene_graph.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/solver_interface.h>
#include <drake/solvers/mosek_solver.h>
#include <drake/solvers/mathematical_program_result.h>
#include <drake/common/yaml/yaml_io.h>

#include "Utils/Utils.h"

namespace drake{
namespace workspace{
namespace fixed_centaur_sim{

template <typename T>
class KinematicsController : public drake::systems::LeafSystem<T> 
{
private:
    /* data */
public:
    KinematicsController(const std::string &control_model_file_name) 
        : _control_model(0.0){
            
        multibody::Parser(&_control_model).AddModelFromFile(
            FindResourceOrThrow(control_model_file_name));  
        drake::log()->info("succeed to load " + control_model_file_name + " as control model!");

        _control_model.WeldFrames(_control_model.world_frame(), _control_model.GetBodyByName("ground").body_frame());
        _control_model.Finalize();
        _plant_context = _control_model.CreateDefaultContext();

        this->DeclareVectorInputPort("states", 12);
        this->DeclareVectorOutputPort(
            "motor_torques", 6,
            &KinematicsController::CalcJacobianOutput);

        
        
    }

private:
    multibody::MultibodyPlant<T> _control_model;
    std::unique_ptr<systems::Context<T>> _plant_context;
    
    
    

    void CalcJacobianOutput(const systems::Context<T>& context,
                  systems::BasicVector<T>* output) const {

    Eigen::Matrix<double, 6, 1> computed_torques;
    computed_torques.setZero();

    VectorX<double> states;
    states = this->GetInputPort("states").Eval(context);
    // drake::log()->info(states);

    _control_model.SetPositionsAndVelocities(_plant_context.get(), states);
    static int sec, k;
    double plan_swing_phase;
    Eigen::Vector3d vel_des, vel_cur, vel_error;
    Eigen::Vector3d pos_des, pos_cur, pos_error, foot_forces_kin;
    Eigen::Vector3d foot_tartget;
    Eigen::Vector3d defualt_foot_pose;
    Eigen::Vector3d kp, kd;
    defualt_foot_pose << -0.132, 0.173, -0.9;
    foot_tartget << -0.132, 0.173, -0.9;
    pos_error.setZero();

    kp << 200, 50, 50; // x, y, z
    kd << 400, 100, 100;

    // kp << 1000, 1000, 1000; // x, y, z
    // kd << 500, 500, 200;

    // kp << 200, 100, 100; // x, y, z
    // kd << 300, 50, 33;

    FootSwingTrajectory<double>  swingtrajectory[2];
    swingtrajectory[0].setInitialPosition(defualt_foot_pose);
    swingtrajectory[0].setHeight(FOOT_SWING_CLEARANCE2);
    swingtrajectory[1].setHeight(FOOT_SWING_CLEARANCE2);

    const multibody::BodyFrame<T>& FloatingBodyFrame = 
                _control_model.GetBodyByName("floating_base").body_frame();
    const multibody::BodyFrame<T>& LeftFootFrame = _control_model.GetBodyByName("left_foot").body_frame();
    pos_cur = LeftFootFrame.CalcPose(*_plant_context, FloatingBodyFrame).translation();
    
    MatrixX<double> J_BF_left(3, _control_model.num_positions());
        _control_model.CalcJacobianTranslationalVelocity(*_plant_context,
                                                         multibody::JacobianWrtVariable::kQDot,
                                                         LeftFootFrame,
                                                         Vector3<double>::Zero(),
                                                         FloatingBodyFrame,
                                                         FloatingBodyFrame,
                                                         &J_BF_left);

    
    Eigen::Matrix<double, 3, 3> Jac;
    Jac = J_BF_left.block<3, 3>(0, 0);
    Eigen::Matrix<double, 6, 1> qdot_vec = _control_model.GetVelocities(*_plant_context);
    vel_cur = Jac * qdot_vec.segment<3>(0);

    if(context.get_time() > k * 0.001)
    {
        k++;
        if (context.get_time() > sec * 1) {
        sec++;
        }
    
    if(sec % 2 == 0)  
        plan_swing_phase = std::fmod(context.get_time(), 1.0f);
    else
        plan_swing_phase = 0.0f;
    // drake::log()->info(computed_torques.transpose());
    if (plan_swing_phase <= 0.0001)
    {
        
        
        swingtrajectory[0].setInitialPosition(defualt_foot_pose);
        pos_des = defualt_foot_pose;
        pos_error = pos_des - pos_cur;
    }
    else
    {
        swingtrajectory[0].setFinalPosition(foot_tartget);
        swingtrajectory[0].computeSwingTrajectoryBezier(plan_swing_phase, 1.0);
        pos_des = swingtrajectory[0].getPosition();
        vel_des = swingtrajectory[0].getVelocity();

        pos_error = pos_des - pos_cur;
        vel_error = vel_des - vel_cur;
        
        foot_forces_kin = kp.cwiseProduct(pos_error) + kd.cwiseProduct(vel_error);
        // drake::log()->info(foot_forces_kin.transpose());
        computed_torques.segment<3>(0) = Jac.lu().solve(foot_forces_kin);
        // computed_torques.segment<3>(0) = Jac.transpose() * foot_forces_kin;
        // computed_torques.segment<3>(0) = Utils::pseudo_inverse(Jac) *  foot_forces_kin;
        
    }
    drake::log()->info(pos_error.transpose());
    

    }
    
    output->set_value(computed_torques);
    }

};



} // namespace fixed_centaur_sim
} // namespace workspace
} // namespace drake
