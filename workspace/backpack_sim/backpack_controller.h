/*
 * @Author: haoyun 
 * @Date: 2023-02-02 17:02:25
 * @LastEditors: haoyun 
 * @LastEditTime: 2023-06-20 12:36:15
 * @FilePath: /drake/workspace/backpack_sim/backpack_controller.h
 * @Description: 
 * 
 * Copyright (c) 2023 by HAR-Lab, All Rights Reserved. 
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


class centaurrobot;

class WBIController;

    struct human_ref_traj_struct {
        std::vector<double> time;
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> height;
        std::vector<double> yaw;

        template <typename Archive>
        void Serialize(Archive* a) {
        a->Visit(DRAKE_NVP(time));
        a->Visit(DRAKE_NVP(x));
        a->Visit(DRAKE_NVP(y));
        a->Visit(DRAKE_NVP(height));
        a->Visit(DRAKE_NVP(yaw));
        }
    };

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

    human_ref_traj_struct human_ref_traj;
    int max_human_ref_index;
    int64_t k;
    Eigen::Vector3d total_torques;
    record_states_struct record_states;
    bool finished_write;


namespace drake{
namespace workspace{
namespace backpack_sim{


template<typename T>
class BackpackController : public systems::LeafSystem<T>
{

public:
    BackpackController()
    {

        this->DeclareVectorInputPort("full_states",
                                    3+3);

        this->DeclareAbstractInputPort("spatial_forces_in",
            Value<std::vector<drake::multibody::SpatialForce<double>>>());

        this->DeclareVectorOutputPort("actuated_torque", 3,
                                    &BackpackController::CalcTorques);

        human_ref_traj = drake::yaml::LoadYamlFile<human_ref_traj_struct>(
            drake::FindResourceOrThrow("drake/workspace/backpack_sim/ab6_treadmill_walking.yaml")
        );
        max_human_ref_index = human_ref_traj.x.size() - 1;
        k = 0;
        total_torques.setZero();
        finished_write = false;
    }

private:
    

    void CalcTorques(const systems::Context<T>& context,
                  systems::BasicVector<T>* output) const {

            
        // static Eigen::VectorXd total_torques(3); 
        // total_torques.setZero();

        double t = context.get_time();
        if((t - k * 0.001) > 0.001)
        {
            k++;
            Eigen::Vector3d prismatic_joint_q_des; prismatic_joint_q_des.setZero();
            Eigen::Vector3d prismatic_joint_qdot_des; prismatic_joint_qdot_des.setZero();


            int number_of_traj = 0;
            number_of_traj =  static_cast<int>((k/5));
            if(number_of_traj > max_human_ref_index) 
                number_of_traj = max_human_ref_index;
                
            prismatic_joint_q_des[0] = human_ref_traj.x[number_of_traj]*1.0;
            prismatic_joint_q_des[1] = human_ref_traj.y[number_of_traj];
            prismatic_joint_q_des[2] = human_ref_traj.height[number_of_traj] - human_ref_traj.height[0];

            Eigen::VectorXd states = this->GetInputPort("full_states").Eval(context);
            Eigen::Matrix<double, 3, 1> human_pos_stiff, human_pos_damp;
            human_pos_stiff << 5000, 5000, 10000;
            human_pos_damp << 3000, 3000, 5000;
            total_torques = human_pos_stiff.cwiseProduct(prismatic_joint_q_des-states.head(3)) 
                            + human_pos_damp.cwiseProduct(prismatic_joint_qdot_des-states.tail(3));

        
            // force data
            // spatial_vec include 12x6 wrenches for each joint; including the fixed joints and the 'worldWeld' joint
            // index [4]: hri_wrench

            const std::vector<drake::multibody::SpatialForce<double>>& spatial_vec =
                this->GetInputPort("spatial_forces_in").template Eval<std::vector<drake::multibody::SpatialForce<double>>>(context);
            Eigen::Matrix<double ,6, 1> hri_wrench = spatial_vec[4].get_coeffs();

            if(!finished_write)
            {
            record_states.time_stamp.push_back(t);

            record_states.mx.push_back(hri_wrench(0));
            record_states.my.push_back(hri_wrench(1));
            record_states.mz.push_back(hri_wrench(2));
            record_states.fx.push_back(hri_wrench(3));
            record_states.fy.push_back(hri_wrench(4));
            record_states.fz.push_back(hri_wrench(5));

            }

        }
        
       if(t > 30.0 && !finished_write) {
            yaml::SaveYamlFile("/home/haoyun/Data/Code/drake/workspace/backpack_sim/backpack_payload_30.yaml", record_states);
            finished_write = true;
            std::cout << "write data ... " << record_states.time_stamp.size() << "in total." << std::endl;
        }

        


        output->set_value(total_torques);
    
    }

    
};


} // namespace backpack_sim
} // namespace workspace
} // namespace drake