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

#include "drake/workspace/centuar_sim/planner.h"
#include "drake/workspace/centuar_sim/staticInvController.h"

DEFINE_double(simulation_sec, 20.0,
              "Number of seconds to simulate.");
DEFINE_double(sim_dt, 3e-3,
              "The time step to use for MultibodyPlant model"
              "discretization.");

namespace drake {
namespace workspace {
namespace centuar_sim {
    void DoMain()
    {
        // construct a builder
        systems::DiagramBuilder<double> builder;

        // add plant & scene graph to builder
        multibody::MultibodyPlant<double>* plant{};
        geometry::SceneGraph<double>* scene_graph{};
        std::tie(plant, scene_graph) =
            multibody::AddMultibodyPlantSceneGraph(&builder, FLAGS_sim_dt);

        std::string centuarURDFPath = 
            "drake/workspace/centuar_sim/centaur_v3.urdf";
        std::string urdf = FindResourceOrThrow(centuarURDFPath);  
        multibody::Parser parser(plant, scene_graph);
        drake::multibody::ModelInstanceIndex centuar_model_index;
        centuar_model_index = parser.AddModelFromFile(urdf);
        drake::log()->info("succeed to load " + centuarURDFPath + " to plant!");


        plant->Finalize();
        drake::log()->info("There are " +  std::to_string(plant->num_positions(centuar_model_index)) + " DoFs in this multibodyplant.");
        drake::log()->info(std::to_string(plant->num_actuated_dofs(centuar_model_index)) + " actuated DoFs."); 
        drake::log()->info(std::to_string(plant->num_actuators()) + " actuators."); 
        

        // Creates and adds LCM publisher for visualization.
        auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();
        geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph, lcm);

        // Add a constant source to the builder
        // VectorX<double> torques(plant->num_positions); torques.setZero();
        // auto zeros_source = builder.AddSystem<systems::ConstantVectorSource>(torques);

        // What if we do not connect the acutated joints:
        // what():  Actuation input port for model instance centuar must be connected
        // Add a constant source to the builder
        VectorX<double> torques(plant->num_actuators()); 
        torques.setZero();
        
        // auto zeros_source = builder.AddSystem<systems::ConstantVectorSource>(torques);
        
        auto planner_logger = builder.AddSystem<systems::VectorLogSink<double>>(6);

        auto planner = builder.AddSystem<plannerSystem<double>>();

        auto controller = builder.AddSystem<staticInvController<double>>(*plant);


        // builder.Connect(zeros_source->get_output_port(),
        //                 plant->get_actuation_input_port(centuar_model_index));

        builder.Connect(controller->get_output_port(),
                        plant->get_actuation_input_port(centuar_model_index));

        builder.Connect(controller->get_output_port(),
                        planner_logger->get_input_port());

        builder.Connect(planner->get_output_port(),
                        controller->GetInputPort("desired_position"));
        
        builder.Connect(plant->get_state_output_port(),
                        controller->GetInputPort("states"));
    

        auto diagram = builder.Build();

        // set up for simulation with the diagram
        systems::Simulator<double>simulator(*diagram);
        // I.C. in context
        systems::Context<double>& plant_context = 
            diagram->GetMutableSubsystemContext(*plant,
                                                &simulator.get_mutable_context());
                                    
        VectorX<double> initial_state(plant->num_positions() + plant->num_velocities());
        initial_state <<
        // 1, 0, 0, 0,
        // 0, 0, 0.9,
        0, acos((0.9 * 0.9 + 0.58 * 0.58 - 0.445 * 0.445) / (2 * 0.9 * 0.58)), acos((-0.9 * 0.9 + 0.58 * 0.58 + 0.445 * 0.445) / (2 * 0.445 * 0.58)) - M_PI,
        0, acos((0.9 * 0.9 + 0.58 * 0.58 - 0.445 * 0.445) / (2 * 0.9 * 0.58)), acos((-0.9 * 0.9 + 0.58 * 0.58 + 0.445 * 0.445) / (2 * 0.445 * 0.58)) - M_PI;

        plant_context.SetDiscreteState(initial_state);  

        MatrixX<double> J_BF_left(3, 6);
        plant->CalcJacobianTranslationalVelocity(plant_context,
                                                multibody::JacobianWrtVariable::kQDot,
                                                plant->GetBodyByName("hind_left_foot").body_frame(),
                                                Vector3<double>::Zero(),
                                                plant->GetBodyByName("floating_base").body_frame(),
                                                plant->GetBodyByName("floating_base").body_frame(),
                                                &J_BF_left);
        
        math::RigidTransformd X_BF_left = plant->GetBodyByName("hind_left_foot").body_frame().CalcPose(plant_context, plant->GetBodyByName("floating_base").body_frame());
        math::RigidTransformd X_BF_right = plant->GetBodyByName("hind_right_foot").body_frame().CalcPose(plant_context, plant->GetBodyByName("floating_base").body_frame());

        std::cout << J_BF_left << std::endl;

        std::cout << X_BF_left.translation() << std::endl;
        std::cout << X_BF_right.translation() << std::endl;
        

        // ready to run the simulation
        simulator.set_publish_every_time_step(false);
        simulator.set_target_realtime_rate(1.0);
        simulator.Initialize();

        // let's go
        simulator.AdvanceTo(FLAGS_simulation_sec);

        // data post processing
        // Plot the results (launch call_python_client to see the plots).
        const auto& log = planner_logger->FindLog(simulator.get_context());
        common::CallPython("figure", 1);
        common::CallPython("clf");
        common::CallPython("plot", log.sample_times(),
                           log.data().row(2).transpose());
        common::CallPython("legend", common::ToPythonTuple("left_z"));
        common::CallPython("axis", "tight");
        
        
        
    }
}
}
}