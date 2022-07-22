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

#include "kinematicscontroller.h"


DEFINE_double(simulation_sec, 20.0,
              "Number of seconds to simulate.");
DEFINE_double(sim_dt, 5e-4,
              "The time step to use for MultibodyPlant model"
              "discretization.");

namespace drake {
namespace workspace {
namespace fixed_centaur_sim {

    struct test_params_struct {
        double foo{0.0};
        std::vector<double> bar;

        template <typename Archive>
        void Serialize(Archive* a) {
        a->Visit(DRAKE_NVP(foo));
        a->Visit(DRAKE_NVP(bar));
        }
    };

    void DoMain()
    {
        // construct a builder
        systems::DiagramBuilder<double> builder;

        // add plant & scene graph to builder
        multibody::MultibodyPlant<double>* plant{};
        geometry::SceneGraph<double>* scene_graph{};
        std::tie(plant, scene_graph) =
            multibody::AddMultibodyPlantSceneGraph(&builder, FLAGS_sim_dt);

        std::string centaur_scene_SDF_path = 
            "drake/workspace/fixed_centaur_sim/sdf/fixed_centaur.sdf";
        std::string sdf = FindResourceOrThrow(centaur_scene_SDF_path);  
        multibody::Parser parser(plant, scene_graph);
        drake::multibody::ModelInstanceIndex centaur_model_index;
        centaur_model_index = parser.AddModelFromFile(sdf);
        drake::log()->info("succeed to load " + centaur_scene_SDF_path + " to plant!");

        plant->WeldFrames(plant->world_frame(), plant->GetBodyByName("ground").body_frame());

        plant->Finalize();

        drake::log()->info("There are " +  std::to_string(plant->num_positions(centaur_model_index)) + " position DoFs in this multibodyplant.");
        drake::log()->info("There are " +  std::to_string(plant->num_velocities(centaur_model_index)) + " velocity DoFs in this multibodyplant.");
        drake::log()->info(std::to_string(plant->num_actuated_dofs(centaur_model_index)) + " actuated DoFs."); 
        
        

        // Creates and adds LCM publisher for visualization.
        auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();
        geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph, lcm);


        // VectorX<double> torques(plant->num_actuators()); 
        // torques.setZero();     
        // auto zeros_source = builder.AddSystem<systems::ConstantVectorSource>(torques);
        // builder.Connect(zeros_source->get_output_port(),
        //                 plant->get_actuation_input_port(centaur_model_index));

        std::string control_model_path = 
            "drake/workspace/fixed_centaur_sim/sdf/fixed_centaur_control_model.sdf";
        
        auto kin_controller_block = builder.AddSystem<workspace::fixed_centaur_sim::KinematicsController<double>>(control_model_path);


        builder.Connect(plant->get_state_output_port(),
                        kin_controller_block->get_input_port());

        builder.Connect(kin_controller_block->get_output_port(),
                        plant->get_actuation_input_port());


        auto diagram = builder.Build();

        // set up for simulation with the diagram
        systems::Simulator<double>simulator(*diagram);

        // I.C. in context
        systems::Context<double>& plant_context = 
            diagram->GetMutableSubsystemContext(*plant,
                                                &simulator.get_mutable_context());
                                    
        VectorX<double> initial_state(plant->num_positions() + plant->num_velocities());
        // initial_state.setZero();
        initial_state <<
        // 1, 0, 0, 0,
        // 0, 0, 0,
        0, acos((0.9 * 0.9 + 0.58 * 0.58 - 0.455 * 0.455) / (2 * 0.9 * 0.58)), acos((-0.9 * 0.9 + 0.58 * 0.58 + 0.455 * 0.455) / (2 * 0.455 * 0.58)) - M_PI,
        0, acos((0.9 * 0.9 + 0.58 * 0.58 - 0.455 * 0.455) / (2 * 0.9 * 0.58)), acos((-0.9 * 0.9 + 0.58 * 0.58 + 0.455 * 0.455) / (2 * 0.455 * 0.58)) - M_PI;

        plant_context.SetDiscreteState(initial_state);  
        // plant_context.SetContinuousState(initial_state);                


        // ready to run the simulation
        simulator.set_publish_every_time_step(false);
        simulator.set_target_realtime_rate(1.0);
        simulator.Initialize();

        // let's go
        simulator.AdvanceTo(FLAGS_simulation_sec);
        // simulator.AdvanceTo(0.003);

        // // data post processing
        // // Plot the results (launch ./bazel-bin/common/proto/call_python_client_cli to see the plots).
        // const auto& log = states_logger->FindLog(simulator.get_context());
        // common::CallPython("figure", 1);
        // common::CallPython("clf");
        // common::CallPython("plot", log.sample_times(),
        //                    log.data().row(6).transpose());
        // common::CallPython("legend", common::ToPythonTuple("left_z"));
        // common::CallPython("axis", "tight");
        
        
        
    }

    
}
}
}