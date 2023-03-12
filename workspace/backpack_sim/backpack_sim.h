#pragma once

#include <gflags/gflags.h>
#include <iostream>
#include <numeric>
#include <vector>
#include <algorithm>

#include <drake/common/text_logging.h>
#include <drake/common/find_resource.h>
#include <drake/common/drake_assert.h>
#include <drake/common/proto/call_python.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_interface_system.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/systems/primitives/zero_order_hold.h>
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

#include "drake/workspace/backpack_sim/backpack_controller.h"

DEFINE_double(simulation_sec, 30.01,
              "Number of seconds to simulate.");
DEFINE_double(sim_dt, 5e-4,
              "The time step to use for MultibodyPlant model"
              "discretization.");

namespace drake {
namespace workspace {
namespace backpack_sim {

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

        std::string backpack_scene_SDF_path = 
            "drake/workspace/backpack_sim/backpack_sim_scene.sdf";
        std::string sdf = FindResourceOrThrow(backpack_scene_SDF_path);  
        multibody::Parser parser(plant, scene_graph);
        drake::multibody::ModelInstanceIndex backpack_model_index;
        backpack_model_index = parser.AddModelFromFile(sdf);
        drake::log()->info("succeed to load " + backpack_scene_SDF_path + " to plant!");

        plant->WeldFrames(plant->world_frame(), plant->GetBodyByName("ground").body_frame());

        plant->Finalize();
        drake::log()->info("There are " +  std::to_string(plant->num_positions(backpack_model_index)) + " position DoFs in this multibodyplant.");
        drake::log()->info("There are " +  std::to_string(plant->num_velocities(backpack_model_index)) + " velocity DoFs in this multibodyplant.");
        drake::log()->info(std::to_string(plant->num_actuated_dofs(backpack_model_index)) + " actuated DoFs."); 
        
        

        // Creates and adds LCM publisher for visualization.
        auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();
        geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph, lcm);

        auto backpack_ctrller = builder.AddSystem<workspace::backpack_sim::BackpackController<double>>();

        auto zoh = builder.AddSystem<systems::ZeroOrderHold<double>>(FLAGS_sim_dt, plant->num_actuated_dofs(backpack_model_index));
        // What if we do not connect the acutated joints:
        // what():  Actuation input port for model instance centaur must be connected
        // Add a constant source to the builder
        // VectorX<double> torques(plant->num_actuators()); 
        // torques.setZero();
        

        // auto zeros_source = builder.AddSystem<systems::ConstantVectorSource>(torques);
        builder.Connect(plant->get_state_output_port(),
                        backpack_ctrller->GetInputPort("full_states"));

        builder.Connect(plant->get_reaction_forces_output_port(),
                        backpack_ctrller->GetInputPort("spatial_forces_in"));

        builder.Connect(backpack_ctrller->GetOutputPort("actuated_torque"),
                        zoh->get_input_port());

        builder.Connect(zoh->get_output_port(),
                        plant->get_actuation_input_port(backpack_model_index));






        auto diagram = builder.Build();

        // set up for simulation with the diagram
        systems::Simulator<double>simulator(*diagram);

        // I.C. in context
        systems::Context<double>& plant_context = 
            diagram->GetMutableSubsystemContext(*plant,
                                                &simulator.get_mutable_context());
                                    

        VectorX<double> initial_state(plant->num_positions() + plant->num_velocities());
        initial_state.setZero();

        plant_context.SetDiscreteState(initial_state);  
    
        

        // ready to run the simulation
        simulator.set_publish_every_time_step(false);
        simulator.set_target_realtime_rate(1.0);
        simulator.Initialize();

        // let's go
        simulator.AdvanceTo(FLAGS_simulation_sec);
        
    }

    
}
}
}


// reminder
// 1. run the simulation
// $ bazel run --config mosek //workspace/centaur_sim:centaur_sim
// 2. open meshcat
// bazel run //tools:meldis -- --open-window
// 3. python plot
// $ bazel run //common/proto:call_python_client_cli

// 4. check the sdf or urdf file
// $ bazel run //manipulation/util/:how_model 
//    ./manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf
// 5. open the tools:drake_visualizer
// $ bazel run //tools:drake_visualizer