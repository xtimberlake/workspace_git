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

#include "drake/workspace/whole-body-sim/myPID_controller.h"

DEFINE_double(simulation_sec, 10.0,
              "Number of seconds to simulate.");
DEFINE_double(sim_dt, 3e-3,
              "The time step to use for MultibodyPlant model"
              "discretization.");


namespace drake {
namespace workspace {
namespace threelinks {
    void DoMain()
    {
        // construct builder
        systems::DiagramBuilder<double> builder;

        // add plant & scene graph to builder
        // auto [plant, scene_graph] = 
        //     multibody::AddMultibodyPlantSceneGraph(&builder, FLAGS_sim_dt);

        // or
        multibody::MultibodyPlant<double>* plant{};
        geometry::SceneGraph<double>* scene_graph{};
        std::tie(plant, scene_graph) =
            multibody::AddMultibodyPlantSceneGraph(&builder, FLAGS_sim_dt);

        // parse the urdf file; add to plant and scene
        // and take the model instance as "tl_index"(threelinks)
        std::string threelinksSdfPath = 
            "drake/workspace/whole-body-sim/threeLinks.sdf";
        std::string sdf = FindResourceOrThrow(threelinksSdfPath);
        drake::log()->info("succeed to load " + threelinksSdfPath + "!");
        multibody::Parser parser(plant, scene_graph);
        drake::multibody::ModelInstanceIndex tl_index;
        tl_index = parser.AddModelFromFile(sdf);

        // fix the base frame with world frame
        plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("table_fixed_frame"));
        
        // extract the joints(Do we have to extract them one by one?)
        // const multibody::RevoluteJoint<double>& joint_1 = 
        //     plant->GetJointByName<multibody::RevoluteJoint>("joint1");
        // const multibody::RevoluteJoint<double>& joint_2 = 
        //     plant->GetJointByName<multibody::RevoluteJoint>("joint2");
        // const multibody::RevoluteJoint<double>& joint_3 = 
        //     plant->GetJointByName<multibody::RevoluteJoint>("joint3");
    
        // multibody(the plant) configuration complete.
        // (but we are still able to add more systems to the builder)
        plant->Finalize();

        // Add controller system

        // auto controller = builder.AddSystem(std::make_unique(myPID_controller()));
        auto controller = builder.AddSystem<myPID_controller<double>>(*plant);

        // Creates and adds LCM publisher for visualization.
        auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();
        geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph, lcm);

        // check the number of joints
        // it's important to weld the base frame or the base will be 
        // treated as a floating-base which is connected to the world
        // with 6 free joints
        const int num_joints = plant->num_positions();
        drake::log()->info("There are " + std::to_string(num_joints)+ " joints in the model. ");
        // DRAKE_DEMAND(num_joints % 3 == 0); // an interesting assert
        
        // Add a constant source to the builder
        // VectorX<double> torques(num_joints); torques.setZero();
        // auto zeros_source = builder.AddSystem<systems::ConstantVectorSource>(torques);

        // auto logger = builder.AddSystem<systems::VectorLogSink<double>>(num_joints * 2 /* input size */, FLAGS_sim_dt);
        auto logger = builder.AddSystem<systems::VectorLogSink<double>>(num_joints /* input size */, FLAGS_sim_dt);
        // since the plant and the scene graph has been set up,
        // let connect the flow between them.
        // At least connnect the actuation of the model.(what kind of input:torque or position?)
        // builder.Connect(zeros_source->get_output_port(),
        //                 plant->get_actuation_input_port(tl_index));
        // builder.Connect(zeros_source->get_output_port(),
        //                 plant->get_actuation_input_port(tl_index));

        builder.Connect(controller->get_output_port(), 
                        plant->get_actuation_input_port(tl_index));

        builder.Connect(plant->get_state_output_port(), 
                        controller->GetInputPort("desired_state"));
        
        builder.Connect(controller->get_output_port(), logger->GetInputPort("data"));
        
    
        // now the diagram is complete
        auto diagram = builder.Build();
        
        // set up for simulation with the diagram
        systems::Simulator<double>simulator(*diagram);
        // initial condition(Note that the simulator only cares about the numerical
        // calculation; i.e. context)
        
        systems::Context<double>& plant_context = 
            diagram->GetMutableSubsystemContext(*plant,
                                                &simulator.get_mutable_context());
        VectorX<double> initial_state(2 * plant->num_actuators());
        initial_state 
        << -M_PI/2, M_PI/2, -M_PI/2,
        0, 0, 0;
        // joint_1.set_angle(&plant_context, initial_state(0));
        // joint_2.set_angle(&plant_context, initial_state(1));
        // joint_3.set_angle(&plant_context, initial_state(2));
        // or
        plant_context.SetDiscreteState(initial_state);
        

        // ready to run the simulation
        simulator.set_publish_every_time_step(false);
        simulator.set_target_realtime_rate(1.0);
        simulator.Initialize();

        // let's go
        simulator.AdvanceTo(FLAGS_simulation_sec);


        // data post processing
        // Plot the results (launch call_python_client to see the plots).
        const auto& log = logger->FindLog(simulator.get_context());
        common::CallPython("figure", 1);
        common::CallPython("clf");
        common::CallPython("plot", log.sample_times(),
                           log.data().row(0).transpose());
        common::CallPython("legend", common::ToPythonTuple("theta1"));
        common::CallPython("axis", "tight");

    }
}
}
}