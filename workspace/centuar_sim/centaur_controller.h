#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/common/find_resource.h"

namespace drake{
namespace workspace{
namespace centuar_sim{

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

    }

private:
    multibody::MultibodyPlant<T> _control_model;
    std::unique_ptr<systems::Context<T>> _plant_context;

    void CalcTorques(const systems::Context<T>& context,
                  systems::BasicVector<T>* output) const {

        Eigen::VectorXd output_torques(_control_model.num_actuators()); output_torques.setZero();
        update_states(context);
        
        output->set_value(output_torques);

    }

    void update_states(const systems::Context<T>& context) const
    {
        Eigen::VectorXd states = this->GetInputPort("full_states").Eval(context);
        _control_model.SetPositionsAndVelocities(_plant_context.get(), states);

    }
};


} // namespace centuar_sim
} // namespace workspace
} // namespace drake