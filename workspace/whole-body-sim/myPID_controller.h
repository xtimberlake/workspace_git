#pragma once

#include <memory>
#include <iostream>

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/multibody/plant/multibody_plant.h"

/// @system
/// name: myPID_controller
/// input_ports:
/// - desired states
/// - current states
/// output_ports:
/// - motor_torques
/// @endsystem

namespace drake{
namespace workspace{
namespace threelinks{

template <typename T>
class myPID_controller : public drake::systems::LeafSystem<T> {
 public:
  myPID_controller(const multibody::MultibodyPlant<T>& plant) 
    : plant_(plant) {
    this->DeclareVectorInputPort("desired_state", 6);
    this->DeclareVectorOutputPort(
      "motor_torques", 3,
      &myPID_controller::CalcOutput);

  }
 
 private:

  void CalcOutput(const systems::Context<T>& context,
                  systems::BasicVector<T>* output) const {
    
    Vector6d states;
    Vector3<T> a;
    a << context.get_time(), 0.0, 0.0;

    states = this->EvalVectorInput(context, 0)->CopyToVector();  
    output->set_value(a);
  
  }

  const multibody::MultibodyPlant<T>& plant_;
};
    


} // namespace threelinks
} // namespace workspace
} // namespace drake