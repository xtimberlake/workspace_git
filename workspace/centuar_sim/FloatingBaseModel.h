#pragma once 

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/common/find_resource.h"
#include "drake/systems/framework/witness_function.h"


namespace drake {
namespace workspace {
namespace centuar_sim {

template <typename T>
class CalcFullDynamics : public systems::LeafSystem<T> {
public:
    
    

    CalcFullDynamics(const std::string &model_file_name) 
        : _floatingBaseModel(0.0) {
        
        multibody::Parser(&_floatingBaseModel).AddModelFromFile(
            FindResourceOrThrow(model_file_name));
        drake::log()->info("succeed to load " + model_file_name + " as controller model!");

        _floatingBaseModel.Finalize();
        _plant_context =  _floatingBaseModel.CreateDefaultContext();

        

    }

private:
    multibody::MultibodyPlant<T> _floatingBaseModel;
    std::unique_ptr<systems::Context<T>> _plant_context;
    
};

} // centuar_sim
} // namespace workspace
} // namespace drake