#pragma once 

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake{
namespace workspace{
namespace centuar_sim{

template <typename T>
class plannerSystem :public drake::systems::LeafSystem<T>
{
private:
    /* data */
public:
    plannerSystem(/* args */)
    {
        this->DeclareVectorOutputPort(
            "p_des", 6,
            &plannerSystem::CalcOutput);
    }
    ~plannerSystem(){}

private:
    void CalcOutput(const systems::Context<T>& context,
                  systems::BasicVector<T>* output) const {

    Vector6d p_foot_desired; /* with respect to base frame */
    p_foot_desired << 
    -0.132, 0.173, -0.9,
    -0.132, -0.173, -0.9;

    double k = (-0.75 - (-0.9)) / (1 - (-1));
    double b = -0.9 - k * (-1);
    p_foot_desired(2) = k * sin(7 * context.get_time() + M_PI_2 * 3) + b;  
    p_foot_desired(5) = k * sin(7 * context.get_time() + M_PI_2 * 2) + b; 
    
    output->set_value(p_foot_desired);
  
  }
};




} // namespace centuar_sim
} // namespace workspace
} // namespace drake