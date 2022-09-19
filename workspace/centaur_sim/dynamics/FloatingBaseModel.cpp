#include "drake/workspace/centaur_sim/dynamics/FloatingBaseModel.h"


FloatingBaseModel::FloatingBaseModel(/* args */)
{
}

FloatingBaseModel::~FloatingBaseModel()
{
}

 void FloatingBaseModel::addBase(const Eigen::Matrix<double, 6, 6>& inertia)
 {
    this->InertiaTensor = inertia;
 }