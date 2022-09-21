/*
 * @Author: haoyun 
 * @Date: 2022-09-19 16:25:29
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-09-21 19:38:05
 * @FilePath: /drake/workspace/centaur_sim/dynamics/FloatingBaseModel.cpp
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#include "drake/workspace/centaur_sim/dynamics/FloatingBaseModel.h"

using namespace spatial;

FloatingBaseModel::FloatingBaseModel(/* args */)
{
}

FloatingBaseModel::~FloatingBaseModel()
{
}

 void FloatingBaseModel::addBase(const SpatialInertia<double>& inertia) {

   if (_nDof) {
    throw std::runtime_error("Cannot add base multiple times!\n");
   }

   Mat6<double> eye6 = Mat6<double>::Identity();
   Mat6<double> zero6 = Mat6<double>::Zero();
   SpatialInertia<double> zeroInertia(zero6);

   _nDof = 6;
   for (size_t i = 0; i < 6; i++)
   {
      _parents.push_back(0);
      _jointTypes.push_back(JointType::Nothing);
      _jointAxes.push_back(CoordinateAxis::X);
      _Xtree.push_back(eye6);
      _Ibody.push_back(zeroInertia);
      _bodyNames.push_back("N/A");
   }

   _jointTypes[5] = JointType::FloatingBase;
   _Ibody[5] = inertia;
   _bodyNames[5] = "Floating Base";

   addDynamicsVars(6);
}

void FloatingBaseModel::addDynamicsVars(int count) {
   if (count != 1 && count != 6) {
    throw std::runtime_error(
        "addDynamicsVars must be called with count=1 (joint) or count=6 "
        "(base).\n");
   }

   // Mat6<double> eye6 = Mat6<double>::Identity();
   SVec<double> zero6 = SVec<double>::Zero();
   Mat6<double> zero66 = Mat6<double>::Zero();
   SpatialInertia<double> zeroInertia(zero66);

   for (int i = 0; i < count; i++)
   {
      _v.push_back(zero6);
      _a.push_back(zero6);
      _avp.push_back(zero6);
      _c.push_back(zero6);
      _S.push_back(zero6);
      _f.push_back(zero6);
      _fvp.push_back(zero6);
      _ag.push_back(zero6);
      
   //  _IC.push_back(zeroInertia);
   //  _Xup.push_back(eye6);
   //  _Xa.push_back(eye6);
   }

  //TODOs: what dose this Jacobian matrix stand for?
   _J.push_back(D6Mat<double>::Zero(6, _nDof));
   _Jdqd.push_back(SVec<double>::Zero());
   


}

void FloatingBaseModel::resizeSystemMatricies() {

  _H.setZero(_nDof, _nDof);
  _C.setZero(_nDof, _nDof);
  _Cqd.setZero(_nDof);
  _G.setZero(_nDof);
  for (size_t i = 0; i < _J.size(); i++) {
    _J[i].setZero(6, _nDof);
    _Jdqd[i].setZero();
  }

  for (size_t i = 0; i < _Jc.size(); i++) {
    _Jc[i].setZero(3, _nDof);
    _Jcdqd[i].setZero();
  }
  
//   _qdd_from_subqdd.resize(_nDof - 6, _nDof - 6);
//   _qdd_from_base_accel.resize(_nDof - 6, 6);
  _state.q = DVec<double>::Zero(_nDof - 6);
  _state.qd = DVec<double>::Zero(_nDof - 6);
   
}


/*!
 * Add a body
 * @param inertia The inertia of the body
 * @param parent The parent body, which is also assumed to be the body the rotor
 * is connected to
 * @param jointType The type of joint (prismatic or revolute)
 * @param jointAxis The joint axis (X,Y,Z), in the parent's frame
 * @param Xtree  The coordinate transformation from parent to this body
 * @return The body's ID (can be used as the parent)
 */
int FloatingBaseModel::addBody(const SpatialInertia<double>& inertia, int parent,
              JointType jointType, spatial::CoordinateAxis jointAxis,
              const Mat6<double>& Xtree) {

  if (static_cast<size_t> (parent) > _nDof) {
    throw std::runtime_error(
        "addBody got invalid parent: " + std::to_string(parent) +
        " nDofs: " + std::to_string(_nDof) + "\n");
  }

  _parents.push_back(parent);
  _jointTypes.push_back(jointType);
  _jointAxes.push_back(jointAxis);
  _Xtree.push_back(Xtree);
  _Ibody.push_back(inertia);
  _nDof++;

  addDynamicsVars(1);
  
  return _nDof;

}

int FloatingBaseModel::addGroundContactPoint(int bodyID, const Vec3<double> &location) {

  if (static_cast<size_t> (bodyID) > _nDof) {
    throw std::runtime_error(
        "addGroundContactPoint got invalid bodyID: " + std::to_string(bodyID) +
        " nDofs: " + std::to_string(_nDof) + "\n");
  }

  _gcParent.push_back(bodyID);
  _gcLocation.push_back(location);
  
  Vec3<double> zero3 = Vec3<double>::Zero();

  _pGC.push_back(zero3);
  _vGC.push_back(zero3);

  D3Mat<double> J(3, _nDof);
  J.setZero();

  _Jc.push_back(J);
  _Jcdqd.push_back(zero3);

  resizeSystemMatricies();
  return _nGroundContact++;
  
}

