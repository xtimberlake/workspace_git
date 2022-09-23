/*
 * @Author: haoyun 
 * @Date: 2022-09-19 16:25:29
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-09-23 21:04:36
 * @FilePath: /drake/workspace/centaur_sim/dynamics/FloatingBaseModel.cpp
 * @Description: 
 * 
 * From MIT-Cheetah code
 */
#include "drake/workspace/centaur_sim/dynamics/FloatingBaseModel.h"

using namespace spatial;

FloatingBaseModel::FloatingBaseModel(/* args */)
{
}

FloatingBaseModel::~FloatingBaseModel()
{
}


void FloatingBaseModel::printModelTable() {
  if (_nDof == 0) {
    throw std::runtime_error("Please build the model before print!\n");
  }
  std::cout << "Total DoFs = " << _bodyID.size() << std::endl;
  for (size_t i = 0; i < _nDof; i++)
  {

    std::cout << "link \#" << _bodyID.at(i) << ": "
    << "     index = " << i << ","
    << "     parent = " << _parents.at(i) << ","
    << "     name = " << _bodyNames.at(i) << std::endl;
  }
  

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
      _bodyID.push_back(-1);
   }

   _jointTypes[5] = JointType::FloatingBase;
   _Ibody[5] = inertia;
   _bodyNames[5] = "Floating Base";
   _bodyID[5] = 5;

   addDynamicsVars(6);
}

void FloatingBaseModel::addDynamicsVars(int count) {
   if (count != 1 && count != 6) {
    throw std::runtime_error(
        "addDynamicsVars must be called with count=1 (joint) or count=6 "
        "(base).\n");
   }

   Mat6<double> eye6 = Mat6<double>::Identity();
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
      _Xup.push_back(eye6);
      _Xa.push_back(eye6);
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
int FloatingBaseModel::addBody(const std::string name, const SpatialInertia<double>& inertia, int parent,
              JointType jointType, ori::CoordinateAxis jointAxis,
              const Mat6<double>& Xtree) {

  if (static_cast<size_t> (parent) > _nDof) {
    throw std::runtime_error(
        "addBody got invalid parent: " + std::to_string(parent) +
        " nDofs: " + std::to_string(_nDof) + "\n");
  }

  _bodyNames.push_back(name);
  _parents.push_back(parent);
  _jointTypes.push_back(jointType);
  _jointAxes.push_back(jointAxis);
  _Xtree.push_back(Xtree);
  _Ibody.push_back(inertia);
  _bodyID.push_back(_nDof);
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

/*!
 * Forward kinematics of all bodies.  Computes _Xup (from up the tree) and _Xa
 *(from absolute) Also computes _S (motion subspace or Screw asix), _v (spatial velocity in
 *link coordinates), and _c (coriolis acceleration in link coordinates)
 */

void FloatingBaseModel::forwardKinematics() {
  
  if (_kinematicsUpToDate) return;
  
  // base transformation and twist
  _Xup[5] = createSXform(ori::quaternionToRotationMatrix(_state.bodyOrientation),
                         _state.bodyPosition);
  _v[5] = _state.bodyVelocity;

  

  // transforamtion and twist for each links(from ID=6 to 11)
  for (size_t i = 6; i < _nDof; i++) {
    
    Mat6<double> XJ = jointXform(_jointTypes[i], _jointAxes[i], _state.q[i - 6]);

    _Xup[i] = XJ * _Xtree[i];
    
    _S[i] = jointMotionSubspace<double>(_jointTypes[i], _jointAxes[i]);
    SVec<double> vJ = _S[i] * _state.qd[i - 6];
    // total velocity of body i
    _v[i] = _Xup[i] * _v[_parents[i]] + vJ;
    
    // Q(haoyun): what's Coriolis Accelerations?
    // A:[V]x S_i*qdot_i.
    // recall acceleration propagation: A_i = X_up*A_{parent} + [V]x S_i*qdot_i + S_i*qddot.
    _c[i] = motionCrossProduct(_v[i], vJ);
  }

  // calculate from absolute transformations
  // i.e., {i}^X_{world}
  _Xa[5] = _Xup[5]; 
  for (size_t i = 6; i < _nDof; i++) {
      _Xa[i] = _Xup[i] * _Xa[_parents[i]];
  }
  
  // position and velocity of contact points(foot-ends) in the world frame
  for (size_t j = 0; j < _nGroundContact; j++) {
    size_t i_link = _gcParent.at(j);
    Mat6<double> Xai = invertSXform(_Xa[i_link]);
    SVec<double> vSpatial = Xai * _v[i_link];
    
    _pGC.at(j) = sXFormPoint(Xai, _gcLocation.at(j));
    _vGC.at(j) = spatialToLinearVelocity(vSpatial, _pGC.at(j));
  }
  
  _kinematicsUpToDate = true;
  
}

/*!
 * (Support Function) Computes velocity product accelerations of
 * each link and rotor _avp, and _avprot
 */
void FloatingBaseModel::biasAccelerations() {
  if (_biasAccelerationsUpToDate) return;
  forwardKinematics();
  // velocity product acceelration of base
  _avp[5] << 0, 0, 0, 0, 0, 0;

  // from base to tips
  for (size_t i = 6; i < _nDof; i++) {
    // Outward kinamtic propagtion
    _avp[i] = _Xup[i] * _avp[_parents[i]] + _c[i];
  }
  _biasAccelerationsUpToDate = true;
}

/*!
 * Compute the contact Jacobians (3xn matrices) for the velocity
 * of each contact point expressed in absolute coordinates
 */
void FloatingBaseModel::contactJacobians() {
  forwardKinematics();
  biasAccelerations();

  for (size_t k = 0; k < _nGroundContact; k++) {
    _Jc[k].setZero();
    _Jcdqd[k].setZero();

    // // Skip it if we don't care about it
    // if (!_compute_contact_info[k]) continue;

    size_t i = _gcParent.at(k);

    // Rotation to absolute coords
    Mat3<double> Rai = _Xa[i].template block<3, 3>(0, 0).transpose();
    Mat6<double> Xc = createSXform(Rai, _gcLocation.at(k));

    // Bias acceleration
    SVec<double> ac = Xc * _avp[i];
    SVec<double> vc = Xc * _v[i];

    // Correct to classical
    _Jcdqd[k] = spatialToLinearAcceleration(ac, vc);

    // rows for linear velcoity in the world
    D3Mat<double> Xout = Xc.template bottomRows<3>();

    // from tips to base
    while (i > 5) {
      _Jc[k].col(i) = Xout * _S[i];
      Xout = Xout * _Xup[i];
      i = _parents[i];
    }
    _Jc[k].template leftCols<6>() = Xout;
  }
}
