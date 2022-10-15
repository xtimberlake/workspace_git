/*
 * @Author: haoyun 
 * @Date: 2022-09-19 16:25:38
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-10-15 10:34:19
 * @FilePath: /drake/workspace/centaur_sim/dynamics/FloatingBaseModel.h
 * @Description: 
 * 
 * from MIT-Cheetah open source code
 */
#pragma once
#include <eigen3/Eigen/Core>
#include <vector>
#include <eigen3/Eigen/StdVector>

#include "drake/workspace/centaur_sim/dynamics/spatial.h"

using namespace spatial;
using namespace ori;

/*!
 * The state of a floating base model (base and joints)
 */
template <typename T>
struct FBModelState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Quat<T> bodyOrientation;
  Vec3<T> bodyPosition;
  SVec<T> bodyVelocity;  // body coordinates (6*1 vector)
  DVec<T> q; // joint angle
  DVec<T> qd;

  /*!
   * Print the position of the body
   */
  void print() const {
    printf("position: %.3f %.3f %.3f\n", bodyPosition[0], bodyPosition[1],
           bodyPosition[2]);
  }
};


template <typename T>
struct FBModelStateDerivative {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec3<T> dBodyPosition;
  SVec<T> dBodyVelocity;
  DVec<T> qdd;
};

class FloatingBaseModel
{
public:
    FloatingBaseModel(/* args */);
    ~FloatingBaseModel();

    void addBase(const SpatialInertia<double>& inertia);
    void addDynamicsVars(int count);
    void resizeSystemMatricies();
    int addBody(const std::string name, const SpatialInertia<double>& inertia, int parent,
              JointType jointType, ori::CoordinateAxis jointAxis,
              const Mat6<double>& Xtree);
    void printModelTable();
    int addGroundContactPoint(int bodyID, const Vec3<double> &location);
    void forwardKinematics();
    void biasAccelerations();
    void contactJacobians();
    void compositeInertias();
    DMat<double> massMatrix();
    DVec<double> generalizedGravityForce();
    DVec<double> generalizedCoriolisForce();


    void setState(const FBModelState<double>& state) {
      _state = state;
      resetCalculationFlags();
    }

   /*!
   * Mark all previously calculated values as invalid
   */
    void resetCalculationFlags() {
      // _articulatedBodiesUpToDate = false;
      _kinematicsUpToDate = false;
      _biasAccelerationsUpToDate = false;
      _compositeInertiasUpToDate = false;
      // _forcePropagatorsUpToDate = false;
      // _qddEffectsUpToDate = false;
      // _accelerationsUpToDate = false;
    }    

    // constant physical parameters
    size_t _nDof = 0;

    std::vector<std::string> _bodyNames;
    std::vector<int> _bodyID; // parent body's id
    std::vector<int> _parents; // parent body's id
    std::vector<JointType> _jointTypes; // The type of joint (prismatic or revolute)
    std::vector<ori::CoordinateAxis> _jointAxes; // The joint axis (X,Y,Z), in the parent's frame
    std::vector<Eigen::Matrix<double, 6, 6>,
                Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>> _Xtree; // The coordinate transformation from parent to this body
    std::vector<SpatialInertia<double>> _Ibody; // Inertia Tensor expressed in the local(joint) frame
    std::vector<SpatialInertia<double>> _IC;    // Inertia of composite rigid body  
    // states-dependent variables

    vectorAligned<SVec<double>> _v,  _a, _avp, _c, _S, _fvp, _ag, _f;
    // Spatial velocity, acceleration, velocity product acceleration, Coriolis accelerations, screw asix, ?, gravity acc, wrench
    vectorAligned<Mat6<double>> _Xup, _Xa, _Xai; // {i}^X_{i-1}, {i}^X_{world} and {world}^X_{i}

    vectorAligned<D6Mat<double>> _J;
    vectorAligned<SVec<double>> _Jdqd;

    // contact info
    size_t _nGroundContact = 0;
    std::vector<size_t> _gcParent;
    std::vector<Vec3<double>> _gcLocation, _pGC, _vGC; // position and velocity
    vectorAligned<D3Mat<double>> _Jc; // contatct Jacobian matrix
    vectorAligned<Vec3<double>> _Jcdqd; // JcDot*qdot

    DMat<double> _H, _C;
    DVec<double> _Cqd, _G;
    FBModelState<double> _state;
    FBModelStateDerivative<double> _dState;

    // flag 
    bool _kinematicsUpToDate = false;
    bool _biasAccelerationsUpToDate = false;
    bool _compositeInertiasUpToDate = false;

    Vec3<double> _gravity;

  /*!
   * Get the mass matrix for the system
   */
  const DMat<double>& getMassMatrix() const { return _H; }

  /*!
   * Get the gravity term (generalized forces)
   */
  const DVec<double>& getGravityForce() const { return _G; }

  /*!
   * Get the coriolis term (generalized forces)
   */
  const DVec<double>& getCoriolisForce() const { return _Cqd; }

};


