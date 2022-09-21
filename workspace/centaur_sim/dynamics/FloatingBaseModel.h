/*
 * @Author: haoyun 
 * @Date: 2022-09-19 16:25:38
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-09-21 19:55:31
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

enum class JointType { Prismatic, Revolute, FloatingBase, Nothing };


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
    int addBody(const SpatialInertia<double>& inertia, int parent,
              JointType jointType, spatial::CoordinateAxis jointAxis,
              const Mat6<double>& Xtree);
    int addGroundContactPoint(int bodyID, const Vec3<double> &location);

    // constant physical parameters
    size_t _nDof;

    std::vector<std::string> _bodyNames;
    std::vector<int> _parents; // parent body's id
    std::vector<JointType> _jointTypes; // The type of joint (prismatic or revolute)
    std::vector<spatial::CoordinateAxis> _jointAxes; // The joint axis (X,Y,Z), in the parent's frame
    std::vector<Eigen::Matrix<double, 6, 6>,
                Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>> _Xtree; // The coordinate transformation from parent to this body
    std::vector<SpatialInertia<double>> _Ibody; // Inertia Tensor expressed in the local(joint) frame

    // states-dependent variables

    vectorAligned<SVec<double>> _v,  _a, _avp, _c, _S, _fvp, _ag, _f;
    // Spatial velocity, acceleration, ?, ?, screw asix, ?, gravity acc, wrench

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

};


