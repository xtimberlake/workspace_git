/*
 * @Author: haoyun 
 * @Date: 2022-09-19 16:25:38
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-09-19 17:19:02
 * @FilePath: /drake/workspace/centaur_sim/dynamics/FloatingBaseModel.h
 * @Description: 
 * 
 * from MIT-Cheetah open source code
 */
#pragma once
#include <eigen3/Eigen/Core>
#include <vector>
#include <eigen3/Eigen/StdVector>

enum class CoordinateAxis { X, Y, Z };
enum class JointType { Prismatic, Revolute, FloatingBase, Nothing };

class FloatingBaseModel
{
public:
    FloatingBaseModel(/* args */);
    ~FloatingBaseModel();

    void addBase(const Eigen::Matrix<double, 6, 6>& inertia);

    Eigen::Matrix<double, 6, 6> InertiaTensor;

    size_t _nDof;

    std::vector<std::string> bodyNames;
    std::vector<int> _parents; // parent body's id
    std::vector<JointType> _jointTypes; // The type of joint (prismatic or revolute)
    std::vector<CoordinateAxis> _jointAxes; // The joint axis (X,Y,Z), in the parent's frame
    std::vector<Eigen::Matrix<double, 6, 6>,
                Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>> _Xtree; // The coordinate transformation from parent to this body
    std::vector<Eigen::Matrix<double, 6, 6>,
                Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>> _Ibody; // Inertia Tensor
    


};


