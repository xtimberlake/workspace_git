/*
 * @Author: haoyun 
 * @Date: 2022-10-10 15:11:18
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-10-12 20:04:33
 * @FilePath: /drake/workspace/centaur_sim/controller/ContactSet/SingleContact.hpp
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once

#include "drake/workspace/centaur_sim/controller/ContactSet/ContactSpec.hpp"
#include "drake/workspace/centaur_sim/dynamics/CentaurModel.h"
#include "drake/workspace/centaur_sim/dynamics/FloatingBaseModel.h"

template <typename T>
class SingleContact : public ContactSpec<T> {
 public:
  SingleContact(const FloatingBaseModel* robot, int contact_pt);
  virtual ~SingleContact();

  void setMaxFz(T max_fz) { _max_Fz = max_fz; }

 protected:
  T _max_Fz;
  int _contact_pt;
  int _dim_U;

  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateInequalityVector();

  const FloatingBaseModel* robot_sys_;
};

