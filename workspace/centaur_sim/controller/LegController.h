#pragma once

#include "drake/workspace/centaur_sim/controller/CentaurStates.h"

class LegController
{
private:
    /* data */
public:
    LegController();
    Eigen::Matrix<double, 6, 1> impedance_control(const CentaurStates& state);

private:
    Eigen::Matrix<double, 3, 1> _kp_swing, _kp_stance;
    Eigen::Matrix<double, 3, 1> _kd_swing, _kd_stance;
};

