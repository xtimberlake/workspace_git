#include "drake/workspace/centaur_sim/controller/ConvexMPC.h"

ConvexMPC::ConvexMPC(int mpc_horizon, 
                     Eigen::VectorXd q_weights, 
                     Eigen::VectorXd r_weights)
{
    this->_mpc_horizon = mpc_horizon;
    this->_state_dim = q_weights.rows();
    this->_u_dim = r_weights.rows();
    drake::log()->info(_state_dim);

    _Q_qp.resize(_state_dim * _mpc_horizon, _state_dim * _mpc_horizon);
    _R_qp.resize(_u_dim * _mpc_horizon, _u_dim * _mpc_horizon);

}