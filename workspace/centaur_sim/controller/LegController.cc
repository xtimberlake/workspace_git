#include "LegController.h"

LegController::LegController()
{
    this->_kp_stance << 5.0, 2, 2;
    this->_kd_stance << 5.0, 2, 2;

    this->_kp_swing << 40.0, 30, 30;
    this->_kd_swing << 20.0, 10, 10;
}

Eigen::Matrix<double, 6, 1> LegController::impedance_control(const CentaurStates& state){

    Eigen::Matrix<double, 6, 1> torques;
    

    for (int leg = 0; leg < 2; leg++)
    {
        if (state.plan_contacts_phase(leg) > .99) {
            torques.setZero();
        }
        else {
           
        }
    }
    
        

    return torques;
}