/*
 * @Author: haoyun 
 * @Date: 2022-09-16 17:07:03
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-10-21 21:25:49
 * @FilePath: /drake/workspace/centaur_sim/controller/WBIController.cc
 * @Description: 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#include "drake/workspace/centaur_sim/controller/WBIController.h"


WBIController::WBIController(/* args */):
    _full_config(centaurParam::num_act_joint + 6),
    _tau_ff(centaurParam::num_act_joint),
    _des_jpos(centaurParam::num_act_joint),
    _des_jvel(centaurParam::num_act_joint) {



    num_qdot_ = centaurParam::dim_config; // 12
    num_act_joint_ = centaurParam::num_act_joint; // 6
    I_mtx =  DMat<double>::Identity(num_qdot_, num_qdot_); // 12x 12 Identity 

    _dim_floating = 6;
    _eye = DMat<double>::Identity(num_qdot_, num_qdot_); // 12x 12 Identity 
    _eye_floating = DMat<double>::Identity(_dim_floating, _dim_floating); // 6x 6 Identity 

    last_dim_decision_variables = 0;
    
    Sa_ = DMat<double>::Zero(num_act_joint_, num_qdot_);
    Sv_ = DMat<double>::Zero(_dim_floating, num_qdot_);

    Sa_.block(0, 6, num_act_joint_, num_act_joint_).setIdentity(); // [0 I]
    Sv_.block(0, 0, 6, 6).setIdentity(); // [I 0]

    _quat_des.setZero();
    _pBody_des.setZero();
    for (int i = 0; i < 4; i++) {
        _pFoot_des[i].setZero();
    }

    _full_config.setZero();
    _tau_ff.setZero();
    _des_jpos.setZero();
    _des_jvel.setZero();
    

    ctModel.buildModel();
    ctModel._fb_model.printModelTable();


    _torso_pos_task = new TorsoPosTask<double>(&(ctModel._fb_model));
    _torso_ori_task = new TorsoOriTask<double>(&(ctModel._fb_model));

    for (int i = 0; i < 2; i++) {
        _foot_contact[i] = new SingleContact<double>(&(ctModel._fb_model), i); // foot contact
        _foot_task[i] = new LinkPosTask<double>(&(ctModel._fb_model), i);
    }
    
}


WBIController::~WBIController(){ }


void WBIController::run(CentaurStates& state)
{
    // std::cout << "left contact = " << state.plan_contacts_phase[0] << "-----------" << std::endl;
    update_model(state);
    update_contact_task(state);
    kin_wbc();
    dyn_wbc();
    update_command(state, _des_jpos, _des_jvel, _tau_ff);

}


void WBIController::update_model(CentaurStates& state) {
    

    // Part1: update configuration of floating base model;
    // and store the dynamics matrix in whole-body controller.

    FBModelState<double> fb_states;
    /* The quat's coeffs in Drake is ordered as (x, y, z, w), which
       is different from MIT-Cheetah code*/
    fb_states.bodyOrientation[0] = state.root_quat.coeffs()[3];
    fb_states.bodyOrientation[1] = state.root_quat.coeffs()[0];
    fb_states.bodyOrientation[2] = state.root_quat.coeffs()[1];
    fb_states.bodyOrientation[3] = state.root_quat.coeffs()[2];

    fb_states.bodyPosition = state.root_pos; // Vec_from_world_to_base, expressed in world

    fb_states.bodyVelocity.head(3) = state.root_ang_vel_rel;
    fb_states.bodyVelocity.tail(3) = state.root_lin_vel_rel;
    
    fb_states.q = state.q;
    fb_states.qd = state.qdot;

    ctModel._fb_model.setState(fb_states);
    ctModel._fb_model.forwardKinematics();
    ctModel._fb_model.contactJacobians();
    ctModel._fb_model.massMatrix();
    ctModel._fb_model.generalizedGravityForce();
    ctModel._fb_model.generalizedCoriolisForce();
    
    
    // The equations of motion can be expressed as:
    // _A*qddot + _coriolis + _grav = tau + Jc^{T} * Fc
    _A = ctModel._fb_model.getMassMatrix();
    _grav = ctModel._fb_model.getGravityForce();
    _coriolis = ctModel._fb_model.getCoriolisForce();
    _Ainv = _A.inverse();
    
    // Part 2: copy the configuration(only care about the joint configuration 
    // in kinamatics-WBC?)
    for (size_t i = 0; i < num_act_joint_; i++) {
        _full_config[i + 6] = state.q[i]; 
    }
    
    // std::cout << "calculated contact velocity = " << ctModel._fb_model._vGC.at(1).transpose() << ", ";
    // std::cout << "ground true = " << state.foot_vel_world.block<3, 1>(0, 1).transpose() << std::endl;
    
    // std::cout << "computed Jacobian = " << std::endl << ctModel._fb_model._Jc.at(0) << ", " << std::endl;
    
    // std::cout << "-----" << std::endl;


}

void WBIController::clean_up() {
    _task_list.clear();
    _contact_list.clear();
}

void WBIController::update_contact_task(CentaurStates& state) {

    // wash out the previous setup
    clean_up();
    
    // receice target position
    _pBody_des = state.root_pos_d;
    _quat_des = ori::rpyToQuat(state.root_euler_d);
    _pFoot_des[0] = state.foot_pos_cmd_world.block<3, 1>(0, 0);
    _pFoot_des[1] = state.foot_pos_cmd_world.block<3, 1>(0, 1);

    

    // update desired variables for each tasks
    // Vec3<double> zero_vec3; zero_vec3.setZero();
    _torso_pos_task->UpdateTask(&_pBody_des, state.root_lin_vel_d_world, state.root_acc_d_world);
    _torso_ori_task->UpdateTask(&_quat_des, state.root_ang_vel_d_world, state.root_ang_acc_d_world);

    // store in the task lists
    _task_list.push_back(_torso_ori_task);
   
    
    for (size_t leg(0); leg < 2; leg++)
    {
        if (state.plan_contacts_phase[leg] > 0.) { // contact
            // NOTICE: the sign of GRF
            Vec3<double> impulse_force;
            impulse_force.setZero();
            if (state.plan_contacts_phase[leg] >= 0.95) {

                impulse_force[2] = 0 * sin(M_PI * (state.plan_contacts_phase[leg] - 0.95) / 0.05);
            }
            
            _foot_contact[leg]->setRFDesired(state.foot_force_cmd_world.block<3, 1>(0, leg) + impulse_force);
            _foot_contact[leg]->UpdateContactSpec();
            _contact_list.push_back(_foot_contact[leg]);

        }else{ // swing foot task
            _foot_task[leg]->UpdateTask(&_pFoot_des[leg],
                                            state.foot_vel_cmd_world.block<3, 1>(0, leg),
                                            state.foot_acc_cmd_world.block<3, 1>(0, leg));
            _task_list.push_back(_foot_task[leg]);
        }
    }
    
    _task_list.push_back(_torso_pos_task);
    
    
}

void WBIController::kin_wbc() {
    kin_wbcFindConfiguration(_full_config, _task_list, _contact_list,
                              _des_jpos, _des_jvel);
}


void WBIController::dyn_wbc() {

    // resize G, g0, CE, ce0, CI, ci0
    _SetOptimizationSize();
    _SetCost();

    // contact consistency
    DVec<double> qddot_pre;
    DMat<double> JcBar;
    DMat<double> Npre;

    if (_dim_rf > 0) {

        // Step #1 Update contact information;
        // i.e., _Jc, _JcDotQdot, _Uf, _Uf_ieq_vec, _Fr_des
        _ContactBuilding();

        // Step #2 Set inequality constraints
        _SetInEqualityConstraint();

        // Step #3 Build nullspace prejection
        _WeightedInverse(_Jc, _Ainv, JcBar); // dynamics cosistent inverse
        qddot_pre = JcBar * (-_JcDotQdot);   // xc_ddot = JcDotQdot * qdot + Jc * qddot; 
                                             // this acc command qddot is used to achieve contact consistency
        Npre = _eye - JcBar * _Jc; // iterative way to compute NullSpace projection

    } else {
        qddot_pre = DVec<double>::Zero(num_qdot_);
        Npre = _eye;
    }
    
    // Task(s)
    Task<double>* task;
    DMat<double> Jt, JtBar, JtPre;
    DVec<double> JtDotQdot, xddot;

    for (size_t i(0); i < (_task_list).size(); ++i) {
        task = (_task_list)[i];

        task->getTaskJacobian(Jt);
        task->getTaskJacobianDotQdot(JtDotQdot);
        task->getCommand(xddot);

        // std::cout << "xddot = (" << xddot.rows() << "," << xddot.cols() << ") = " << std::endl;
        // std::cout << xddot.transpose() << std::endl;

        JtPre = Jt * Npre;
        _WeightedInverse(JtPre, _Ainv, JtBar);

        qddot_pre += JtBar * (xddot - JtDotQdot - Jt * qddot_pre);
        Npre = Npre * (_eye - JtBar * JtPre);
    }

    // Set equality constraints
    _SetEqualityConstraint(qddot_pre);
    
    _SolveQuadraticProgramming(z_star);
    // std::cout << "original qddot = " << qddot_pre.head(6).transpose() << std::endl;
    // std::cout << "xstar = " << z_star.transpose() << ",  Total cost = " << cost << std::endl;

    _InverseDyn(qddot_pre, _tau_ff);
}

bool WBIController::kin_wbcFindConfiguration(const DVec<double>& curr_config,
                         const std::vector<Task<double>*>& task_list,
                         const std::vector<ContactSpec<double>*>& contact_list,
                         DVec<double>& jpos_cmd, DVec<double>& jvel_cmd) {

    // Contact Jacobian Setup
    // stack all the contact Jacobian matrix togather
    DMat<double> Nc(num_qdot_, num_qdot_); Nc.setIdentity();                        
    if(contact_list.size() > 0){ // no contact motion is the primary task
        DMat<double> Jc, Jc_i;
        contact_list[0]->getContactJacobian(Jc);
        size_t num_rows = Jc.rows();

        for (size_t i(1); i < contact_list.size(); ++i) {
        contact_list[i]->getContactJacobian(Jc_i);
        size_t num_new_rows = Jc_i.rows();
        Jc.conservativeResize(num_rows + num_new_rows, num_qdot_);
        Jc.block(num_rows, 0, num_new_rows, num_qdot_) = Jc_i;
        num_rows += num_new_rows;
        }

        // Projection Matrix
        _BuildProjectionMatrix(Jc, Nc);
    }

    // First Task
    DVec<double> delta_q, qdot;
    DMat<double> Jt, JtPre, JtPre_pinv, N_nx, N_pre;

    Task<double>* task = task_list[0];
    task->getTaskJacobian(Jt);
    JtPre = Jt * Nc;
    _PseudoInverse(JtPre, JtPre_pinv);

    delta_q = JtPre_pinv * (task->getPosError());
    qdot = JtPre_pinv * (task->getDesVel());

    DVec<double> prev_delta_q = delta_q;
    DVec<double> prev_qdot = qdot;

    _BuildProjectionMatrix(JtPre, N_nx);
    N_pre = Nc * N_nx;
    // recall: an iterative approach to calculate Null space projection matrix:
    // N_2 = N_1 * NullSpace(J_2 * N_1)
    // N_3 = N_2 * NullSpace(J_3 * N_2)
    // ...


    for (size_t i(1); i < task_list.size(); ++i) {
        task = task_list[i];

        task->getTaskJacobian(Jt);
        JtPre = Jt * N_pre;

        _PseudoInverse(JtPre, JtPre_pinv);
        delta_q =
            prev_delta_q + JtPre_pinv * (task->getPosError() - Jt * prev_delta_q);
        qdot = prev_qdot + JtPre_pinv * (task->getDesVel() - Jt * prev_qdot);

        // For the next task
        _BuildProjectionMatrix(JtPre, N_nx);
        N_pre *= N_nx;
        prev_delta_q = delta_q;
        prev_qdot = qdot;
    }

    for (size_t i(0); i < num_act_joint_; ++i) {
        jpos_cmd[i] = curr_config[i + 6] + delta_q[i + 6];
        jvel_cmd[i] = qdot[i + 6];
    }

    return true;
}


void WBIController::_SetOptimizationSize() {

  // Dimension
  _dim_rf = 0;  // Dimension of contact forces
  _dim_Uf = 0;  // Dimension of inequality constraint
  for (size_t i(0); i < (_contact_list).size(); ++i) {
    _dim_rf += (_contact_list)[i]->getDim();
    _dim_Uf += (_contact_list)[i]->getDimRFConstraint();
  }

  _dim_floating = 6;
  _dim_opt = _dim_floating + _dim_rf;
  _dim_eq_cstr = _dim_floating; // floating base dyn. equality constraints

  G.resize(_dim_opt, _dim_opt); G.setZero();
  g0.resize(_dim_opt); g0.setZero();
  CE.resize(_dim_eq_cstr, _dim_opt); CE.setZero();
  ce0.resize(_dim_eq_cstr); ce0.setZero();
  if (last_dim_decision_variables != _dim_opt) {
    initial_guess_vec.resize(_dim_opt); initial_guess_vec.setZero();
    last_dim_decision_variables = _dim_opt;
  }

  if(_dim_rf > 0) { // at least have one contact
    CI.resize(_dim_Uf, _dim_opt); CI.setZero();
    ci0.resize(_dim_Uf); ci0.setZero();
    
    _Jc = DMat<double>::Zero(_dim_rf, num_qdot_);
    _JcDotQdot = DVec<double>::Zero(_dim_rf);
    _Fr_des = DVec<double>::Zero(_dim_rf);

    _Uf = DMat<double>::Zero(_dim_Uf, _dim_rf);
    _Uf_ieq_vec = DVec<double>::Zero(_dim_Uf);

  } else {
    // drake::log()->info("no contact!!");
    CI.resize(1, _dim_opt); CI.setZero();
    ci0.resize(1); ci0.setZero();
  }

//   std::cout << "qp dim:" << " G=(" << G.rows() << "," << G.cols() << ")"
//   << " g0=(" << g0.rows() << "," << g0.cols() << ")" 
//   << " CE=(" << CE.rows() << "," << CE.cols() << ")"
//   << " ce0=(" << ce0.rows() << "," << ce0.cols() << ")"
//   << " CI=(" << CI.rows() << "," << CI.cols() << ")"
//   << " ci0=(" << ci0.rows() << "," << ci0.cols() << ")" << std::endl;

    // std::cout << "Jacobian dim:"
    // << " _Jc=(" << _Jc.rows() << "," << _Jc.cols() << ")"
    // << " _JcDotQdot=(" << _JcDotQdot.rows() << "," << _JcDotQdot.cols() << ")"
    // << " _Fr_des=(" << _Fr_des.rows() << "," << _Fr_des.cols() << ")"
    // << " _Uf=(" << _Uf.rows() << "," << _Uf.cols() << ")"
    // << " _Uf_ieq_vec=(" << _Uf_ieq_vec.rows() << "," << _Uf_ieq_vec.cols() << ")"
    // << std::endl;
  
}
void WBIController::_ContactBuilding() {
  DMat<double> Uf;
  DVec<double> Uf_ieq_vec;
  // Initial
  DMat<double> Jc; // temp contact Jacobian
  DVec<double> JcDotQdot; // temp JdQd
  size_t dim_accumul_rf, dim_accumul_uf;
  (_contact_list)[0]->getContactJacobian(Jc);
  (_contact_list)[0]->getJcDotQdot(JcDotQdot);
  (_contact_list)[0]->getRFConstraintMtx(Uf);
  (_contact_list)[0]->getRFConstraintVec(Uf_ieq_vec);

  dim_accumul_rf = (_contact_list)[0]->getDim();
  dim_accumul_uf = (_contact_list)[0]->getDimRFConstraint();

  // append to the head of _Jc, _JcDotQdot, _Uf, _Uf_ieq_vec, _Fr_des
  _Jc.block(0, 0, dim_accumul_rf, num_qdot_) = Jc;
  _JcDotQdot.head(dim_accumul_rf) = JcDotQdot;
  _Uf.block(0, 0, dim_accumul_uf, dim_accumul_rf) = Uf;
  _Uf_ieq_vec.head(dim_accumul_uf) = Uf_ieq_vec;
  _Fr_des.head(dim_accumul_rf) = (_contact_list)[0]->getRFDesired();

  // if there are more than one contact
  size_t dim_new_rf, dim_new_uf;

  for (size_t i = 1; i < _contact_list.size(); i++) {
    (_contact_list)[i]->getContactJacobian(Jc);
    (_contact_list)[i]->getJcDotQdot(JcDotQdot);

    dim_new_rf = (_contact_list)[i]->getDim();
    dim_new_uf = (_contact_list)[i]->getDimRFConstraint();

    // Jc append
    _Jc.block(dim_accumul_rf, 0, dim_new_rf, num_qdot_) = Jc;

    // JcDotQdot append
    _JcDotQdot.segment(dim_accumul_rf, dim_new_rf) = JcDotQdot;

    // Uf
    (_contact_list)[i]->getRFConstraintMtx(Uf);
    _Uf.block(dim_accumul_uf, dim_accumul_rf, dim_new_uf, dim_new_rf) = Uf;

    // Uf inequality vector
    (_contact_list)[i]->getRFConstraintVec(Uf_ieq_vec);
    _Uf_ieq_vec.segment(dim_accumul_uf, dim_new_uf) = Uf_ieq_vec;

    // Fr desired
    _Fr_des.segment(dim_accumul_rf, dim_new_rf) =
      (_contact_list)[i]->getRFDesired();
    dim_accumul_rf += dim_new_rf;
    dim_accumul_uf += dim_new_uf;
  }

}
void WBIController::_SetCost() {
    // Set p.s.d. weight matrix
  size_t idx_offset(0);

  // d/dt{[wx wy wz vx vy vz]}
  G.block<6, 6>(0, 0).diagonal() << 0.1, 0.1, 0.1, 0, 0, 0.1;

  idx_offset += _dim_floating;
  for (size_t i(0); i < _dim_rf; ++i) {
    G(i + idx_offset, i + idx_offset) = 1.0;
  }

}
void WBIController::_SetInEqualityConstraint() {
    // friction pyramid + normal grf limits
    if (_dim_rf > 0) {
        CI.block(0, _dim_floating, _dim_Uf, _dim_rf) = _Uf;
        ci0 = _Uf_ieq_vec - _Uf * _Fr_des;
    }
    
    
    // std::cout << "_Fr_des = (" << _Fr_des.rows() << "," << _Fr_des.cols() << ") = " << std::endl;
    // std::cout << _Fr_des.transpose() << std::endl;
}

void WBIController::_SetEqualityConstraint(const DVec<double>& qddot) {
    // floating base dyn
    // Sv * (A * qddot_cmd + _coriolis + _grav) = Sv * (Jc^{T} * Fr)
    // where Sv is the select matrix that extract the first 6 rows of EoMs,
    // qddot_cmd = qddot + delta_q_fb,
    // Fr = Fr_des + delta_fr.
    
    if (_dim_rf > 0) {
    CE.block(0, 0, _dim_eq_cstr, _dim_floating) =
      _A.block(0, 0, _dim_floating, _dim_floating);
    CE.block(0, _dim_floating, _dim_eq_cstr, _dim_rf) =
      -Sv_ * _Jc.transpose();
    ce0 = -Sv_ * (_A * qddot + _coriolis + _grav -
        _Jc.transpose() * _Fr_des);
    } else {
        CE.block(0, 0, _dim_eq_cstr, _dim_floating) =
        _A.block(0, 0, _dim_floating, _dim_floating);
        ce0 = -Sv_ * (_A * qddot + _coriolis + _grav);
    }

    // std::cout << "_Fr_des = (" << _Fr_des.rows() << "," << _Fr_des.cols() << ") = " << std::endl;
    // std::cout << _Fr_des.transpose() << std::endl;
}

double WBIController::_SolveQuadraticProgramming(Eigen::VectorXd& z) {
    z = DVec<double>::Zero(_dim_opt);
    double cost = 0.;
    drake::solvers::MathematicalProgram prog;
    drake::solvers::MosekSolver mosek_solver;
    // optimal decision variables
    // TODO: the dim of NewContinuousVariables must be a constant?
    Eigen::Matrix<drake::symbolic::Variable, -1, 1> x_star;
    switch (_dim_opt)
    {
    case 6: x_star = prog.NewContinuousVariables<6>(); break;
    case 6 + 1*3: x_star = prog.NewContinuousVariables<9>(); break;
    case 6 + 2*3: x_star = prog.NewContinuousVariables<12>(); break;
    default:  drake::log()->warn("ERROR opt dim");
        break;
    }
    
    auto qp_cost = prog.AddQuadraticCost(G, g0, x_star);

    // std::cout << "G = (" << G.rows() << "," << G.cols() << ") = " << std::endl;
    // std::cout << G << std::endl;
    
    
    if (_dim_rf > 0) {
        DVec<double> ub;
        ub = DVec<double>::Ones(_dim_Uf) * std::numeric_limits<double>::infinity();
        auto inEq_constraint = prog.AddLinearConstraint(CI, ci0, ub, x_star);
    }

    

    
    
    
    
    
    // auto inEq_constraint = prog.AddLinearConstraint(CI, lb, ub, x_star);
    // std::cout << "CI = (" << CI.rows() << "," << CI.cols() << ") = " << std::endl;
    // std::cout << CI << std::endl;
    
    auto eq_constraint = prog.AddLinearEqualityConstraint(CE, ce0, x_star);
    // std::cout << "CE = (" << CE.rows() << "," << CE.cols() << ") = " << std::endl;
    // std::cout << CE << std::endl;
    // std::cout << "ce0 = (" << ce0.rows() << "," << ce0.cols() << ") = " << std::endl;
    // std::cout << ce0 << std::endl;

    if (mosek_solver.available()) {
        drake::solvers::MathematicalProgramResult prog_result;
        mosek_solver.Solve(prog, initial_guess_vec, {}, &prog_result);
        if (prog_result.is_success()) {
            // drake::log()->info("congras!");
            z = prog_result.GetSolution();
            initial_guess_vec = z;
            cost = prog_result.get_optimal_cost();
            // const drake::solvers::MosekSolverDetails& mosek_solver_details =
            //     prog_result.get_solver_details<drake::solvers::MosekSolver>();
            // drake::log()->info("optimizer time: " + std::to_string(mosek_solver_details.optimizer_time));
        }
        else {
            drake::log()->warn("fail to find a result...");
        }
        
    } else {
        drake::log()->warn("mosek solver is not available !");
    }


    return cost;
}

void WBIController::_InverseDyn(const DVec<double>& qddot_original, DVec<double>& tao_j) {

    DVec<double> total_tau; // generalized torques
    DVec<double> Fr; //final contact forces
    DVec<double> qddot_cmd;

    total_tau.resize(num_qdot_); total_tau.setZero();
    Fr.resize(_dim_rf); Fr.setZero();
    qddot_cmd.resize(num_qdot_); qddot_cmd.setZero();

    qddot_cmd = qddot_original;

    for (size_t i(0); i < _dim_floating; ++i)
        qddot_cmd[i] = qddot_original[i] + z_star[i];

    // qddot_cmd.head(6).setZero();
    // qddot_cmd[0] = 0.0;   // roll
    // qddot_cmd[1] = 0.0; // pitch
    // qddot_cmd[2] = 0.0; // yaw
    qddot_cmd[3] = 0.0; // x
    qddot_cmd[4] = 0.0; // y 
    // qddot_cmd[5] = 20.0; // z

    if (_dim_rf > 0) {
        for (size_t i(0); i < _dim_rf; ++i)
            Fr[i] = z_star[i + _dim_floating] + _Fr_des[i];
        
        total_tau = 
            _A * qddot_cmd + _coriolis + _grav - _Jc.transpose() * Fr;
        total_tau = 
            _A * qddot_original + _coriolis + _grav - _Jc.transpose() * Fr;
        // total_tau = 
        //     - _Jc.transpose() * _Fr_des;
    } else {
        total_tau = _A * qddot_cmd + _coriolis + _grav;
        // total_tau.setZero();
    }

    // std::cout << "qddot_original = " << qddot_original.transpose() << std::endl;
    // std::cout << "qddot_cmd = " << qddot_cmd.transpose() << std::endl;
    // // std::cout << "A = (" << _A.rows() << "," << _A.cols() << ")" << " = " << std::endl << _A << std::endl; 
    // std::cout << "---" << std::endl;

    // std::cout << "_A * qddot_cmd  = " << (total_tau).transpose() << std::endl;
    std::cout << "before fmpc = " << _Fr_des.transpose() << ". After wbc fr = " << Fr.transpose() << std::endl;

    tao_j = total_tau.tail(num_act_joint_); 
}

void WBIController::update_command(CentaurStates& state, const DVec<double>& qj, const DVec<double>& qj_dot, const DVec<double>& tau) {

    state.wbc_q_cmd = qj;
    state.wbc_qdot_cmd = qj_dot;
    state.wbc_tau_ff = tau;

}