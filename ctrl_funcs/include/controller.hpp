#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include "leg_model.hpp"
#include "filter.hpp"
#include <stdio.h>
#include <iostream>
#include <vector>
#include "msg_handler.hpp"

using namespace Eigen;

class Controller : public Filter
{
public:
    explicit Controller(LegModel *leg_model );
    virtual ~Controller();

    // 내부에서 PID게인 세팅하기
    void ctrl_update(); // delay update
    void FOBRW(); // cutoff GUI에서
    // void admittance(); // 
    Vector2d rw_pos_pid(); // 이전 control값 기억
    Vector2d rw_vel_pid();
    Vector2d DOBRW(); // cutoff GUI에서
    bool is_biarticular_;
    int leg_num_;

private:
//leg model structure
    LegModel *leg_model_;
    
    
    

// rw pid : 2x1 [첫번째 R방향 gain, 두번째 theta방향 gain]
    // rwpos msg
    Vector2d Kp_rw{0,0}; Vector2d Ki_rw{0,0};  Vector2d Kd_rw{0,0}; 
    double Qd_rw = 0.1;
    // rwvel msg
    Vector2d Kp_vel_rw{0,0}; Vector2d Ki_vel_rw{0,0}; Vector2d Kd_vel_rw{0,0};
    double Qd_vel_rw{0.1};
    
// // DOB parameter : joint Q: 3x1 / RW, Ori Q:1x1
//     //rw DOB, rw simulation DOB
    double Q_rwDOB{0.1}; double Q_rwFOB{0.1};
//     //orientation DOB, orientation simulatino DOB
    double Q_oriDOB{0.1};

// // admittance parameter : 3x1 : [omega, zeta, k] or [m, b, k]
    Vector3d admit_param_f{0.1,0.1,0.1}; // 2차 필터 형태 [omega, zeta, k]
    Vector3d admit_param_mbk{0.1,0.1,0.1}; // mbk모델 [m, b, k]
    Vector2d delta_pos{0,0};
    Vector2d delta_pos_old{0,0};
    Vector2d delta_pos_old2{0,0};

// error variable
    // simulattion error
        // pos err
    Vector2d rwpos_err; Vector2d rwpos_err_old;
    Vector2d drwpos_err; Vector2d drwpos_err_old;
        // vel err
    Vector2d rwvel_err; Vector2d rwvel_err_old;
    Vector2d drwvel_err; Vector2d drwvel_err_old;
    Vector2d irwvel_err; Vector2d irwvel_err_old;
    
// control variable
    Vector2d rw_pos_o; Vector2d rw_vel_o;
    
// dob variables : 2x1 
    Vector2d rw_dob_rhs; Vector2d rw_dob_rhs_old;
    Vector2d rw_dob_lhs; Vector2d rw_dob_lhs_old;
    Vector2d rw_dob_rhs_LPF; Vector2d rw_dob_rhs_LPF_old;
    Vector2d rw_dob_lhs_LPF; Vector2d rw_dob_lhs_LPF_old;
    Vector2d d_hat_tau;

// FOB variables
    Vector2d rw_fob_rhs; Vector2d rw_fob_rhs_old;
    Vector2d rw_fob_lhs; Vector2d rw_fob_lhs_old;
    Vector2d rw_fob_rhs_LPF; Vector2d rw_fob_rhs_LPF_old;
    Vector2d rw_fob_lhs_LPF; Vector2d rw_fob_lhs_LPF_old;
    Vector2d tau_hat; Vector2d force_hat;
    

};