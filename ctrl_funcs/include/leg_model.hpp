#pragma once

#include <stdbool.h>
#include <stdio.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#define NDOF_TRUNK 3 // #(DoF) of trunk
#define NDOF_LEG 2   // #(DoF) of leg
#define NUM_LEG 4

const double Ts = 0.0001; // sampling period
const double g = 9.81;    // gravitational accel.
const double pi = 3.141592;

using namespace Eigen;
using namespace std;

struct LegModel{
    LegModel(int leg_num_):leg_num(leg_num_){};
    int leg_num;
    /* joint */
    double q_br;
    double q_abd; // add/abd
    double qdot_abd; // add/abd

    bool rwdob_on;

    Vector2d q_bi;    // Serial Coordinates
    Vector2d q_bi_old; // gravity,coriolis
    
    Vector2d qdot_bi;
    Vector2d qdot_bi_old;
    Vector2d qddot_bi;
    Vector2d qddot_bi_old;

    Vector2d torque_bi;
    Vector2d torque_bi_old;

    /* Rotating Workspace Coordinates */
    Vector2d posRW; // RW position
    Vector2d posRW_old;
    
    Vector2d posRW_ref; // RW position reference
    Vector2d posRW_ref_old;
    Vector2d posRW_ref_old2;

    Vector2d posRW_des; // not influenced by admittance

    Vector2d velRW; // RW velocity
    Vector2d velRW_old;
    Vector2d velRW_ref; // RW velocity reference
    Vector2d velRW_ref_old;

    Vector2d ctrl_input_RW; // control input
    Vector2d ctrl_input_RW_old;

    /* Jacobian (Rotating Workspace) */
    Matrix2d jacbRW;
    Matrix2d jacbRW_trans;
    Matrix2d jacbRW_trans_inv;

    Matrix2d Lamda_nominal_DOB;
    Matrix2d Lamda_nominal_FOB;

    Vector2d H; // Coriolis & Gravity term
    Vector2d H_old;

    Matrix2d force_ext_hat; // FOB를 통해 측정된 힘.
    Matrix2d torque_ext_hat; // FOB를 통해 추정된 토크
};

