#pragma once
#include "leg_model.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include "filter.hpp"
#include <stdio.h>
#include <iostream>
#include <vector>
#include "msg_handler.hpp"

using namespace Eigen;

class Kinodynamics : public Filter
{
public:
    explicit Kinodynamics(MsgHandler *msg_handler, LegModel *leg_model );
    virtual ~Kinodynamics();

    void jnt_acc_update(); // simulation, 실제 가속도 계산 -> 시뮬에서는 RT가 안 되니까
    void model_update();
    void sensor_update(bool is_biarticular);
    void FK();
    bool is_biarticular_;
    int leg_num_;
    // void init_state(LegModel *leg_model);
private:
    // std::shared_ptr<void> message_handler_; // 자료형 SimRobotState 또는 RealRobotState    
    std::shared_ptr<MsgHandler> message_handler_;
    LegModel *leg_model_;


    /* Trunk Parameter */
    double m_hip_;         // mass of hip torso
    double m_trunk_front_; // mass of front trunk
    double m_trunk_rear_;  // mass of rear trunk
    double m_trunk_;       // total mass of trunk
    double m_total_;       // total robot mass

    /* Leg Parameter */  
    double L_; // leg length : thigh and shank links' length are assumed to be the same

    double m_thigh_; // mass of thigh link
    double m_shank_; // mass of shank link
    double m_leg_;   // mass of leg

    double d_thigh_; // CoM pos of thigh w.r.t HFE
    double d_shank_; // CoM pos of shank w.r.t KFE

    double Izz_thigh_; // MoI(z) of thigh w.r.t its CoM
    double Izz_shank_; // MoI(z) of shank w.r.t its CoM

    double Jzz_thigh_; // MoI(z) of thigh w.r.t HFE
    double Jzz_shank_; // MoI(z) of shank w.r.t KFE

    double JzzR_thigh_;
    double JzzR_shank_;
    double JzzR_couple_;

    Vector2d coriolis_bi_;
    Vector2d gravity_bi_;
    Matrix2d off_diag_inertia_bi_;

    Matrix2d MatInertia_bi_;
    Matrix2d MatInertia_RW_;
    Matrix2d Inertia_DOB_;

    Vector2d H_;
    Vector2d H_old_;

};