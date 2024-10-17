#include "controller.hpp"

Controller::Controller(LegModel *leg_model)
    :leg_model_(leg_model)
{
    leg_num_= leg_model_ -> leg_num;
    ctrl_update();
}

Controller::~Controller(){}

void Controller::ctrl_update()
{
    
    leg_num_= leg_model_ -> leg_num;
    // std::cout << "controller leg num = "<< leg_num_ << std::endl;
    //PID pos
    rwpos_err_old = rwpos_err; 
    drwpos_err_old = drwpos_err;

    //PID vel
    rwvel_err_old = rwvel_err;
    drwvel_err_old = drwvel_err;
    
    //DOB
    rw_dob_lhs_old = rw_dob_lhs;
    rw_dob_rhs_old = rw_dob_rhs;

    //FOB
    rw_fob_lhs_old = rw_fob_lhs;
    rw_fob_rhs_old = rw_fob_rhs;
    
    //admittance
    delta_pos_old2 = delta_pos_old;
    delta_pos_old = delta_pos;
    

}

Vector2d Controller::rw_pos_pid()
{
    rwpos_err = leg_model_->posRW_ref-leg_model_->posRW;
    for(int i = 0 ; i<NDOF_LEG; i++ )
        {
            drwpos_err[i] = tustin_derivative(rwpos_err[i], rwpos_err_old[i], drwpos_err[i],Qd_rw);
            rw_pos_o[i] = Kp_rw[i]*rwpos_err[i]+Kd_rw[i]*drwpos_err[i];
        }
    return rw_pos_o;
}

Vector2d Controller::rw_vel_pid()
{
    rwvel_err = leg_model_->velRW_ref-leg_model_->velRW;
    for(int i = 0 ; i<NDOF_LEG; i++ )
        {
            drwvel_err[i] = tustin_derivative(rwvel_err[i], rwvel_err_old[i], drwvel_err_old[i],Qd_vel_rw);
            irwvel_err[i] = integral(rwvel_err[i], rwvel_err_old[i], irwvel_err_old[i]);
            rw_vel_o[i] = Kp_rw[i]*rwvel_err[i]+Kd_rw[i]*drwvel_err[i]+ Ki_rw[i]*irwvel_err[i];
        }
    return rw_vel_o;
}

Vector2d Controller::DOBRW()
{
    
    rw_dob_lhs = leg_model_ -> torque_bi;
    rw_dob_rhs = leg_model_-> Lamda_nominal_DOB * leg_model_-> qddot_bi;
    if(leg_model_->rwdob_on)
    {
        for(int i = 0 ; i< NDOF_LEG; i++)
        {
            rw_dob_lhs_LPF[i] = lowpassfilter(rw_dob_lhs[i], rw_dob_lhs_old[i], rw_dob_lhs_LPF[i], Q_rwDOB);
            rw_dob_rhs_LPF[i] = lowpassfilter(rw_dob_rhs[i], rw_dob_rhs_old[i], rw_dob_rhs_LPF[i], Q_rwDOB);
            d_hat_tau[i] = rw_dob_rhs_LPF[i] - rw_dob_lhs_LPF[i];
        }
    }
    else
    {
        for(int i = 0 ; i <NDOF_LEG; i++)
            d_hat_tau[i];
    }
    return d_hat_tau;
}

void Controller::FOBRW()
{
    rw_fob_lhs = leg_model_ -> torque_bi;
    rw_fob_rhs = leg_model_->Lamda_nominal_FOB* leg_model_->qddot_bi;
    for (int i = 0; i < NDOF_LEG; i++)
    {
        rw_fob_lhs_LPF[i] = lowpassfilter(rw_fob_lhs[i], rw_fob_lhs_old[i], rw_fob_lhs_LPF[i], Q_rwFOB);
        rw_fob_rhs_LPF[i] = lowpassfilter(rw_fob_rhs[i], rw_fob_rhs_old[i], rw_fob_rhs_LPF[i], Q_rwFOB);
        tau_hat[i] = rw_fob_rhs_LPF[i] - rw_fob_lhs_LPF[i];
    }
}
