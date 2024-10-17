#include "kinodynamics.hpp"


// Legnum : FL(0),FR(1),RR(2), RL(3) 
// simulation과 실제 플랜트의 모델이 차이가남. -> 경우에 따라 다르게 해야할 듯
Kinodynamics::Kinodynamics(MsgHandler *msg_handler, LegModel *leg_model)
       :message_handler_(msg_handler), leg_model_(leg_model)
{
    leg_num_ = leg_model_->leg_num;
    // if(is_biarticular_) message_handler_ = std::static_pointer_cast<MsgHandler::RealRobotState>(msg_handler -> get_real_cmd_ptr());
    // else  message_handler_ = std::static_pointer_cast<MsgHandler::SimRobotState>(msg_handler -> get_sim_cmd_ptr());
    /* Trunk Parameter */
    m_hip_ = 2.5;
    m_trunk_front_ = 10.;
    m_trunk_rear_ = 18.;
    m_trunk_ = 4 * m_hip_ + m_trunk_front_ + m_trunk_rear_;

    /* Leg Parameters */
    L_ = 0.25;
    d_thigh_ = 0.11017; // local position of CoM of thigh
    d_shank_ = 0.12997; // local position of CoM of shank
    // printf("d_thigh : %f, d_shank : %f \n", d_thigh, d_shank);

    m_thigh_ = 1.017; // mass of thigh link
    m_shank_ = 0.143; // mass of shank link
    m_leg_ = m_thigh_ + m_shank_;
    m_total_ = m_trunk_ + 4 * m_leg_;
    // printf("m_thigh : %f, m_shank : %f \n", m_thigh, m_shank);

    Izz_thigh_ = 0.0057;     // MoI of thigh w.r.t. CoM
    Izz_shank_ = 8.0318e-04; // MoI of shank w.r.t. CoM
    // printf("Izz_thigh : %f, Izz_shank : %f \n", Izz_thigh, Izz_shank);

    Jzz_thigh_ = Izz_thigh_ + m_thigh_ * pow(d_thigh_, 2); // MoI of thigh w.r.t. HFE
    Jzz_shank_ = Izz_shank_ + m_shank_ * pow(d_shank_, 2); // MoI of thigh w.r.t. KFE
    // printf("Jzz_thigh : %f, Jzz_shank : %f \n", Jzz_thigh, Jzz_shank);
    
    leg_model_->Lamda_nominal_DOB(0,0)= 0.0;
    leg_model_->Lamda_nominal_DOB(0,1)= 0.0;
    leg_model_->Lamda_nominal_DOB(1,0)= 0.0;
    leg_model_->Lamda_nominal_DOB(1,1)= 0.0;
};

Kinodynamics::~Kinodynamics(){};

void Kinodynamics::sensor_update(bool is_biarticular)
{   
    is_biarticular_ = is_biarticular;
    leg_num_ = leg_model_->leg_num;
    std::cout << "kinematics leg num = "<< leg_num_ << std::endl;
    
    for (int i = 0; i < NDOF_LEG; i++)
    {
        // Joint
        leg_model_->q_bi_old[i] = leg_model_->q_bi[i];
        leg_model_->qdot_bi_old[i] = leg_model_->qdot_bi[i];
        leg_model_->qddot_bi_old[i] = leg_model_->qddot_bi[i];

        // Feedback - RW Kinematics
        leg_model_->posRW_old[i] = leg_model_->posRW[i];
        leg_model_->velRW_old[i] = leg_model_->velRW[i];
        leg_model_->posRW_ref_old2[i] = leg_model_->posRW_ref_old[i];
        leg_model_->posRW_ref_old[i] = leg_model_->posRW_ref[i];
        leg_model_->velRW_ref_old[i] = leg_model_->velRW_ref[i];
    }
    // 실제 로봇에서 들어오는 정보라면
    if(is_biarticular_)
    {   
        // auto real_state = std::static_pointer_cast<MsgHandler::RealRobotState>(message_handler_);
        auto real_state = message_handler_->get_real_cmd_ptr();
        std::cout << "vector size = " <<  real_state -> jpos.size()<<endl;
        if(real_state -> jpos.size() == 13)
        {
            if(leg_num_==0 || leg_num_ == 1) // FL, FR일때 
            {
                
                leg_model_ -> q_abd = 1;//real_state -> jpos[5*leg_num_];
                real_state -> jpos[5*leg_num_] = 1;
                leg_model_->q_bi[0] = real_state -> jpos[3*(leg_num_-1)+4];
                leg_model_->q_bi[1] = real_state -> jpos[leg_num_+2];

                leg_model_ -> q_abd = real_state -> jvel[5*leg_num_];
                leg_model_->qdot_bi[0] = real_state -> jvel[3*(leg_num_-1)+4];
                leg_model_->qdot_bi[1] = real_state -> jvel[leg_num_+2];
            }
            else if(leg_num_==2 || leg_num_ == 3) // FL, FR일때 
            {
                leg_model_ -> q_abd = real_state -> jpos[5*(leg_num_-2)+6];
                leg_model_->q_bi[0] = real_state -> jpos[3*(leg_num_-2)+7];
                leg_model_->q_bi[1] = real_state -> jpos[leg_num_+6];
                
                leg_model_ -> qdot_abd = real_state -> jpos[5*(leg_num_-2)+6];
                leg_model_->qdot_bi[0] = real_state -> jvel[3*(leg_num_-2)+7];
                leg_model_->qdot_bi[1] = real_state -> jvel[leg_num_+6];
            }
        }
    }
    else // if simulation
    {
        auto sim_state = message_handler_->get_sim_cmd_ptr();
        std::cout << "vector size = " <<  sim_state -> jpos.size()<<endl;
        if(sim_state -> jpos.size() == 12)
        {
            if(leg_num_==0 || leg_num_ == 1) // FL, FR일때 
            {
                leg_model_ -> q_abd = sim_state -> jpos[3*leg_num_+7];
                leg_model_->q_bi[0] = sim_state -> jpos[3*(leg_num_)+8];
                leg_model_->q_bi[1] = 
                    sim_state -> jpos[3*(leg_num_)+9] +sim_state -> jpos[3*(leg_num_)+8];

                leg_model_ -> qdot_abd = sim_state -> jvel[3*(leg_num_)+6];
                leg_model_->qdot_bi[0] = sim_state -> jvel[3*(leg_num_)+7];
                leg_model_->qdot_bi[1] = 
                    sim_state -> jvel[3*(leg_num_)+8] +sim_state -> jvel[3*(leg_num_)+7];
            }
            else if(leg_num_==2 || leg_num_ == 3) // FL, FR일때 
            {
                leg_model_ -> q_abd = sim_state -> jpos[-3*(leg_num_-2)+16];
                leg_model_->q_bi[0] = sim_state -> jpos[-3*(leg_num_-2)+17];
                leg_model_->q_bi[1] = 
                    sim_state -> jpos[-3*(leg_num_-2)+17] +sim_state -> jpos[-3*(leg_num_-2)+18];

                leg_model_ -> qdot_abd = sim_state -> jvel[-3*(leg_num_-2)+15];
                leg_model_->qdot_bi[0] = sim_state -> jvel[-3*(leg_num_-2)+16];
                leg_model_->qdot_bi[1] = 
                    sim_state -> jvel[-3*(leg_num_-2)+16] + sim_state -> jvel[-3*(leg_num_-2)+17];
            }
        }
    }
    leg_model_-> q_br = leg_model_ -> q_bi[1]-leg_model_->q_bi[0];
}
void Kinodynamics::FK()
{
    /*** Rotating Workspace ***/
    leg_model_->jacbRW(0,0) =  L_ * sin(leg_model_->q_br / 2);
    leg_model_->jacbRW(0,1) = -L_ * sin(leg_model_->q_br / 2);
    leg_model_->jacbRW(1,0) =  L_ * cos(leg_model_->q_br / 2);
    leg_model_->jacbRW(1,1) =  L_ * cos(leg_model_->q_br / 2);
    
    leg_model_->jacbRW_trans = leg_model_->jacbRW.transpose(); 

    leg_model_ -> jacbRW_trans_inv = leg_model_ -> jacbRW_trans.inverse();
    leg_model_ -> posRW[0] = 2 * L_ * cos((leg_model_ -> q_bi[1] - leg_model_ -> q_bi[0]) / 2); // r
    leg_model_ -> posRW[1] = (leg_model_ -> q_bi[0] + leg_model_ -> q_bi[1]) / 2;                           // qr
    leg_model_ -> velRW = leg_model_ -> jacbRW * leg_model_ -> qdot_bi;
    
};
void Kinodynamics::model_update()
{
    double M1 = Jzz_thigh_ + m_shank_ * pow(L_, 2);
    double M2 = m_shank_ * d_shank_ * L_ * cos(leg_model_->q_br);
    double M12 = Jzz_shank_;


    leg_model_->Lamda_nominal_FOB(0,0) = M1;
    leg_model_->Lamda_nominal_FOB(0,1) = M12;
    leg_model_->Lamda_nominal_FOB(1,0) = M12;
    leg_model_->Lamda_nominal_FOB(1,1) = M2;

    JzzR_thigh_  = Jzz_thigh_ + Jzz_shank_ + m_shank_ * pow(L_, 2) 
        - 2 * m_shank_ * d_shank_ * L_ * cos(leg_model_-> q_br);
    JzzR_couple_ = Jzz_thigh_ + m_shank_ * pow(L_, 2) - Jzz_shank_;
    JzzR_shank_ = Jzz_thigh_ + Jzz_shank_ + m_shank_ * pow(L_, 2) 
        + 2 * m_shank_ * d_shank_ * L_ * cos(leg_model_-> q_br);

    MatInertia_RW_(0,0) = JzzR_thigh_ / (4 * pow(L_, 2) * pow(sin(leg_model_->q_br / 2), 2));
    MatInertia_RW_(0,1) = JzzR_couple_ / (2 * pow(L_, 2) * sin(leg_model_->q_br));
    MatInertia_RW_(1,0) = JzzR_couple_ / (2 * pow(L_, 2) * sin(leg_model_->q_br));
    MatInertia_RW_(1,1) = JzzR_shank_ / (4 * pow(L_, 2) * pow(cos(leg_model_->q_br / 2), 2));
        
    
    Inertia_DOB_(0,0) = MatInertia_RW_(0,0);
    Inertia_DOB_(0,1) = 0;
    Inertia_DOB_(1,0) = 0;
    Inertia_DOB_(1,1) = MatInertia_RW_(1,1);
    
    leg_model_->Lamda_nominal_DOB = leg_model_->jacbRW_trans*Inertia_DOB_*leg_model_->jacbRW;

    coriolis_bi_[0] = -m_shank_*d_shank_*L_*sin(leg_model_->q_br)*pow(leg_model_->qddot_bi[1],2);
    coriolis_bi_[1] = m_thigh_*d_shank_*L_*sin(leg_model_->q_br) * pow(leg_model_->qdot_bi[0], 2);
    gravity_bi_[0] = g * (m_thigh_ * d_thigh_ + m_shank_ * L_) * cos(leg_model_->q_bi[0]);
    gravity_bi_[1] = g * m_shank_ * d_shank_ * cos(leg_model_->q_bi[2]);

    off_diag_inertia_bi_(0,0)= 0;
    off_diag_inertia_bi_(0,1)= m_shank_*d_shank_*L_*cos(leg_model_->q_br);
    off_diag_inertia_bi_(1,0)= off_diag_inertia_bi_(0,1);
    off_diag_inertia_bi_(1,1)= 0;
    
    // leg_model_ -> corriolis_bi_torq = coriolis_bi_;
    // leg_model_ -> gravity_bi_torq = gravity_bi_;
    // leg_model_ -> off_diag_inertia_bi = off_diag_inertia_bi_;

}


void Kinodynamics::jnt_acc_update()
{

}