#include "trajectory.hpp"

Trajectory::Trajectory(LegModel* leg_model): leg_model_(leg_model)
{
    leg_num_ = leg_model->leg_num;
}

Trajectory::~Trajectory(){}
void Trajectory::hold()
{   
    leg_num_ = leg_model_->leg_num;
    // std::cout << "trajectory leg num = "<< leg_num_ << std::endl;
    leg_model_->posRW_ref[0] = 0.3536;
    leg_model_->posRW_ref[1] = pi / 2;
}