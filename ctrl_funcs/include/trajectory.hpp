#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include "leg_model.hpp"
#include <stdio.h>
#include <iostream>
#include <iostream>

using namespace std;
class Trajectory
{
    private:
        

        LegModel *leg_model_;
    public:
        int leg_num_;
        int test_ ; 
        Trajectory(LegModel* leg_model);
        ~Trajectory();
        // void Squat(double t,LegModel* state_model);
        void hold();
};


