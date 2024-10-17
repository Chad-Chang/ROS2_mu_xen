#pragma once
#include <stdio.h>
#include <iostream>
#define RNUMOFSLAVE 13
#define SNUMOFSLAVE 12
#define NUMOFLEG 4
#define NDOFLEG 2
#include "msg_handler.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace Eigen;

class Filter
{
    
public:
    explicit Filter();
    virtual ~Filter();
    static double tustin_derivative(double input, double input_old, double output_old, double cutoff_freq);
    static double lowpassfilter(double input, double input_old, double output_old, double cutoff_freq);
    static double integral(double input, double input_old, double output_old);
    // static Matrix2d TF_SeriQ2BiQ(Vector2d serial_q);
    // static Matrix2d TF_seri2bi(Vector2d serial_q);
private:
    static double dt_;
    static double pi_;
};