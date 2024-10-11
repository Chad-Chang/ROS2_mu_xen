#pragma once
#include <stdio.h>
#include <iostream>
#define RNUMOFSLAVE 13;
#define SNUMOFSLAVE 12;
#define NUMOFLEG 4;
class Filter
{
    
public:
    explicit Filter(double sampling_time);
    virtual ~Filter();
    static double tustin_derivative(double input, double input_old, double output_old, double cutoff_freq);
    static double lowpassfilter(double input, double input_old, double output_old, double cutoff_freq);
    static double integral(double input, double input_old, double output_old);

private:
    static double dt_;
    static double pi_;
};