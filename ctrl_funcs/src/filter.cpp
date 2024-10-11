#include "filter.hpp"

double Filter::dt_; // 이거 쫌 보기 그렇다 .
double Filter::pi_;

Filter::Filter(double sampling_time)
{
    dt_ = sampling_time;
    pi_= 3.14159265358979323846;
}
Filter::~Filter(){}

double Filter::tustin_derivative(double input, double input_old, double output_old, double cutoff_freq)
{
    double time_const = 1 / (2 * pi_ * cutoff_freq);
    double output = 0;

    output = (2 * (input - input_old) - (dt_ - 2 * time_const) * output_old) / (dt_ + 2 * time_const);

    return output;
}

double Filter::lowpassfilter(double input, double input_old, double output_old, double cutoff_freq)
{
    //double cutoff_freq = 100;
    double time_const = 1 / (2 * pi_ * cutoff_freq);
    double output = 0;

    output = (dt_ * (input + input_old) - (dt_ - 2 * time_const) * output_old) / (dt_ + 2 * time_const);

    return output;
}

double Filter::integral(double input, double input_old, double output_old)
{
    double output = 0;
    output = output_old + dt_*(input + input_old)/2;
    return output;
}