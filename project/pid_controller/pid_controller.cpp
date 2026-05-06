/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_min_spec, double output_lim_max_spec) {
    Kp = Kpi;
    Ki = Kii;
    Kd = Kdi;
    output_lim_min = output_lim_min_spec;
    output_lim_max = output_lim_max_spec;
    
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    previous_cte = 0.0;
    delta_time = 0.0;
}


void PID::UpdateError(double error) {
    d_error = delta_time > 0.0 ? (error - previous_cte) / delta_time : 0.0;
    p_error = error;
    i_error += error * delta_time;
    previous_cte = error;
}

double PID::TotalError() {
    double control = Kp * p_error + Ki * i_error + Kd * d_error;
    if (control > output_lim_max) {
        return output_lim_max;
    }
    if (control < output_lim_min) {
        return output_lim_min;
    }
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
    delta_time = new_delta_time;
    return delta_time;
}
