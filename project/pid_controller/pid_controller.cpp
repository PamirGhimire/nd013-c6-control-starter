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

void PID::Init(double Kpi, double Kii, double Kdi) {
    Kp = Kpi;
    Ki = Kii;
    Kd = Kdi;

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    previous_cte = 0.0;
    delta_time = 0.0;
}


void PID::UpdateError(double cte) {
    d_error = delta_time > 0.0 ? (cte - previous_cte) / delta_time : 0.0;
    p_error = cte;
    i_error += cte * delta_time;
    previous_cte = cte;
}

double PID::TotalError() {
    double control = -(Kp * p_error + Ki * i_error + Kd * d_error);
    if (control > throttle_output_lim_max) {
        return throttle_output_lim_max;
    }
    if (control < throttle_output_lim_min) {
        return throttle_output_lim_min;
    }
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
    delta_time = new_delta_time;
    return delta_time;
}
