/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
public:

    // Create the PID class, modify section until END OF TODO
    /*
    * Errors
    */
    double p_error; //cte
    double i_error; //cumulative cte
    double d_error; //differential cte
    double previous_cte;

    /*
    * Coefficients
    */
    double Kp;
    double Ki;
    double Kd;

    /*
    * Output limits for throttle
    */
    const double throttle_output_lim_max{1.0};
    const double throttle_output_lim_min{-1.0};

    /*
    * Output limits for throttle
    */
    const double steer_output_lim_max{1.2};
    const double steer_output_lim_min{-1.2};

  
    /*
    * Delta time
    */
    double delta_time;


    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();
  
    /*
    * Update the delta time.
    */
    double UpdateDeltaTime(double new_delta_time);
};

#endif //PID_CONTROLLER_H


