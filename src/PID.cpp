#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
   Kp = Kp_;
   Ki = Ki_;
   Kd = Kd_;
  p_error = 0;
   i_error = 0;
  d_error =0;
  first_flag = 1;
}

void PID::Pid_set(const double p[3])
{
   Kp = p[0];
   Ki = p[1];
   Kd = p[2];
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  if(first_flag)
  {
    cte_last = cte; 
    first_flag = 0;
  }
   p_error = Kp * cte;
   i_error += Ki * cte;
   d_error = Kd * (cte-cte_last);
   cte_last = cte; 
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double error_sum = -p_error - i_error - d_error;
  return error_sum;  // TODO: Add your total error calc here!
}