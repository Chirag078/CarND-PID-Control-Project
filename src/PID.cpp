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
  Kd = Kd_;
  Ki = Ki_;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  static double prev_cte = cte;
  static double total_cte = 0;
	P_Error = cte;
    
    D_Error = cte - prev_cte;
    prev_cte = cte;
    
  	total_cte += cte;
    I_Error = total_cte;
  
}

double PID::TotalError() {
  
  /**
   * TODO: Calculate and return the total error
   */
  return -Kp*P_Error -Kd*D_Error - Ki*I_Error;  // TODO: Add your total error calc here!
}