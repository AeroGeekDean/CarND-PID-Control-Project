#ifndef PID_H
#define PID_H

#include "LinearInterpolate1D.h"

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  double output_limit;  // output will be capped at [+/-] of this value
  double int_err_limit;  // Integrator path will be limited at this fraction of total output.
                         // This prevents integrator "wind-up" issue.
  bool isIntegratorSaturated;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID coefficients
  */
  void Init(double Kp, double Ki, double Kd);

  /*
   * Set controller limits
   */
  void SetLimits(double output_lim, double int_lim);

  /*
  * Reset PID: dynamic by-pass / controller dynamic state synchronization
  */
  void Reset(double cte);

  /*
  * Update the PID error (state) variables given cross track error (input).
  */
  void UpdateError(double dt, double cte);

  /*
  * Calculate the total PID error (output).
  */
  double TotalError();

private:

};

#endif /* PID_H */
