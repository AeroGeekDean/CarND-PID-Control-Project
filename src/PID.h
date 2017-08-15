#ifndef PID_H
#define PID_H

/*----------------------------------
 * Note:
 * "LinearInterpolation1D.h" was included below because I was originally going to implement
 * controller gain scheduling, based on vehicle speed...
 * https://en.wikipedia.org/wiki/Gain_scheduling
 * https://www.mathworks.com/help/control/ug/gain-scheduled-control-systems.html
 *
 * However, after init testing it appears the simulator's reference trajectory may contain
 * discontinuities, in either CTE and/or d(CTE)/dx. Plus I was able to easily manual-tune
 * a set of PID gains to navigate the car around the track at 20 mph. Thus did not implement
 * the gain scheduling strategy.
 */
#include "LinearInterpolate1D.h"
//-----------------------------------


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

  double int_err_limit;  // Integrator output will be limited to this fraction of total output.
                         // This prevents integrator "wind-up" issue. See .cpp for details...

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
   * output_lim = max magnitude of output
   * int_lim = integrator path authority, range [0 to 1]
   */
  void SetLimits(double output_lim, double int_auth_lim);

  /*
   * Reset controller limits to INF
   */
  void ResetLimits();

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
