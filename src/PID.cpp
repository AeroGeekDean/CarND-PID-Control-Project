#include "PID.h"

//using namespace std; // poor coding practice!

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  ResetLimits();
}

PID::~PID() {

}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::SetLimits(double output_lim, double int_auth_lim) {
  this->output_limit = output_lim;
  this->int_err_limit = output_lim*int_auth_lim/Ki; // back calc limit for i_error
}

void PID::ResetLimits() {
  output_limit = INF;
  int_err_limit = INF;
}

void PID::Reset(double cte) {
  d_error = 0.0;
  i_error = 0.0;
  p_error = cte;
}

void PID::UpdateError(double dt, double cte) {
  d_error = (cte- p_error)/dt;
  i_error += cte*dt;
  p_error = cte;

  // Limit the integrator error to prevent windup.
  // See here: https://en.wikipedia.org/wiki/Integral_windup
  if (i_error > int_err_limit)
  {
    i_error = int_err_limit;
    isIntegratorSaturated = true;
  }
  else if (i_error < -int_err_limit)
  {
    i_error = -int_err_limit;
    isIntegratorSaturated = true;
  }
  else
  {
    isIntegratorSaturated = false;
  }
}

double PID::TotalError() {
  double output = -(Kp*p_error + Ki*i_error + Kd*d_error);

  // apply output limits
  if (output > output_limit)
    output = output_limit;
  else if (output < -output_limit)
    output = -output_limit;

  return output;
}

