#include "PID.h"
#include <cmath>
using namespace std;
PID::PID() {}

PID::~PID() {}

void PID::Init(double kp_s, double ki_s, double kd_s,double kp_t,double kd_t) {
  Kp_s = kp_s;
  Ki_s = ki_s;
  Kd_s = kd_s;
  Kp_t = kp_t;
  Kd_t = kd_t;

  i_error_s = 0;
  d_error_s = 0;
  p_error_s = 0;
  d_error_t = 0;
  p_error_t = 0;

  error = 0;
  steps = 0;
}

void PID::UpdateError(double cte) {
  error += fabs(cte);
  i_error_s += cte;
  if(steps>0) {
    d_error_s = cte-p_error_s;
    d_error_t = fabs(cte)-p_error_t;
  } else {
    d_error_s = 0.0;
    d_error_t = 0.0;
  }
  p_error_s = cte;
  p_error_t = fabs(cte);
  steps++;
}

double PID::AvgError() {
  if(steps==0)
    return 9e99;
  return error/steps;
}


void PID::GetControls(double& steering,double& throttle) {
  steering= Kp_s*p_error_s+Ki_s*i_error_s+Kd_s*d_error_s;
  throttle= 0.2+Kp_t*p_error_t+Kd_t*d_error_t;
  if(throttle<0.1) throttle = 0.1;
  //  throttle=0.2;
}
