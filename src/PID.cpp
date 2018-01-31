#include "PID.h"
#include <cmath>
using namespace std;
PID::PID() {}

PID::~PID() {}

void PID::Init(double kp, double ki, double kd) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
  i_error = 0;
  d_error = 0;
  p_error = 0;
  error = 0;
  steps = 0;
}

void PID::UpdateError(double cte) {
  error += fabs(cte);
  i_error += cte;
  d_error = cte-p_error;
  p_error = cte;
  steps++;
}
double PID::AvgError() {
  if(steps==0)
    return 9e99;
  return error/steps;
}


double PID::TotalError() {
  return Kp*p_error+Ki*i_error+Kd*d_error;
}
