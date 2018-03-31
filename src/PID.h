#ifndef PID_H
#define PID_H

class PID {
public:
  int steps;
  double p_error_s,i_error_s,d_error_s,p_error_t,d_error_t,error;   // Errors
  double Kp_s,Ki_s,Kd_s,Kp_t,Kd_t;   // Coefficients
  PID();
  virtual ~PID();
  void Init(double kp_s, double ki_s, double kd_s,double kp_t,double kd_t);
  // Update the PID error variables given cross track error.
  void UpdateError(double cte);
  double AvgError();
  void GetControls(double& steering,double& throttle);
};

#endif /* PID_H */
