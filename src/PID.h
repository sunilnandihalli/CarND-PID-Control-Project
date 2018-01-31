#ifndef PID_H
#define PID_H

class PID {
public:
  int steps;
  double p_error,i_error,d_error,error;   // Errors
  double Kp,Ki,Kd;   // Coefficients
  PID();
  virtual ~PID();
  void Init(double Kp, double Ki, double Kd);
  // Update the PID error variables given cross track error.
  void UpdateError(double cte);
  double TotalError(); // Calculate the total PID error.
};

#endif /* PID_H */
