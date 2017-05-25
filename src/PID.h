#ifndef PID_H
#define PID_H

#include <vector>

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

  bool twiddle_it;
  int counter;

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
  void Init(double Kp, double Ki, double Kd, bool twiddle_it);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  * @param steering_angle will ensure that the returned error is between the range -1 and 1
  */
  double TotalError(bool steering_angle);


  void Twiddle(double cte);
  /*
  * Set the internal errors (used for the twiddle method)
  */
  void SetErrors(double p_error, double i_error,double d_error);


};

#endif /* PID_H */
