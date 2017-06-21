#ifndef PID_H
#define PID_H

class PID {
protected:
  /*
  * Errors
  */
  double p_error = 0;
  double i_error = 0;
  double d_error = 0;
  double sum_error = 0;

  // step counter and minimal steps for twiddle
  int steps = 0;
  int min_steps = 100;

  // use two help-variables. The first is to control where in the twiddle operation you are. The second one
  // controls if you work currently on 'p', 'i' or 'd' in twiddle.
  int help_1 = 0;
  char help_2 = 'p';

  // initialize the best error
  double best_error = 1e12;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  double dpp;
  double dpi;
  double dpd;

  public:

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  double getKp() {return Kp;}
  double getKi() {return Ki;}
  double getKd() {return Kd;}

  double getValue();

  double getValue_speed();

  int getNumberOfSteps() { return steps; }

  void setNumberOfSteps(int num) { steps = num; }

  /*

  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void twiddle();
};

#endif /* PID_H */
