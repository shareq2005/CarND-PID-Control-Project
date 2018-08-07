#ifndef PID_H
#define PID_H

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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
   * Returns a steer value based on the cte provided
   */
  double GetSteerValue();

private:
  double dpSum();
  void AddDpAtCurrentParamIndex(double dP);
  
private:
  enum Twiddle_state
  {
    TWIDDLE_INIT_STATE = 0,
    TWIDDLE_ADD_STATE = 1,
    TWIDDLE_SUB_STATE = 2
  };
  
  const double m_tolerance = 0.0001;
  
  // number of evaluation stpes
  const int m_num_eval_steps = 5;
  
  const int m_num_skip_steps = 1;

  // twiddle parameter
  bool m_do_twiddle;
  
  Twiddle_state m_twiddle_state;
  
  // the current parameter index we're adding/subtracting
  int m_param_index;
  
  // potential change array
  double m_dp[3];
  
  // our best error yet
  double m_best_err;
  
  // total error
  double m_total_err;
  
  // the current step
  int m_step;
  
  // num steps for
};

#endif /* PID_H */
