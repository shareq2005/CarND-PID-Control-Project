#include "PID.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <cassert>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() : m_do_twiddle(false) {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

  // Initialize the coefficients
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  
  // Initialize the potential changes array
  m_dp[0] = 0.01;
  m_dp[1] = 0.01;
  m_dp[2] = 0.01;
  
  // Initialize the best error to max value
  m_best_err = std::numeric_limits<double>::max();
  
  // Initialize total error to 0.0
  m_total_err = 0.0;
  
  // Set the current twiddle state
  if(m_do_twiddle)
  {
    m_twiddle_state = Twiddle_state::TWIDDLE_INIT_STATE;
  }
  
  // Current parameter index
  m_param_index = 0;
  
  // Number of steps to 0
  m_step = 0;
}

void PID::AddDpAtCurrentParamIndex(double dP)
{
  if(m_param_index == 0)
  {
    Kp += dP;
  }
  else if(m_param_index == 1)
  {
    Kd += dP;
  }
  else if(m_param_index == 2)
  {
    Ki += dP;
  }
  else
  {
    assert(false);
  }
}

void PID::UpdateError(double cte)
{
  // if this is the first step
  if(m_step == 0)
  {
    p_error = cte;
  }
  
  // Update the PID errors
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
  
  // this is a slightly modified twiddle which ignores the I term
  // since we don't have a systematic bias
  if(m_do_twiddle && (dpSum() > m_tolerance))
  {
    // calculate the total error
    if(m_step % (m_num_eval_steps + m_num_skip_steps) > m_num_skip_steps)
    {
      m_total_err += pow(cte, 2);
    }
    
    if((m_step != 0) && (m_step % (m_num_skip_steps + m_num_eval_steps) == 0))
    {
      // Update the best error
      if(m_total_err < m_best_err)
      {
        m_best_err = m_total_err;
        m_dp[m_param_index] *= 1.1;
        m_param_index = (++m_param_index) % 2;
       
        cout << "Best error improved = " << m_total_err << endl;

        m_twiddle_state = TWIDDLE_INIT_STATE;
      }

      // Initial state
      if(m_twiddle_state == TWIDDLE_INIT_STATE)
      {
        cout << "Init state" << endl;
        
        // add dP to the parameter, and move to add state
        double dP = m_dp[m_param_index];
        AddDpAtCurrentParamIndex(dP);
        m_twiddle_state = TWIDDLE_ADD_STATE;
      }
      else if (m_twiddle_state == TWIDDLE_ADD_STATE)
      {
        cout << "Add state" << endl;
        
        // subtract dP and move to subtract state
        double dP = -2 * m_dp[m_param_index];
        AddDpAtCurrentParamIndex(dP);
        m_twiddle_state = TWIDDLE_SUB_STATE;
      }
      else if (m_twiddle_state == TWIDDLE_SUB_STATE)
      {
        cout << "Sub state" << endl;
        
        // reset the parameter P, and lower the dP
        double dP = m_dp[m_param_index];
        AddDpAtCurrentParamIndex(dP);
        
        m_dp[m_param_index] *= 0.9;
        m_param_index = (++m_param_index) % 2;
        
        // move to the init state
        m_twiddle_state = TWIDDLE_INIT_STATE;
      }
      
      cout << "Kp = " << Kp << ", Ki = " << Ki << ", Kd = " << Kd << endl;
      
      // reset the total error
      m_total_err = 0.0;
    }
  }

  m_step++;
}

double PID::GetSteerValue()
{
  double steer_value = - (Kp * p_error) - (Kd * d_error) - (Ki * i_error);
  return steer_value;
}

double PID::dpSum()
{
  //double sum = m_dp[0] + m_dp[1] + m_dp[2];
  double sum = m_dp[0] + m_dp[2];
  return sum;
}

double PID::TotalError()
{

}

