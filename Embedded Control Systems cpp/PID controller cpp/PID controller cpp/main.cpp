//
//  main.cpp
//  PID controller cpp
//
//  Created by Stephen Aboasu on 6/11/21.
//

#include <cmath>

// Select 'double or 'float' ther:
typedef double real;

class PID_Controller
{
public:
    void Initialize(real kp, real ki, real kd,
                    real error_thresh, real step_time);
    real Update(real error);
    
private:
    bool m_started:
    real m_kp, m_ki, m_kd, m_h, m_inv_h, m_prev_error,
    m_error_thresh, m_integral;
};

void PID_Controller::Initialize(real kp, real ki, real kd,
                                real error_thresh, real step_time)
{
        // Initialize controller parameters
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
    m_error_thresh = error_thresh;
    
    // Controller step time and its inverse
    m_h = step_time;
    m_inv_h = 1/ step_time;
    
    // Initialize integral and derivative calculations
    m_integral = 0;
    m_started = false;
}

real PID_Controller::Update(real error)
{
    // Set q to 1 if the rror magnitude is below
    // the threshould and 0 otherwise
    real q;
    
    if (fabs(error) < m_error_thresh)
        q =1;
    else
        q = 0;
    
    // Update the error integral
    m_integral += m_h*q*error;
    
    //Compute the error derivative
    real driv;
    if (!m_started)
    {
        m_started = true;
        driv = 0;
    }
    else
        driv = (error - m_prev_error)* m_inv_h;
    
    m_prev_error = error;
    
    // Return the PID controller actuator comand
    return m_kp*(error+m_ki*m_integral + m_kd*deriv);
    
}
