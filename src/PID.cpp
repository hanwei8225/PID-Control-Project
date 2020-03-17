#include "PID.h"
#include <cmath>
#include <iostream>
#include<limits>

using namespace std;
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_)
{
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  dkp = 0.1*Kp;
  dkd = 0.1*Kd;
  dki = 0.1*Ki;

  i_error = 0.0;
  p_error = 0.0;
  d_error = 0.0;
  pre_cte = 0.0;

  step = 0;
  stable_step = 100;
  err_step = 100;

  best_err = 0.0;
  err = numeric_limits<double>::max();

  p_index = 0;
  second_eval = false;

  twiddle = false;

  first_step = true;
}

void PID::UpdateError(double cte)
{
  /**
   * TODO: Update PID errors based on cte.
   */
  if (first_step)
  {
    d_error = cte;
    first_step = false;
  }else
  {
    p_error = cte;
    d_error = cte - pre_cte;
    i_error += cte;
    pre_cte = cte;
  }
  
  




  if (dkp+dki+dkd < 0.01)
  {
    twiddle = false;
  }
  
  // twiddle

  if (twiddle)
  {
  // 先让走stable_step步计算err
  if (step - stable_step > 0 && step - stable_step < err_step)
  {
    err += pow(p_error, 2);
  }

  //积累了一定的步数之后
  if (step == stable_step + err_step)
  {
    //如果是第一层的循环
    if (second_eval == false)
    {
      if (err < best_err)
      {
        best_err = err;
        Dp_step(p_index,1.1);
        second_eval = false;
        p_index += 1;
      }
      else
      {
        if (p_index % 3 == 0)
        {
          Kp -= dkp * 2;
        }
        else if (p_index % 3 == 1)
        {
          Kd -= dkd * 2;
        }
        else if (p_index % 3 == 2)
        {
          Ki -= dki * 2;
        }
        second_eval = true;
      }
    }
    //如果进入了第二层循环
    else
    {
      if (err < best_err)
      {
        best_err = err;
        Dp_step(p_index,1.1);
        second_eval = false;
      }
      else
      {
        Dp_step(p_index,0.9);
      }
      second_eval = false;
      p_index += 1;
    }

    if (p_index % 3 == 0)
    {
      Kp += dkp;
    }
    else if (p_index % 3 == 1)
    {
      Kd += dkd;
    }
    else if (p_index % 3 == 2)
    {
      Ki += dki;
    }
    step = 0;

    std::cout << "Kp:" << Kp << " Ki:" << Ki << " Kd:" << Kd << std::endl;
    std::cout << "dKp:" << dkp << " dKi:" << dki << " dKd:" << dkd << std::endl;
  }
  step += 1 ;
  }
}

double PID::TotalError()
{
  /**
   * TODO: Calculate and return the total error
   */

  double total = Kp * p_error + Kd * d_error + Ki * i_error;

  return total; // TODO: Add your total error calc here!
}

void PID::Dp_step(long p_index, double per){
  if (p_index % 3 == 0)
        {
          dkp *= per;
        }
        else if (p_index % 3 == 1)
        {
          dkd *= per;
        }
        else if (p_index % 3 == 2)
        {
          dki *= per;
        }

}
