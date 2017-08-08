#include "PID.h"

//using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double kp, double ki, double kd) {
  Kp = kp;
  Ki = ki;
  Kd = kd;

  int_cte = 0;
  is_initialized = false;

}

void PID::UpdateError(double cte) {


  // proportional error
  p_error = -Kp * cte;

  // differential error

  // initial case
  if (!is_initialized) { 
    prev_cte = cte;
    is_initialized = true;
  }

  d_error = -Kd * (cte - prev_cte);
  prev_cte = cte;

  // integral error
  int_cte += cte;
  i_error = -Ki * int_cte;

}

double PID::TotalError() {

  return (p_error + d_error + i_error);
}

